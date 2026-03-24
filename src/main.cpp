#include <Arduino.h>
#include <MAVLink_common.h>
#include <ESP32Servo.h>
#include <Wire.h>

#define LED_PIN 2 // LED-Pin
#define BATTERY_PIN 34 // Pin zur Akkuüberwachung

// Empfängerpins
#define RECV_RX 26
#define RECV_TX 27

// Motorpins
#define MOTOR_PIN_1 15 // links vorne
#define MOTOR_PIN_2 23 // links hinten
#define MOTOR_PIN_3 13 // rechts vorne
#define MOTOR_PIN_4 32 // rechts hinten

// MPU-I2C-Addresse
#define MPU_ADDR 0x68

// Stack-Speicher für die statische Erstellung reservieren
StaticTask_t xPIDTaskBuffer;
StackType_t xPIDStack[2048];
StaticTask_t xLogicTaskBuffer;
StackType_t xLogicStack[4096];

// ESP32-Servo Parameter
Servo motor_lf; // left front
Servo motor_lb; // left back
Servo motor_rf; // right front
Servo motor_rb; // right back
ESP32PWM pwm;
int minUs = 1000;
int maxUs = 2000;

// RC-Channels 
uint16_t rc_channels_buffer[16]; // Roll, Pitch, Throttle, Yaw, Arm-Switch,...
portMUX_TYPE rcMux = portMUX_INITIALIZER_UNLOCKED;
uint32_t last_tx_msg;

// PID-Loop Variablen
uint16_t rc_channels_cpy[16]; // kopierte RC-Kanäle
bool is_armed = false; // Motoren aktiviert
float gyroRoll, gyroPitch; // Grad mit Gyro
float accRoll, accPitch;  // Grad mit Beschleunigungssensor
float roll = 0, pitch = 0; // Grad (Komplementärfilter angewendet)
int16_t AcXcal = -123, AcYcal = -26, AcZcal = -363;
int16_t GyXcal = 35, GyYcal = -69, GyZcal = -7;
unsigned long lastMeasure; // Zeitpunkt 1
unsigned long thisMeasure; // Zeitpunkt 2
bool anti_windup; // zur Verhinderung von Überreaktionen
float pid_dt; // Zeit zwischen PID-Loop-Durchgängen
float ax, ay, az; // MPU-Rohdaten Beschleunigungssensor
float gx, gy, gz; // MPU-Rohdaten Gyroskop
float p_target_ang, r_target_ang; // Ziel-Winkel
float y_target_rot; // Ziel-Winkelgeschwindigkeit
uint32_t t_target; // Throttle
float r_ang_error, p_ang_error; // Winkelfehler
float r_rot_error, p_rot_error, y_rot_error; // Winkelgeschindigkeitsfehler
float p_result, i_result, d_result, pid_result; // Ergebnisse von pid_equation()
// PID-Werte und Ergebnisse
float r_pid_result, p_pid_result, y_pid_result;
float r_a_P, r_a_I, r_a_D, r_a_prev_error, r_a_prev_I;
float r_target_rot;
float r_P, r_I, r_D, r_prev_error, r_prev_I;
float p_a_P, p_a_I, p_a_D, p_a_prev_error, p_a_prev_I;
float p_target_rot;
float p_P, p_I, p_D, p_prev_error, p_prev_I;
float y_P, y_I, y_D, y_prev_error, y_prev_I;
// Motorgeschwindigkeiten
uint16_t motor_lf_speed, motor_lb_speed, motor_rf_speed, motor_rb_speed;

// Akkustatus
uint16_t raw_battery_voltage;
float battery_voltage;

// Funktionen
void MPU_init();
void motors_init();
void get_actual_pos();
void get_input_target();
void pid_equation(float error, float dt, float P, float I, float D, float prev_error, float prev_I);
void reset_pid_params();
void decode_messages();
void send_heartbeat();
void get_battery_voltage();


void PID_Loop(void *) {
  for(;;) { 
    // PID-Loop mit MPU6050, Motoransteuerung
    vTaskDelay(pdMS_TO_TICKS(2)); // ca. 380Hz Frequenz, weil Looptime +650us
    // Zeitdauer der Loop berechnen
    thisMeasure = micros();
    pid_dt = (thisMeasure - lastMeasure) / 1000000.0;
    lastMeasure = thisMeasure;

    portENTER_CRITICAL(&rcMux);
    memcpy(rc_channels_cpy, rc_channels_buffer, sizeof(rc_channels_buffer));
    portEXIT_CRITICAL(&rcMux);

    if (is_armed == true){
      if (rc_channels_cpy[4] < 1500) {is_armed = false;}
    } else {
      if ((rc_channels_cpy[2] < 1050) && (rc_channels_cpy[4] > 1500)) {is_armed = true;}
    }

    if (is_armed == true) {
      // Motoren aktiviert

      get_actual_pos(); // MPU
      get_input_target(); // aus RC-Input Winkel ausrechnen 

      // Anti-Windup
      if ((motor_lf_speed == 1150 || motor_lf_speed == 1999) ||
          (motor_lb_speed == 1150 || motor_lb_speed == 1999) ||
          (motor_rf_speed == 1150 || motor_rf_speed == 1999) ||
          (motor_rb_speed == 1150 || motor_rb_speed == 1999)) {
        anti_windup = true;
      } else {
        anti_windup = false;
      }

      // Roll-Berechnungen
      r_ang_error = r_target_ang + roll; // + wegen MPU-Richtung
      pid_equation(r_ang_error, pid_dt, r_a_P, r_a_I, r_a_D, r_a_prev_error, r_a_prev_I);
      r_a_prev_error = r_ang_error;
      r_a_prev_I = i_result;
      r_target_rot = pid_result; // diese Winkelgeschwindigkeit ist nahezu nie erreichbar, aber das muss sie nicht
      r_rot_error = r_target_rot + gx; // + wegen MPU-Richtung
      pid_equation(r_rot_error, pid_dt, r_P, r_I, r_D, r_prev_error, r_prev_I);
      r_prev_error = r_rot_error;
      r_prev_I = i_result;
      r_pid_result = pid_result;

      // Pitch-Berechnungen
      p_ang_error = p_target_ang + pitch;
      pid_equation(p_ang_error, pid_dt, p_a_P, p_a_I, p_a_D, p_a_prev_error, p_a_prev_I);
      p_a_prev_error = p_ang_error;
      p_a_prev_I = i_result;
      p_target_rot = pid_result;
      p_rot_error = p_target_rot + gy;
      pid_equation(p_rot_error, pid_dt, p_P, p_I, p_D, p_prev_error, p_prev_I);
      p_prev_error = p_rot_error;
      p_prev_I = i_result;
      p_pid_result = pid_result;

      // Yaw-Berechnungen
      y_rot_error = y_target_rot + gz; // nur PID für Winkelgeschwindigkeit
      pid_equation(y_rot_error, pid_dt, y_P, y_I, y_D, y_prev_error, y_prev_I);
      y_prev_error = y_rot_error;
      y_prev_I = i_result;
      y_pid_result = pid_result;

      // Throttle-Limit setzen (Spielraum für PID-Regler)
      if (t_target>1800) {
        t_target = 1800;
      }

      // Motor-Inputs berechnen
      // Pitch anders wegen MPU-Richtung
      motor_lf_speed = t_target + r_pid_result - p_pid_result - y_pid_result;
      motor_lb_speed = t_target + r_pid_result + p_pid_result + y_pid_result;
      motor_rf_speed = t_target - r_pid_result - p_pid_result + y_pid_result;
      motor_rb_speed = t_target - r_pid_result + p_pid_result - y_pid_result;
      if (motor_lf_speed > 2000) {motor_lf_speed = 1999;} 
      else if (motor_lf_speed < 1150) {motor_lf_speed = 1150;}
      if (motor_lb_speed > 2000) {motor_lb_speed = 1999;} 
      else if (motor_lb_speed < 1150) {motor_lb_speed = 1150;}
      if (motor_rf_speed > 2000) {motor_rf_speed = 1999;} 
      else if (motor_rf_speed < 1150) {motor_rf_speed = 1150;}
      if (motor_rb_speed > 2000) {motor_rb_speed = 1999;} 
      else if (motor_rb_speed < 1150) {motor_rb_speed = 1150;}
      motor_lf.writeMicroseconds(motor_lf_speed);
      motor_lb.writeMicroseconds(motor_lb_speed);
      motor_rf.writeMicroseconds(motor_rf_speed);
      motor_rb.writeMicroseconds(motor_rb_speed);
    } else {
      // Motoren deaktiviert
      motor_lf.writeMicroseconds(1000);
      motor_lb.writeMicroseconds(1000);
      motor_rf.writeMicroseconds(1000);
      motor_rb.writeMicroseconds(1000);
      reset_pid_params();
    }
  }
}

void Logic_Loop(void *) {
  for(;;) {
    // Empfänger, GPS, Kompass, Barometer, Steuerlogik
    vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz Frequenz
    get_battery_voltage();
    decode_messages();
    send_heartbeat();
  }
}

void setup() {
  Serial.begin(9600);

  Serial1.begin(460800, SERIAL_8N1, RECV_RX, RECV_TX);

  analogReadResolution(12);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  MPU_init();
  motors_init();

  // PID-Werte setzen
  r_a_P = 4; r_a_I = 0; r_a_D = 0;
  p_a_P = r_a_P; p_a_I = r_a_I; p_a_D = r_a_D;
  r_P = 0.7; r_I = 2; r_D = 0.04;
  p_P = r_P; p_I = r_I; p_D = r_D;
  y_P = 5; y_I = 10; y_D = 0;


  // PID Task auf Core 1 
  xTaskCreateStaticPinnedToCore(
    PID_Loop,         
    "PID_Loop-Task",  
    2048,             
    NULL,             
    2,                
    xPIDStack,        
    &xPIDTaskBuffer,  
    1                 
  );

  // Logic Task auf Core 0 
  xTaskCreateStaticPinnedToCore(
    Logic_Loop,
    "Logic_Loop-Task",
    4096,             
    NULL,
    1,                
    xLogicStack,
    &xLogicTaskBuffer,
    0                 
  );
}

void loop() { 
  vTaskDelete(NULL); 
}


void MPU_init() {
  Wire.begin(33, 25, 400000); // 400kHz (S.5 Datenblatt)

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00); // Sensor aktivieren (S.40 Datenblatt)
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); Wire.write(0x08); // Gyro auf +/- 500°/s (65.5 LSB/°/s) (S.14 Datenblatt)
  Wire.endTransmission();
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); Wire.write(0x10); // Accel auf +/- 8g (4096 LSB/g) (S.15 Datenblatt)
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A); Wire.write(0x04); // 4...21Hz Filterung gegen Vibrationen (S.13 Datenblatt)
  Wire.endTransmission();
}

void motors_init() {
  // ZW: ESP-Servo Setup
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
  // 500Hz PWM-Frequenz
	motor_lf.setPeriodHertz(500);      
	motor_lb.setPeriodHertz(500);
	motor_rf.setPeriodHertz(500);
	motor_rb.setPeriodHertz(500);

  motor_lf.attach(MOTOR_PIN_1, minUs, maxUs);
	motor_lb.attach(MOTOR_PIN_2, minUs, maxUs);
  motor_rf.attach(MOTOR_PIN_3, minUs, maxUs);
	motor_rb.attach(MOTOR_PIN_4, minUs, maxUs);

  // ESCs initialisieren
  motor_lf.writeMicroseconds(1000);
  motor_lb.writeMicroseconds(1000);
  motor_rf.writeMicroseconds(1000);
  motor_rb.writeMicroseconds(1000);
  delay(3000);
}

void get_actual_pos() { // MPU-Daten bekommen
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); // 14 aufeinander folgende Register abfragen

  // Daten lesen, Kalibrierung andwenden und skalieren
  ax = (int16_t(Wire.read()<<8|Wire.read()) + AcXcal) / 4096.0;
  ay = (int16_t(Wire.read()<<8|Wire.read()) + AcYcal) / 4096.0;
  az = (int16_t(Wire.read()<<8|Wire.read()) + AcZcal) / 4096.0;
  Wire.read()<<8|Wire.read(); // Temperatur überspringen
  gx = (int16_t(Wire.read()<<8|Wire.read()) + GyXcal) / 65.5;
  gy = (int16_t(Wire.read()<<8|Wire.read()) + GyYcal) / 65.5;
  gz = (int16_t(Wire.read()<<8|Wire.read()) + GyZcal) / 65.5;

  // Winkelberechnung mit Beschleundigungssensor
  accPitch  = atan2(-ax, sqrt(ay * ay + az * az)) * 57.296; 
  accRoll = atan2(ay, sqrt(ax * ax + az * az)) * 57.296; 

  // Komplementärfilter 
  pitch  = 0.98 * (pitch + gy * pid_dt) + 0.02 * accPitch;
  roll = 0.98 * (roll + gx * pid_dt) + 0.02 * accRoll;
}

void get_input_target() {
  // Roll 
  r_target_ang = 0.1*(rc_channels_cpy[0]-1500);

  // Pitch
  p_target_ang = 0.1*(rc_channels_cpy[1]-1500);

  // Throttle (nur Input)
  t_target = rc_channels_cpy[2];

  // Yaw (nur Winkelgeschwindigkeit)
  y_target_rot = 0.15*(rc_channels_cpy[3]-1500);
}

void pid_equation(float error, float dt, float P, float I, float D, float prev_error, float prev_I) {
  p_result = P * error;

  if (anti_windup == false) { 
    i_result = prev_I + (((prev_error+error) * dt)/2)*I;
    // I-Wert begrenzen
    if (i_result > 300) {i_result = 300;}
    else if (i_result < -300) {i_result = -300;}
  } else {
    i_result = prev_I;
  }

  d_result = D*((error - prev_error)/dt);

  pid_result = p_result + i_result + d_result;
  // Gesamtwert begrenzen
  if (pid_result > 400) {pid_result = 400;}
  else if (pid_result < -400) {pid_result = -400;}
}

void reset_pid_params() {
  // Roll
  r_a_prev_error = 0;
  r_a_prev_I = 0;
  r_prev_error = 0;
  r_prev_I = 0;
  // Pitch
  p_a_prev_error = 0;
  p_a_prev_I = 0;
  p_prev_error = 0;
  p_prev_I = 0;
  // Yaw
  y_prev_error = 0;
  y_prev_I = 0;
}

void send_heartbeat() {
  static uint32_t last_time = millis();
  const uint32_t current_time = millis();
  constexpr const uint32_t heartbeat_interval = 1000;
  if (current_time - last_time > heartbeat_interval) {
    last_time = current_time;

    static mavlink_message_t mavlink_message;
    static uint8_t mavlink_message_buffer[MAVLINK_MAX_PACKET_LEN];
    static uint16_t mavlink_message_length = 0;

    if (mavlink_message_length == 0) { 
      const int system_id = 1;
      const int component_id = 1;
      const int mavlink_type = MAV_TYPE_GENERIC;
      const int autopilot_type = MAV_AUTOPILOT_INVALID;
      const int system_mode = MAV_MODE_PREFLIGHT;
      const int custom_mode = 0x0000; // No flag
      const int mavlink_state = MAV_STATE_ACTIVE;
      mavlink_msg_heartbeat_pack(
        system_id, component_id, &mavlink_message, mavlink_type, autopilot_type, system_mode, custom_mode, mavlink_state
      );
      mavlink_message_length = mavlink_msg_to_send_buffer(mavlink_message_buffer, &mavlink_message);
   }
    Serial1.write(mavlink_message_buffer, mavlink_message_length);
  }
}

void decode_messages() {
  static mavlink_message_t message;
  static mavlink_status_t status;

  while(Serial1.available() > 0) {
    uint8_t serial_byte = Serial1.read();

    // Testen, ob Nachricht vollständig
    if(mavlink_parse_char(MAVLINK_COMM_1, serial_byte, &message, &status)) {

      // Nachricht auswerten
      switch(message.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            // Heartbeat wurde empfangen
            break;
        case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
          mavlink_rc_channels_override_t rc;
          mavlink_msg_rc_channels_override_decode(&message, &rc);

          portENTER_CRITICAL(&rcMux);
          rc_channels_buffer[0]=rc.chan1_raw;
          rc_channels_buffer[1]=rc.chan2_raw;
          rc_channels_buffer[2]=rc.chan3_raw;
          rc_channels_buffer[3]=rc.chan4_raw;
          rc_channels_buffer[4]=rc.chan5_raw;
          rc_channels_buffer[5]=rc.chan6_raw;
          rc_channels_buffer[6]=rc.chan7_raw;
          rc_channels_buffer[7]=rc.chan8_raw;
          portEXIT_CRITICAL(&rcMux);

          last_tx_msg = millis();

          break;
        default:
          break;
      }
    }
    else {
      // warten auf genug Bytes
    }
  }
  // keine Verbindung
  if ((millis() - last_tx_msg) > 500) {
    portENTER_CRITICAL(&rcMux);
    rc_channels_buffer[4] = 1000; 
    portEXIT_CRITICAL(&rcMux);
  }
}

void get_battery_voltage() {
  raw_battery_voltage = analogRead(BATTERY_PIN);
  battery_voltage = (raw_battery_voltage * 3.3) / 4095.0; // in Volt umrechnen
  battery_voltage = battery_voltage * 3; // mal 3, wegen Spannungsteiler
  if (battery_voltage <= 7.1) { 
    // Akku ist fast leer
    digitalWrite(LED_PIN, HIGH);
  }
}
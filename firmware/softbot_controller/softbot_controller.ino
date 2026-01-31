/**
 * @file softbot_controller_v15_max_attack.ino
 * @brief Controlador v15 - "MAXIMUM ATTACK" (Inflado Brusco Restaurado).
 * @details
 * - Lógica Híbrida Agresiva: Si faltan > 5 kPa, PWM = 255 a TODO.
 * - Sin suavizado PID hasta el último momento.
 * - Hardware: 4 Bombas activas (Inflado Aux en PIN 5).
 */

#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h> 
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/int16_multi_array.h>

// ==========================================
// 1. HARDWARE
// ==========================================
const int PIN_VALVE_INFLATE = 25;
const int PIN_VALVE_SUCTION = 26;
const int PIN_VALVE_BOOST   = 23; // AJUSTA segun tu conexion
const int PIN_MUX_CHAMBER_A = 32; 
const int PIN_MUX_CHAMBER_B = 33; 

// Bombas
const int PIN_PUMP_INFLATE_MAIN = 19;      
const int PIN_PUMP_INFLATE_AUX  = 4;   

const int PIN_PUMP_SUCTION_MAIN = 18; 
const int PIN_PUMP_SUCTION_AUX  = 27;  

// Canales PWM
const int CH_INFLATE_MAIN = 0;
const int CH_SUCCION_MAIN = 1;
const int CH_SUCCION_AUX  = 2; 
const int CH_INFLATE_AUX  = 3; 

const int PWM_FREQ = 1000;
const int PWM_RES  = 8;
const int PWM_MAX  = 255;

// UMBRALES DE AGRESIVIDAD
// Si el error es mayor a esto, ignoramos el PID y damos 100%
const float AGGRESSIVE_THRESHOLD = 5.0f; 
const int THRESHOLD_TURBO = 40; 

const int PIN_I2C_SDA = 21;
const int PIN_I2C_SCL = 22;
const int PIN_LED_STATUS = 2; 

// ==========================================
// 2. PARÁMETROS DE CONTROL
// ==========================================
float Kp_neg = -75.00f;  float Ki_neg = -750.00f; 
float Kp_pos = 24.0f;    float Ki_pos = 1500.0f; 

const float TS_SECONDS = 0.020f; 
const int   TS_MS      = 20;

float integral_pos_sum = 0.0f;
float integral_neg_sum = 0.0f;
float safety_limit_max = 55.0f;  
float safety_limit_min = -60.0f; 

float setpoint_pressure = 0.0f;
float current_pressure  = 0.0f;
int8_t control_mode     = 0; 
int8_t active_chamber   = 0; 

bool emergency_stop_active = false;
unsigned long last_control_timestamp = 0;
bool boost_enabled = false;

Adafruit_ADS1115 ads;
const float V_OFFSET  = 2.5f;    
const float V_SENSITIVITY = 0.02f; 

// ==========================================
// 2.1 TANQUE (BOOST) - ESTADO
// ==========================================
const float TANK_TOL_KPA = 1.0f;           // tolerancia para considerar "lleno"
const uint32_t TANK_STABLE_MS = 500;       // tiempo estable en tolerancia
const uint32_t TANK_TIMEOUT_MS = 12000;    // timeout de llenado

bool tank_fill_active = false;
bool tank_full_latched = false;
uint32_t tank_fill_start_ms = 0;
uint32_t tank_within_since_ms = 0;
int8_t tank_state = 0; // 0=idle, 1=filling, 2=full, 3=timeout

// ==========================================
// 3. MICRO-ROS
// ==========================================
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_subscription_t sub_setpoint;
rcl_subscription_t sub_mode;
rcl_subscription_t sub_chamber;
rcl_subscription_t sub_tuning; 
rcl_subscription_t sub_boost;

rcl_publisher_t pub_feedback;
rcl_publisher_t pub_debug; 
rcl_publisher_t pub_tank_state;

std_msgs__msg__Float32 msg_setpoint;
std_msgs__msg__Int8    msg_mode;
std_msgs__msg__Int8    msg_chamber;
std_msgs__msg__Int8    msg_boost;
std_msgs__msg__Float32MultiArray msg_tuning;
std_msgs__msg__Float32 msg_feedback;
std_msgs__msg__Int16MultiArray msg_debug; 
std_msgs__msg__Int8 msg_tank_state;

int16_t debug_data[4]; 
float tuning_buffer[6]; 

void fatal_error_loop(int code){
  while(1){ for(int i=0; i<code; i++){ digitalWrite(PIN_LED_STATUS, HIGH); delay(150); digitalWrite(PIN_LED_STATUS, LOW); delay(150); } delay(1500); }
}
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){fatal_error_loop(1);}}

// ==========================================
// FUNCIONES
// ==========================================
float readPressureSensor() {
  int16_t adc = ads.readADC_SingleEnded(0);
  float volts = ads.computeVolts(adc);
  return (volts - V_OFFSET) / V_SENSITIVITY;
}

void stopActuators() {
  ledcWrite(CH_INFLATE_MAIN, 0); ledcWrite(CH_INFLATE_AUX, 0);
  ledcWrite(CH_SUCCION_MAIN, 0); ledcWrite(CH_SUCCION_AUX, 0);
  digitalWrite(PIN_VALVE_INFLATE, LOW); digitalWrite(PIN_VALVE_SUCTION, LOW);
  digitalWrite(PIN_VALVE_BOOST, LOW);
  digitalWrite(PIN_MUX_CHAMBER_A, LOW); digitalWrite(PIN_MUX_CHAMBER_B, LOW);
}

void triggerEmergencyStop() {
  ledcWrite(CH_INFLATE_MAIN, 0); ledcWrite(CH_INFLATE_AUX, 0);
  ledcWrite(CH_SUCCION_MAIN, 0); ledcWrite(CH_SUCCION_AUX, 0);
  digitalWrite(PIN_VALVE_INFLATE, HIGH); digitalWrite(PIN_VALVE_SUCTION, HIGH);
  digitalWrite(PIN_VALVE_BOOST, LOW);
  digitalWrite(PIN_MUX_CHAMBER_A, HIGH); digitalWrite(PIN_MUX_CHAMBER_B, HIGH);
  delay(2000); 
  stopActuators();
  emergency_stop_active = true;
  control_mode = 0;
  boost_enabled = false;
  tank_fill_active = false;
  tank_full_latched = false;
  tank_state = 0;
}

float computePID_Positive(float error) {
  integral_pos_sum += error * TS_SECONDS;
  float i_term = Ki_pos * integral_pos_sum;
  if (i_term > PWM_MAX) integral_pos_sum = PWM_MAX / Ki_pos;
  return (Kp_pos * error) + (Ki_pos * integral_pos_sum);
}

float computePID_Negative(float error) {
  integral_neg_sum += error * TS_SECONDS;
  float i_term = Ki_neg * integral_neg_sum;
  if (i_term < -PWM_MAX) integral_neg_sum = -PWM_MAX / Ki_neg;
  return (Kp_neg * error) + (Ki_neg * integral_neg_sum);
}

// ==========================================
// CALLBACKS
// ==========================================
void cb_setpoint(const void * msgin) { if (!emergency_stop_active) setpoint_pressure = ((const std_msgs__msg__Float32 *)msgin)->data; }
void cb_chamber(const void * msgin) { if (!emergency_stop_active) active_chamber = ((const std_msgs__msg__Int8 *)msgin)->data; }

void cb_mode(const void * msgin) {
  int8_t m = ((const std_msgs__msg__Int8 *)msgin)->data;
  if (emergency_stop_active && m == 0) emergency_stop_active = false;
  if (emergency_stop_active) return;
  if (m == 0) boost_enabled = false;
  if (m != control_mode) { 
      integral_pos_sum = 0; 
      integral_neg_sum = 0; 
  }
  control_mode = m;

  // Inicializar llenado de tanque si entra en modo 3
  if (control_mode == 3) {
    tank_fill_active = true;
    tank_full_latched = false;
    tank_fill_start_ms = millis();
    tank_within_since_ms = 0;
    tank_state = 1;
  }
}

void cb_tuning(const void * msgin) {
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  if (msg->data.size >= 6) {
    Kp_pos = msg->data.data[0]; Ki_pos = msg->data.data[1];
    Kp_neg = msg->data.data[2]; Ki_neg = msg->data.data[3];
    safety_limit_max = msg->data.data[4]; safety_limit_min = msg->data.data[5];
    integral_pos_sum = 0; integral_neg_sum = 0;
  }
}

void cb_boost(const void * msgin) {
  if (emergency_stop_active) { boost_enabled = false; return; }
  int8_t b = ((const std_msgs__msg__Int8 *)msgin)->data;
  boost_enabled = (b != 0);
}

// ==========================================
// LOOP DE CONTROL
// ==========================================
void controlLoop() {
  current_pressure = readPressureSensor();

  if (current_pressure > safety_limit_max || current_pressure < safety_limit_min) {
    triggerEmergencyStop();
  }
  
  msg_feedback.data = current_pressure;
  rcl_publish(&pub_feedback, &msg_feedback, NULL);

  if (emergency_stop_active) {
      debug_data[0] = -1; debug_data[1] = -1; debug_data[2] = 0; debug_data[3] = 0;
      msg_debug.data.data = debug_data; msg_debug.data.size = 4;
      rcl_publish(&pub_debug, &msg_debug, NULL);
      return;
  }

  if (control_mode == 0 || active_chamber == 0) {
    boost_enabled = false;
  }
  digitalWrite(PIN_VALVE_BOOST, boost_enabled ? HIGH : LOW);

  // Selección de cámara (en llenado de tanque, siempre cerrado)
  if (control_mode == 3) {
    digitalWrite(PIN_MUX_CHAMBER_A, LOW);
    digitalWrite(PIN_MUX_CHAMBER_B, LOW);
  } else {
    int8_t chamber_sel = active_chamber;
    if (control_mode == 4 && chamber_sel == 0) {
      chamber_sel = 3; // en venteo, 0 => ventear ambas
    }
    switch (chamber_sel) {
      case 1: digitalWrite(PIN_MUX_CHAMBER_A, HIGH); digitalWrite(PIN_MUX_CHAMBER_B, LOW); break;
      case 2: digitalWrite(PIN_MUX_CHAMBER_A, LOW); digitalWrite(PIN_MUX_CHAMBER_B, HIGH); break;
      case 3: digitalWrite(PIN_MUX_CHAMBER_A, HIGH); digitalWrite(PIN_MUX_CHAMBER_B, HIGH); break;
      default: digitalWrite(PIN_MUX_CHAMBER_A, LOW); digitalWrite(PIN_MUX_CHAMBER_B, LOW); break;
    }
  }

  float error = setpoint_pressure - current_pressure;
  int pwm_main = 0;
  int pwm_aux = 0;

  if (control_mode == 4) { // VENTEAR (liberar presion a atmosfera)
    boost_enabled = false;
    digitalWrite(PIN_VALVE_BOOST, LOW);

    // Bombas apagadas
    ledcWrite(CH_INFLATE_MAIN, 0); ledcWrite(CH_INFLATE_AUX, 0);
    ledcWrite(CH_SUCCION_MAIN, 0); ledcWrite(CH_SUCCION_AUX, 0);

    // Válvulas desenergizadas (A->R abierto / P cerrado)
    digitalWrite(PIN_VALVE_INFLATE, LOW);
    digitalWrite(PIN_VALVE_SUCTION, LOW);
  }
  else if (control_mode == 0 || (active_chamber == 0 && control_mode != 3)) {
    stopActuators();
    integral_pos_sum = 0; integral_neg_sum = 0;
  }
  else if (control_mode == 3) { // LLENAR TANQUE (modo especial)
    // Siempre cerrar boost durante el llenado
    boost_enabled = false;
    digitalWrite(PIN_VALVE_BOOST, LOW);

    digitalWrite(PIN_VALVE_INFLATE, HIGH); digitalWrite(PIN_VALVE_SUCTION, LOW);
    ledcWrite(CH_SUCCION_MAIN, 0); ledcWrite(CH_SUCCION_AUX, 0);

    if (error > AGGRESSIVE_THRESHOLD) {
        pwm_main = 255;
        pwm_aux  = 255;
        integral_pos_sum = 0;
    } 
    else {
        float pid_out = computePID_Positive(error);
        if (pid_out < 0) pid_out = 0; 
        if (pid_out > PWM_MAX) pwm_main = PWM_MAX; else pwm_main = (int)pid_out;
        if (pwm_main > THRESHOLD_TURBO) pwm_aux = pwm_main; else pwm_aux = 0;
    }

    ledcWrite(CH_INFLATE_MAIN, pwm_main);
    ledcWrite(CH_INFLATE_AUX, pwm_aux); 

    // Estado de llenado
    if (fabs(error) <= TANK_TOL_KPA) {
      if (tank_within_since_ms == 0) tank_within_since_ms = millis();
      if (millis() - tank_within_since_ms >= TANK_STABLE_MS) {
        tank_state = 2; // full
        tank_full_latched = true;
        control_mode = 0; // detener
        tank_fill_active = false;
        stopActuators();
      }
    } else {
      tank_within_since_ms = 0;
      tank_state = 1;
    }

    if ((millis() - tank_fill_start_ms) >= TANK_TIMEOUT_MS) {
      tank_state = 3; // timeout
      control_mode = 0;
      tank_fill_active = false;
      stopActuators();
    }
  }
  else if (control_mode == 1) { // INFLAR (MAX ATTACK)
    digitalWrite(PIN_VALVE_INFLATE, HIGH); digitalWrite(PIN_VALVE_SUCTION, LOW);
    ledcWrite(CH_SUCCION_MAIN, 0); ledcWrite(CH_SUCCION_AUX, 0);

    // 1. ZONA EXPLOSIVA (Bang-Bang Puro)
    if (error > AGGRESSIVE_THRESHOLD) {
        pwm_main = 255;
        pwm_aux  = 255; // ¡AMBAS A TOPE SIN PENSARLO!
        integral_pos_sum = 0; 
    } 
    else {
        // 2. ATERRIZAJE (PID)
        float pid_out = computePID_Positive(error);
        if (pid_out < 0) pid_out = 0; 
        if (pid_out > PWM_MAX) pwm_main = PWM_MAX; else pwm_main = (int)pid_out;
        
        // Espejo en PID
        if (pwm_main > THRESHOLD_TURBO) pwm_aux = pwm_main; else pwm_aux = 0;
    }
    
    ledcWrite(CH_INFLATE_MAIN, pwm_main);
    ledcWrite(CH_INFLATE_AUX, pwm_aux); 
  } 
  else if (control_mode == -1) { // SUCCIONAR (MAX ATTACK)
    digitalWrite(PIN_VALVE_INFLATE, LOW); digitalWrite(PIN_VALVE_SUCTION, HIGH);
    ledcWrite(CH_INFLATE_MAIN, 0); ledcWrite(CH_INFLATE_AUX, 0);

    if (error < -AGGRESSIVE_THRESHOLD) {
        pwm_main = 255;
        pwm_aux  = 255;
        integral_neg_sum = 0;
    }
    else {
        float pid_out = computePID_Negative(error);
        if (pid_out < 0) pid_out = 0; 
        if (pid_out > PWM_MAX) pwm_main = PWM_MAX; else pwm_main = (int)pid_out;
    
        if (pwm_main > THRESHOLD_TURBO) pwm_aux = pwm_main; else pwm_aux = 0;
    }
    
    ledcWrite(CH_SUCCION_MAIN, pwm_main);
    ledcWrite(CH_SUCCION_AUX, pwm_aux);
  }
  // MODOS MANUALES
  else if (abs(control_mode) == 2) {
     pwm_main = constrain((int)setpoint_pressure, 0, 255);
     
     // Espejo Directo en Manual
     if (pwm_main > THRESHOLD_TURBO) pwm_aux = pwm_main; else pwm_aux = 0;

     if (control_mode == 2) { // Inflar
        digitalWrite(PIN_VALVE_INFLATE, HIGH); digitalWrite(PIN_VALVE_SUCTION, LOW);
        ledcWrite(CH_SUCCION_MAIN, 0); ledcWrite(CH_SUCCION_AUX, 0);
        ledcWrite(CH_INFLATE_MAIN, pwm_main);
        ledcWrite(CH_INFLATE_AUX, pwm_aux);
     } else { // Succión
        digitalWrite(PIN_VALVE_INFLATE, LOW); digitalWrite(PIN_VALVE_SUCTION, HIGH);
        ledcWrite(CH_INFLATE_MAIN, 0); ledcWrite(CH_INFLATE_AUX, 0);
        ledcWrite(CH_SUCCION_MAIN, pwm_main);
        ledcWrite(CH_SUCCION_AUX, pwm_aux);
     }
  }

  debug_data[0] = pwm_main; debug_data[1] = pwm_aux; debug_data[2] = (int16_t)(error * 10); debug_data[3] = (int16_t)control_mode;
  msg_debug.data.data = debug_data; msg_debug.data.size = 4;
  rcl_publish(&pub_debug, &msg_debug, NULL);

  // Publicar estado del tanque
  if (control_mode != 3 && !tank_full_latched && !tank_fill_active) {
    tank_state = 0; // idle
  }
  msg_tank_state.data = tank_state;
  rcl_publish(&pub_tank_state, &msg_tank_state, NULL);
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  if (!ads.begin()) { /* Error */ }
  ads.setGain(GAIN_ONE);

  pinMode(PIN_VALVE_INFLATE, OUTPUT); pinMode(PIN_VALVE_SUCTION, OUTPUT);
  pinMode(PIN_VALVE_BOOST, OUTPUT);
  pinMode(PIN_MUX_CHAMBER_A, OUTPUT); pinMode(PIN_MUX_CHAMBER_B, OUTPUT);
  pinMode(PIN_LED_STATUS, OUTPUT);
  
  ledcSetup(CH_INFLATE_MAIN, PWM_FREQ, PWM_RES); ledcAttachPin(PIN_PUMP_INFLATE_MAIN, CH_INFLATE_MAIN);
  ledcSetup(CH_INFLATE_AUX, PWM_FREQ, PWM_RES); ledcAttachPin(PIN_PUMP_INFLATE_AUX, CH_INFLATE_AUX); // PIN 5
  ledcSetup(CH_SUCCION_MAIN, PWM_FREQ, PWM_RES); ledcAttachPin(PIN_PUMP_SUCTION_MAIN, CH_SUCCION_MAIN);
  ledcSetup(CH_SUCCION_AUX, PWM_FREQ, PWM_RES); ledcAttachPin(PIN_PUMP_SUCTION_AUX, CH_SUCCION_AUX);

  stopActuators();

  set_microros_transports();
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "soft_robot_node", "", &support));

  RCCHECK(rclc_subscription_init_default(&sub_setpoint, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "pressure_setpoint"));
  RCCHECK(rclc_subscription_init_default(&sub_mode, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "pressure_mode"));
  RCCHECK(rclc_subscription_init_default(&sub_chamber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "active_chamber"));
  RCCHECK(rclc_subscription_init_default(&sub_tuning, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "tuning_params"));
  RCCHECK(rclc_subscription_init_default(&sub_boost, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "boost_valve"));

  RCCHECK(rclc_publisher_init_default(&pub_feedback, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "pressure_feedback"));
  RCCHECK(rclc_publisher_init_default(&pub_debug, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray), "system_debug"));
  RCCHECK(rclc_publisher_init_default(&pub_tank_state, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "tank_state"));

  msg_debug.data.capacity = 4; msg_debug.data.size = 4; msg_debug.data.data = debug_data;
  msg_tuning.data.capacity = 6; msg_tuning.data.size = 6; msg_tuning.data.data = tuning_buffer;

  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_setpoint, &msg_setpoint, &cb_setpoint, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_mode, &msg_mode, &cb_mode, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_chamber, &msg_chamber, &cb_chamber, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_tuning, &msg_tuning, &cb_tuning, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_boost, &msg_boost, &cb_boost, ON_NEW_DATA));
  
  last_control_timestamp = millis();
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
  unsigned long now = millis();
  
  if (now - last_control_timestamp >= TS_MS) {
    last_control_timestamp += TS_MS;
    controlLoop();
  }
}

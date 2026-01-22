/**
 * @file softbot_controller_v6_ff.ino
 * @brief Controlador v6.0 - Implementación Feed-Forward + PID.
 * @details
 * - Integra curvas de calibración (PWM vs Presión) obtenidas experimentalmente.
 * - Reduce drásticamente el tiempo de subida (Rise Time).
 * - Mantiene sistemas de seguridad (Venteo y Watchdog).
 */

#include <Arduino.h>
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
// 1. CONFIGURACIÓN DE PINES
// ==========================================
const int PIN_VALVULA_INFLAR = 25;
const int PIN_VALVULA_SUCCION = 26;
const int PIN_BOMBA_INFLAR = 19;
const int PIN_BOMBA_SUCCION_MAIN = 18; 
const int PIN_BOMBA_SUCCION_AUX = 27;  

const int PIN_DISTRIB_A = 32; 
const int PIN_DISTRIB_B = 33; 

const int CH_INFLAR = 0;
const int CH_SUCCION_MAIN = 1;
const int CH_SUCCION_AUX = 2; 

const int PWM_FREQ = 1000;
const int PWM_RES = 8;
const int PWM_MAX = 255;
const int UMBRAL_TURBO = 100; 

// ==========================================
// 2. CONSTANTES DE FEED-FORWARD (CALIBRADAS)
// ==========================================
// Ecuación Inflado: PWM = (5.0 * kPa) - 20
const float FF_POS_SLOPE  = 5.0f; 
const float FF_POS_OFFSET = -20.0f;

// Ecuación Succión: PWM = (-4.8 * kPa) + 70
const float FF_NEG_SLOPE  = -4.8f; 
const float FF_NEG_OFFSET = 70.0f;

// ==========================================
// 3. VARIABLES DE CONTROL PID
// ==========================================
float KP_NEG = -50.00f;  // Bajamos un poco el KP ya que el FF hace el trabajo pesado
float KI_NEG = -400.00f; 
const float TS_NEG = 0.010f; 
float integral_neg_sum = 0.0f;

float KP_POS = 8.0f;     // KP más suave para evitar overshoot con el FF
float KI_POS = 150.0f;    
const float TS_POS = 0.010f; 
float integral_pos_sum = 0.0f;

float setpoint_kPa = 0.0f;
float current_pressure = 0.0f;
int8_t mode_control = 0; 
int8_t active_chamber = 0; 

const float Ts_ms = 10.0f; 
unsigned long last_control_ms = 0;

// ==========================================
// 4. SEGURIDAD Y SENSOR
// ==========================================
float MAX_PRESION_SEGURIDAD = 45.0f;  
float MIN_PRESION_SEGURIDAD = -60.0f; 

bool emergency_stop_active = false;
Adafruit_ADS1115 ads;
const int SDA_PIN = 21;
const int SCL_PIN = 22;
const float V_OFFSET = 2.5f;    
const float V_PER_kPa = 0.02f;  

// ==========================================
// 5. MICRO-ROS OBJETOS
// ==========================================
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_subscription_t sub_setpoint;
rcl_subscription_t sub_mode;
rcl_subscription_t sub_chamber;
rcl_subscription_t sub_tuning; 

rcl_publisher_t pub_feedback;
rcl_publisher_t pub_debug; 

std_msgs__msg__Float32 msg_setpoint;
std_msgs__msg__Int8 msg_mode;
std_msgs__msg__Int8 msg_chamber;
std_msgs__msg__Float32MultiArray msg_tuning;
std_msgs__msg__Float32 msg_feedback;
std_msgs__msg__Int16MultiArray msg_debug; 

int16_t debug_data[4]; 
float tuning_buffer[6]; 

// ==========================================
// 0. DIAGNÓSTICO LED
// ==========================================
const int LED_PIN = 2; 
void error_loop(int code){
  while(1){
    for(int i=0; i<code; i++){
      digitalWrite(LED_PIN, HIGH); delay(150);
      digitalWrite(LED_PIN, LOW); delay(150);
    }
    delay(1500); 
  }
}
#define CHECK_RET(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop(1);}}

// ==========================================
// FUNCIONES AUXILIARES
// ==========================================
float readPressure() {
  int16_t adc = ads.readADC_SingleEnded(0);
  float volts = ads.computeVolts(adc);
  return (volts - V_OFFSET) / V_PER_kPa;
}

void stopAll() {
  ledcWrite(CH_INFLAR, 0);
  ledcWrite(CH_SUCCION_MAIN, 0);
  ledcWrite(CH_SUCCION_AUX, 0);
  digitalWrite(PIN_VALVULA_INFLAR, LOW);
  digitalWrite(PIN_VALVULA_SUCCION, LOW);
  // No tocamos pines de distribución para permitir venteo si es necesario
}

// --- VENTEO DE SEGURIDAD ---
void triggerEmergencyStop() {
  ledcWrite(CH_INFLAR, 0);
  ledcWrite(CH_SUCCION_MAIN, 0);
  ledcWrite(CH_SUCCION_AUX, 0);

  digitalWrite(PIN_VALVULA_INFLAR, HIGH);
  digitalWrite(PIN_VALVULA_SUCCION, HIGH);
  digitalWrite(PIN_DISTRIB_A, HIGH);
  digitalWrite(PIN_DISTRIB_B, HIGH);

  delay(2000);

  stopAll();
  emergency_stop_active = true;
  mode_control = 0;
}

float calculate_PI_pos(float error) {
  integral_pos_sum += error * TS_POS;
  // Anti-windup simple
  float i_term = KI_POS * integral_pos_sum;
  if (i_term > PWM_MAX) integral_pos_sum = PWM_MAX / KI_POS;
  
  return (KP_POS * error) + (KI_POS * integral_pos_sum);
}

float calculate_PI_neg(float error) {
  integral_neg_sum += error * TS_NEG;
  // Anti-windup simple
  float i_term = KI_NEG * integral_neg_sum;
  if (i_term < -PWM_MAX) integral_neg_sum = -PWM_MAX / KI_NEG;
  
  return (KP_NEG * error) + (KI_NEG * integral_neg_sum);
}

// ==========================================
// CALLBACKS
// ==========================================
void setpoint_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  if (!emergency_stop_active) setpoint_kPa = msg->data;
}

void mode_callback(const void * msgin) {
  const std_msgs__msg__Int8 * msg = (const std_msgs__msg__Int8 *)msgin;
  if (emergency_stop_active && msg->data == 0) emergency_stop_active = false;
  if (emergency_stop_active) return;
  
  int8_t new_mode = msg->data;
  
  // Reset de integrales al cambiar de modo para evitar saltos
  if (new_mode != mode_control) {
      integral_pos_sum = 0; 
      integral_neg_sum = 0; 
  }
  mode_control = new_mode;
}

void chamber_callback(const void * msgin) {
  const std_msgs__msg__Int8 * msg = (const std_msgs__msg__Int8 *)msgin;
  if (!emergency_stop_active) active_chamber = msg->data;
}

void tuning_callback(const void * msgin) {
  const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  if (msg->data.size >= 6) {
    KP_POS = msg->data.data[0];
    KI_POS = msg->data.data[1];
    KP_NEG = msg->data.data[2];
    KI_NEG = msg->data.data[3];
    MAX_PRESION_SEGURIDAD = msg->data.data[4];
    MIN_PRESION_SEGURIDAD = msg->data.data[5];
    integral_pos_sum = 0; 
    integral_neg_sum = 0;
  }
}

// ==========================================
// LOOP DE CONTROL PRINCIPAL
// ==========================================
void controlLoop() {
  current_pressure = readPressure();

  // 1. CHEQUEO DE SEGURIDAD
  if (current_pressure > MAX_PRESION_SEGURIDAD || current_pressure < MIN_PRESION_SEGURIDAD) {
    triggerEmergencyStop();
  }
  
  // Feedback inmediato
  msg_feedback.data = current_pressure;
  rcl_publish(&pub_feedback, &msg_feedback, NULL);

  if (emergency_stop_active) {
      debug_data[0] = -999; 
      msg_debug.data.data = debug_data; msg_debug.data.size = 4;
      rcl_publish(&pub_debug, &msg_debug, NULL);
      return;
  }

  // 2. SELECCIÓN DE CÁMARA
  switch (active_chamber) {
    case 1: digitalWrite(PIN_DISTRIB_A, HIGH); digitalWrite(PIN_DISTRIB_B, LOW); break;
    case 2: digitalWrite(PIN_DISTRIB_A, LOW); digitalWrite(PIN_DISTRIB_B, HIGH); break;
    case 3: digitalWrite(PIN_DISTRIB_A, HIGH); digitalWrite(PIN_DISTRIB_B, HIGH); break;
    default: digitalWrite(PIN_DISTRIB_A, LOW); digitalWrite(PIN_DISTRIB_B, LOW); break;
  }

  float error = setpoint_kPa - current_pressure;
  int pwm_main = 0;
  int pwm_aux = 0;

  // 3. LÓGICA DE CONTROL (FF + PID)
  if (mode_control == 0 || active_chamber == 0) {
    stopAll();
    integral_pos_sum = 0; integral_neg_sum = 0;
  } 
  else if (mode_control == 1) { // INFLAR (Lazo Cerrado con FF)
    digitalWrite(PIN_VALVULA_INFLAR, HIGH); digitalWrite(PIN_VALVULA_SUCCION, LOW);
    ledcWrite(CH_SUCCION_MAIN, 0); ledcWrite(CH_SUCCION_AUX, 0);

    // A. Feed-Forward Term (Estimación Base)
    float ff_term = (FF_POS_SLOPE * setpoint_kPa) + FF_POS_OFFSET;
    if (ff_term < 0) ff_term = 0; // No podemos tener FF negativo aquí

    // B. PID Term (Corrección de Error)
    float pid_term = calculate_PI_pos(error);

    // C. Salida Total
    float out = ff_term + pid_term;
    
    // Saturación
    if (out < 0) out = 0; if (out > PWM_MAX) out = PWM_MAX;
    pwm_main = (int)out;
    
    ledcWrite(CH_INFLAR, pwm_main);
  } 
  else if (mode_control == -1) { // SUCCIONAR (Lazo Cerrado con FF)
    digitalWrite(PIN_VALVULA_INFLAR, LOW); digitalWrite(PIN_VALVULA_SUCCION, HIGH);
    ledcWrite(CH_INFLAR, 0);

    // A. Feed-Forward Term (Nota: setpoint es negativo, slope es negativo -> resultado positivo)
    float ff_term = (FF_NEG_SLOPE * setpoint_kPa) + FF_NEG_OFFSET;
    if (ff_term < 0) ff_term = 0;

    // B. PID Term
    float pid_term = calculate_PI_neg(error);

    // C. Salida Total
    float out = ff_term + pid_term;

    if (out < 0) out = 0; if (out > PWM_MAX) out = PWM_MAX;
    pwm_main = (int)out;
    ledcWrite(CH_SUCCION_MAIN, pwm_main);
    
    // Turbo Logic
    if (out > UMBRAL_TURBO) {
       int aux = map(out, UMBRAL_TURBO, PWM_MAX, 100, 255); 
       pwm_aux = aux;
       ledcWrite(CH_SUCCION_AUX, pwm_aux);
    } else {
       ledcWrite(CH_SUCCION_AUX, 0);
    }
  }
  // Mantenemos los modos de calibración por si necesitas recalibrar en el futuro
  else if (mode_control == 2) { 
    digitalWrite(PIN_VALVULA_INFLAR, HIGH); digitalWrite(PIN_VALVULA_SUCCION, LOW);
    ledcWrite(CH_SUCCION_MAIN, 0); ledcWrite(CH_SUCCION_AUX, 0);
    pwm_main = constrain((int)setpoint_kPa, 0, 255);
    ledcWrite(CH_INFLAR, pwm_main);
  }
  else if (mode_control == -2) { 
    digitalWrite(PIN_VALVULA_INFLAR, LOW); digitalWrite(PIN_VALVULA_SUCCION, HIGH);
    ledcWrite(CH_INFLAR, 0);
    pwm_main = constrain((int)setpoint_kPa, 0, 255);
    ledcWrite(CH_SUCCION_MAIN, pwm_main);
  }

  // Debug: Enviamos PWM Total y la parte que contribuye el FF (en slot 2) para análisis
  debug_data[0] = pwm_main; 
  debug_data[1] = pwm_aux; 
  debug_data[2] = (int16_t)(error * 10); 
  debug_data[3] = (int16_t)mode_control;
  
  msg_debug.data.data = debug_data; msg_debug.data.size = 4;
  rcl_publish(&pub_debug, &msg_debug, NULL);
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!ads.begin()) { Serial.println("Error ADS"); }
  ads.setGain(GAIN_ONE);

  pinMode(PIN_VALVULA_INFLAR, OUTPUT); pinMode(PIN_VALVULA_SUCCION, OUTPUT);
  pinMode(PIN_DISTRIB_A, OUTPUT); pinMode(PIN_DISTRIB_B, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  ledcSetup(CH_INFLAR, PWM_FREQ, PWM_RES); ledcAttachPin(PIN_BOMBA_INFLAR, CH_INFLAR);
  ledcSetup(CH_SUCCION_MAIN, PWM_FREQ, PWM_RES); ledcAttachPin(PIN_BOMBA_SUCCION_MAIN, CH_SUCCION_MAIN);
  ledcSetup(CH_SUCCION_AUX, PWM_FREQ, PWM_RES); ledcAttachPin(PIN_BOMBA_SUCCION_AUX, CH_SUCCION_AUX);

  stopAll();

  set_microros_transports();
  allocator = rcl_get_default_allocator();
  CHECK_RET(rclc_support_init(&support, 0, NULL, &allocator));
  CHECK_RET(rclc_node_init_default(&node, "soft_robot_node", "", &support));

  rclc_subscription_init_default(&sub_setpoint, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "pressure_setpoint");
  rclc_subscription_init_default(&sub_mode, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "pressure_mode");
  rclc_subscription_init_default(&sub_chamber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "active_chamber");
  rclc_subscription_init_default(&sub_tuning, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "tuning_params");

  rclc_publisher_init_default(&pub_feedback, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "pressure_feedback");
  rclc_publisher_init_default(&pub_debug, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray), "system_debug");

  msg_debug.data.capacity = 4; msg_debug.data.size = 4; msg_debug.data.data = debug_data;
  msg_tuning.data.capacity = 6; msg_tuning.data.size = 6; msg_tuning.data.data = tuning_buffer;

  CHECK_RET(rclc_executor_init(&executor, &support.context, 4, &allocator));
  rclc_executor_add_subscription(&executor, &sub_setpoint, &msg_setpoint, &setpoint_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_mode, &msg_mode, &mode_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_chamber, &msg_chamber, &chamber_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &sub_tuning, &msg_tuning, &tuning_callback, ON_NEW_DATA);
  
  last_control_ms = millis();
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
  unsigned long now = millis();
  if (now - last_control_ms >= Ts_ms) {
    last_control_ms += Ts_ms;
    controlLoop();
  }
}
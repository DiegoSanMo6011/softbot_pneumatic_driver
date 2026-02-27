/**
 * @file softbot_controller_v16_pi_puro.ino
 * @brief Controlador v16 - PI puro discreto con anti-windup.
 * @details
 * - Control PI puro en inflado y succión (sin rama agresiva por umbral).
 * - Anti-windup simétrico con integración condicional y saturación de integrador.
 * - Release automático en succión (histéresis) para evitar sobre-vacío al entrar a PI.
 * - Hardware: 4 Bombas activas.
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
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/int16_multi_array.h>

// ==========================================
// 1. HARDWARE
// ==========================================
const int PIN_VALVE_INFLATE = 25;
const int PIN_VALVE_SUCTION = 26;
const int PIN_VALVE_CHAMBER_C = 23; // Legacy BOOST pin reused for chamber C gating.
const int PIN_MUX_CHAMBER_A = 32;
const int PIN_MUX_CHAMBER_B = 33;

// Bombas
const int PIN_PUMP_INFLATE_MAIN = 19;
const int PIN_PUMP_INFLATE_AUX = 4;

const int PIN_PUMP_SUCTION_MAIN = 18;
const int PIN_PUMP_SUCTION_AUX = 27;

// Canales PWM
const int CH_INFLATE_MAIN = 0;
const int CH_SUCCION_MAIN = 1;
const int CH_SUCCION_AUX = 2;
const int CH_INFLATE_AUX = 3;

const int PWM_FREQ = 1000;
const int PWM_RES = 8;
const int PWM_MAX = 255;
const int PI_PWM_MAX_INFLATE = 255;
const int PI_PWM_MAX_SUCTION = 140;

// Modos de control
const int8_t MODE_STOP = 0;
const int8_t MODE_PID_INFLATE = 1;
const int8_t MODE_PID_SUCTION = -1;
const int8_t MODE_PWM_INFLATE = 2;
const int8_t MODE_PWM_SUCTION = -2;
const int8_t MODE_VENT = 4;
const int8_t MODE_HARDWARE_DIAGNOSTIC = 9;

// Bitmask /hardware_test (modo 9)
const uint16_t HW_PUMP_INFLATE_MAIN = (1 << 0);
const uint16_t HW_PUMP_INFLATE_AUX = (1 << 1);
const uint16_t HW_PUMP_SUCTION_MAIN = (1 << 2);
const uint16_t HW_PUMP_SUCTION_AUX = (1 << 3);
const uint16_t HW_VALVE_INFLATE = (1 << 4);
const uint16_t HW_VALVE_SUCTION = (1 << 5);
const uint16_t HW_VALVE_CHAMBER_C = (1 << 6);
const uint16_t HW_MUX_CHAMBER_A = (1 << 7);
const uint16_t HW_MUX_CHAMBER_B = (1 << 8);

const uint8_t DIAG_GROUP_MAIN = 1;
const uint8_t DIAG_GROUP_AUX = 2;

struct PumpOutputConfig {
  uint16_t mask;
  int channel;
  int pin;
  uint8_t telemetry_group;
};

struct DigitalOutputConfig {
  uint16_t mask;
  int pin;
};

const PumpOutputConfig DIAG_PUMP_OUTPUTS[] = {
    {HW_PUMP_INFLATE_MAIN, CH_INFLATE_MAIN, PIN_PUMP_INFLATE_MAIN, DIAG_GROUP_MAIN},
    {HW_PUMP_INFLATE_AUX, CH_INFLATE_AUX, PIN_PUMP_INFLATE_AUX, DIAG_GROUP_AUX},
    {HW_PUMP_SUCTION_MAIN, CH_SUCCION_MAIN, PIN_PUMP_SUCTION_MAIN, DIAG_GROUP_MAIN},
    {HW_PUMP_SUCTION_AUX, CH_SUCCION_AUX, PIN_PUMP_SUCTION_AUX, DIAG_GROUP_AUX},
};

const DigitalOutputConfig DIAG_DIGITAL_OUTPUTS[] = {
    {HW_VALVE_INFLATE, PIN_VALVE_INFLATE},
    {HW_VALVE_SUCTION, PIN_VALVE_SUCTION},
    {HW_VALVE_CHAMBER_C, PIN_VALVE_CHAMBER_C},
    {HW_MUX_CHAMBER_A, PIN_MUX_CHAMBER_A},
    {HW_MUX_CHAMBER_B, PIN_MUX_CHAMBER_B},
};

const int THRESHOLD_AUX_ENABLE = 40;

const int PIN_I2C_SDA = 21;
const int PIN_I2C_SCL = 22;
const int PIN_LED_STATUS = 2;

// ==========================================
// 2. PARÁMETROS DE CONTROL
// ==========================================
float Kp_neg = -75.00f;
float Ki_neg = -750.00f;
float Kp_pos = 24.0f;
float Ki_pos = 1500.0f;

const float TS_SECONDS = 0.020f;
const int TS_MS = 20;

float integral_pos_sum = 0.0f;
float integral_neg_sum = 0.0f;
float safety_limit_max = 55.0f;
float safety_limit_min = -60.0f;

float setpoint_pressure = 0.0f;
float current_pressure = 0.0f;
int8_t control_mode = 0;
int8_t active_chamber = 0;

bool emergency_stop_active = false;
unsigned long last_control_timestamp = 0;

Adafruit_ADS1115 ads;
const float V_OFFSET = 2.5f;
const float V_SENSITIVITY = 0.02f;

uint16_t hardware_test_mask = 0;

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
rcl_subscription_t sub_hwtest;

rcl_publisher_t pub_feedback;
rcl_publisher_t pub_debug;

std_msgs__msg__Float32 msg_setpoint;
std_msgs__msg__Int8 msg_mode;
std_msgs__msg__Int8 msg_chamber;
std_msgs__msg__Int16 msg_hwtest;
std_msgs__msg__Float32MultiArray msg_tuning;
std_msgs__msg__Float32 msg_feedback;
std_msgs__msg__Int16MultiArray msg_debug;

int16_t debug_data[4];
float tuning_buffer[6];

void fatal_error_loop(int code) {
  stopActuators();
  while (1) {
    for (int i = 0; i < code; i++) {
      digitalWrite(PIN_LED_STATUS, HIGH);
      delay(150);
      digitalWrite(PIN_LED_STATUS, LOW);
      delay(150);
    }
    delay(1500);
  }
}
#define RCCHECK(fn)                                                                                \
  {                                                                                                \
    rcl_ret_t temp_rc = fn;                                                                        \
    if ((temp_rc != RCL_RET_OK)) {                                                                 \
      fatal_error_loop(1);                                                                         \
    }                                                                                              \
  }

// Count transient publish failures without halting control execution.
uint32_t publish_soft_failures = 0;
#define RCSOFTCHECK(fn)                                                                             \
  {                                                                                                 \
    rcl_ret_t temp_rc = fn;                                                                         \
    if ((temp_rc != RCL_RET_OK)) {                                                                  \
      publish_soft_failures++;                                                                      \
    }                                                                                               \
  }
void applyInflateControl(float error, int *pwm_main, int *pwm_aux);
void applySuctionControl(float error, int *pwm_main, int *pwm_aux);
void applyHardwareDiagnosticOutputs(uint16_t mask, int pwm_diag, int16_t *active_main,
                                    int16_t *active_aux);
void applyChamberSelectionMask(int8_t chamber_mask, bool vent_mode);

// ==========================================
// FUNCIONES
// ==========================================
float readPressureChannel(uint8_t channel) {
  int16_t adc = ads.readADC_SingleEnded(channel);
  float volts = ads.computeVolts(adc);
  return (volts - V_OFFSET) / V_SENSITIVITY;
}

void stopActuators() {
  const size_t pump_count = sizeof(DIAG_PUMP_OUTPUTS) / sizeof(DIAG_PUMP_OUTPUTS[0]);
  for (size_t idx = 0; idx < pump_count; idx++) {
    ledcWrite(DIAG_PUMP_OUTPUTS[idx].channel, 0);
  }

  const size_t digital_count = sizeof(DIAG_DIGITAL_OUTPUTS) / sizeof(DIAG_DIGITAL_OUTPUTS[0]);
  for (size_t idx = 0; idx < digital_count; idx++) {
    digitalWrite(DIAG_DIGITAL_OUTPUTS[idx].pin, LOW);
  }
}

void applyHardwareDiagnosticOutputs(uint16_t mask, int pwm_diag, int16_t *active_main,
                                    int16_t *active_aux) {
  int16_t main_pwm = 0;
  int16_t aux_pwm = 0;

  const size_t pump_count = sizeof(DIAG_PUMP_OUTPUTS) / sizeof(DIAG_PUMP_OUTPUTS[0]);
  for (size_t idx = 0; idx < pump_count; idx++) {
    const PumpOutputConfig output = DIAG_PUMP_OUTPUTS[idx];
    const bool enabled = (mask & output.mask) != 0;
    const int pwm_value = enabled ? pwm_diag : 0;
    ledcWrite(output.channel, pwm_value);

    if (enabled && output.telemetry_group == DIAG_GROUP_MAIN) {
      main_pwm = (int16_t)pwm_diag;
    } else if (enabled && output.telemetry_group == DIAG_GROUP_AUX) {
      aux_pwm = (int16_t)pwm_diag;
    }
  }

  const size_t digital_count = sizeof(DIAG_DIGITAL_OUTPUTS) / sizeof(DIAG_DIGITAL_OUTPUTS[0]);
  for (size_t idx = 0; idx < digital_count; idx++) {
    const DigitalOutputConfig output = DIAG_DIGITAL_OUTPUTS[idx];
    digitalWrite(output.pin, (mask & output.mask) ? HIGH : LOW);
  }

  *active_main = main_pwm;
  *active_aux = aux_pwm;
}

void applyChamberSelectionMask(int8_t chamber_mask, bool vent_mode) {
  int8_t normalized = chamber_mask & 0x07;
  if (vent_mode && normalized == 0) {
    normalized = 0x07; // Vent all chambers when command asks for blocked chamber in VENT mode.
  }

  const bool chamber_a_enabled = (normalized & 0x01) != 0;
  const bool chamber_b_enabled = (normalized & 0x02) != 0;
  const bool chamber_c_enabled = (normalized & 0x04) != 0;

  digitalWrite(PIN_MUX_CHAMBER_A, chamber_a_enabled ? HIGH : LOW);
  digitalWrite(PIN_MUX_CHAMBER_B, chamber_b_enabled ? HIGH : LOW);
  digitalWrite(PIN_VALVE_CHAMBER_C, chamber_c_enabled ? HIGH : LOW);
}

void triggerEmergencyStop() {
  ledcWrite(CH_INFLATE_MAIN, 0);
  ledcWrite(CH_INFLATE_AUX, 0);
  ledcWrite(CH_SUCCION_MAIN, 0);
  ledcWrite(CH_SUCCION_AUX, 0);
  digitalWrite(PIN_VALVE_INFLATE, HIGH);
  digitalWrite(PIN_VALVE_SUCTION, HIGH);
  digitalWrite(PIN_VALVE_CHAMBER_C, LOW);
  digitalWrite(PIN_MUX_CHAMBER_A, HIGH);
  digitalWrite(PIN_MUX_CHAMBER_B, HIGH);
  delay(2000);
  stopActuators();
  emergency_stop_active = true;
  control_mode = 0;
}

int computePIOutput(float error, float *integral_sum, float kp, float ki, int pwm_limit) {
  if (pwm_limit < 0) {
    pwm_limit = 0;
  } else if (pwm_limit > PWM_MAX) {
    pwm_limit = PWM_MAX;
  }

  float integral_candidate = *integral_sum + (error * TS_SECONDS);
  float control_candidate = (kp * error) + (ki * integral_candidate);

  float control_limited = control_candidate;
  if (control_limited < 0.0f) {
    control_limited = 0.0f;
  }
  if (control_limited > (float)pwm_limit) {
    control_limited = (float)pwm_limit;
  }

  const bool sat_low = control_limited <= 0.0f;
  const bool sat_high = control_limited >= (float)pwm_limit;
  const bool pushes_further =
      (sat_low && control_candidate < 0.0f) || (sat_high && control_candidate > (float)pwm_limit);

  if (!pushes_further) {
    *integral_sum = integral_candidate;
  }

  if (fabsf(ki) > 1e-6f) {
    const float integral_limit = (float)pwm_limit / fabsf(ki);
    if (*integral_sum > integral_limit) {
      *integral_sum = integral_limit;
    } else if (*integral_sum < -integral_limit) {
      *integral_sum = -integral_limit;
    }
  } else {
    *integral_sum = 0.0f;
  }

  float control = (kp * error) + (ki * (*integral_sum));
  if (control < 0.0f) {
    control = 0.0f;
  }
  if (control > (float)pwm_limit) {
    control = (float)pwm_limit;
  }
  return (int)roundf(control);
}

void applyInflateControl(float error, int *pwm_main, int *pwm_aux) {
  *pwm_main = computePIOutput(error, &integral_pos_sum, Kp_pos, Ki_pos, PI_PWM_MAX_INFLATE);
  if (*pwm_main > THRESHOLD_AUX_ENABLE) {
    *pwm_aux = *pwm_main;
  } else {
    *pwm_aux = 0;
  }
}

void applySuctionControl(float error, int *pwm_main, int *pwm_aux) {
  *pwm_main = computePIOutput(error, &integral_neg_sum, Kp_neg, Ki_neg, PI_PWM_MAX_SUCTION);
  if (*pwm_main > THRESHOLD_AUX_ENABLE) {
    *pwm_aux = *pwm_main;
  } else {
    *pwm_aux = 0;
  }
}

// ==========================================
// CALLBACKS
// ==========================================
void cb_setpoint(const void *msgin) {
  if (!emergency_stop_active)
    setpoint_pressure = ((const std_msgs__msg__Float32 *)msgin)->data;
}
void cb_chamber(const void *msgin) {
  if (!emergency_stop_active) {
    int8_t requested = ((const std_msgs__msg__Int8 *)msgin)->data;
    if (requested < 0) {
      requested = 0;
    }
    active_chamber = requested & 0x07;
  }
}

void cb_mode(const void *msgin) {
  int8_t m = ((const std_msgs__msg__Int8 *)msgin)->data;
  if (emergency_stop_active && m == MODE_STOP)
    emergency_stop_active = false;
  if (emergency_stop_active)
    return;

  bool mode_changed = (m != control_mode);

  if (mode_changed) {
    integral_pos_sum = 0;
    integral_neg_sum = 0;
  }

  if (mode_changed && control_mode == MODE_HARDWARE_DIAGNOSTIC) {
    hardware_test_mask = 0;
    stopActuators();
  }

  control_mode = m;
}

void cb_tuning(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;
  if (msg->data.size >= 6) {
    Kp_pos = msg->data.data[0];
    Ki_pos = msg->data.data[1];
    Kp_neg = msg->data.data[2];
    Ki_neg = msg->data.data[3];
    safety_limit_max = msg->data.data[4];
    safety_limit_min = msg->data.data[5];
    integral_pos_sum = 0;
    integral_neg_sum = 0;
  }
}

void cb_hwtest(const void *msgin) {
  if (emergency_stop_active) {
    hardware_test_mask = 0;
    return;
  }
  hardware_test_mask = (uint16_t)((const std_msgs__msg__Int16 *)msgin)->data;
}

// ==========================================
// LOOP DE CONTROL
// ==========================================
void controlLoop() {
  if (control_mode == MODE_PID_SUCTION || control_mode == MODE_PWM_SUCTION) {
    current_pressure = readPressureChannel(1);
    if (current_pressure < safety_limit_min || current_pressure > safety_limit_max) {
      triggerEmergencyStop();
    }
  } else {
    current_pressure = readPressureChannel(0);
    if (current_pressure > safety_limit_max || current_pressure < safety_limit_min) {
      triggerEmergencyStop();
    }
  }

  msg_feedback.data = current_pressure;
  RCSOFTCHECK(rcl_publish(&pub_feedback, &msg_feedback, NULL));

  if (emergency_stop_active) {
    debug_data[0] = -1;
    debug_data[1] = -1;
    debug_data[2] = 0;
    debug_data[3] = 0;
    msg_debug.data.data = debug_data;
    msg_debug.data.size = 4;
    RCSOFTCHECK(rcl_publish(&pub_debug, &msg_debug, NULL));
    return;
  }

  if (control_mode == MODE_HARDWARE_DIAGNOSTIC) {
    // In diagnostic mode, read BOTH sensors so the GUI can display them
    current_pressure = readPressureChannel(0);
    float vacuum_pressure = readPressureChannel(1);

    msg_feedback.data = current_pressure;
    RCSOFTCHECK(rcl_publish(&pub_feedback, &msg_feedback, NULL));

    int pwm_diag = constrain((int)setpoint_pressure, 0, 255);
    int16_t active_main = 0;
    int16_t active_aux = 0;
    applyHardwareDiagnosticOutputs(hardware_test_mask, pwm_diag, &active_main, &active_aux);
    debug_data[0] = active_main;
    debug_data[1] = active_aux;
    debug_data[2] = (int16_t)(vacuum_pressure * 10.0f); // Ch1 vacuum kPa × 10
    debug_data[3] = (int16_t)control_mode;
    msg_debug.data.data = debug_data;
    msg_debug.data.size = 4;
    RCSOFTCHECK(rcl_publish(&pub_debug, &msg_debug, NULL));
    return;
  }

  applyChamberSelectionMask(active_chamber, control_mode == MODE_VENT);

  float error = setpoint_pressure - current_pressure;
  int pwm_main = 0;
  int pwm_aux = 0;

  if (control_mode == MODE_VENT) { // VENTEAR (liberar presion a atmosfera)
    // Bombas apagadas
    ledcWrite(CH_INFLATE_MAIN, 0);
    ledcWrite(CH_INFLATE_AUX, 0);
    ledcWrite(CH_SUCCION_MAIN, 0);
    ledcWrite(CH_SUCCION_AUX, 0);

    // Válvulas desenergizadas (A->R abierto / P cerrado)
    digitalWrite(PIN_VALVE_INFLATE, LOW);
    digitalWrite(PIN_VALVE_SUCTION, LOW);
  } else if (control_mode == MODE_STOP || (active_chamber == 0 && control_mode != MODE_VENT)) {
    stopActuators();
    integral_pos_sum = 0;
    integral_neg_sum = 0;
  } else if (control_mode == MODE_PID_INFLATE) { // INFLAR PI PURO
    digitalWrite(PIN_VALVE_INFLATE, HIGH);
    digitalWrite(PIN_VALVE_SUCTION, LOW);
    ledcWrite(CH_SUCCION_MAIN, 0);
    ledcWrite(CH_SUCCION_AUX, 0);
    applyInflateControl(error, &pwm_main, &pwm_aux);
    ledcWrite(CH_INFLATE_MAIN, pwm_main);
    ledcWrite(CH_INFLATE_AUX, pwm_aux);
  } else if (control_mode == MODE_PID_SUCTION) { // SUCCIONAR PI PURO
    ledcWrite(CH_INFLATE_MAIN, 0);
    ledcWrite(CH_INFLATE_AUX, 0);

    digitalWrite(PIN_VALVE_INFLATE, LOW);
    digitalWrite(PIN_VALVE_SUCTION, HIGH);
    applySuctionControl(error, &pwm_main, &pwm_aux);
    ledcWrite(CH_SUCCION_MAIN, pwm_main);
    ledcWrite(CH_SUCCION_AUX, pwm_aux);
  }
  // MODOS MANUALES
  else if (control_mode == MODE_PWM_INFLATE || control_mode == MODE_PWM_SUCTION) {
    pwm_main = constrain((int)setpoint_pressure, 0, 255);

    // Espejo Directo en Manual
    if (pwm_main > THRESHOLD_AUX_ENABLE)
      pwm_aux = pwm_main;
    else
      pwm_aux = 0;

    if (control_mode == MODE_PWM_INFLATE) { // Inflar
      digitalWrite(PIN_VALVE_INFLATE, HIGH);
      digitalWrite(PIN_VALVE_SUCTION, LOW);
      ledcWrite(CH_SUCCION_MAIN, 0);
      ledcWrite(CH_SUCCION_AUX, 0);
      ledcWrite(CH_INFLATE_MAIN, pwm_main);
      ledcWrite(CH_INFLATE_AUX, pwm_aux);
    } else { // Succión
      digitalWrite(PIN_VALVE_INFLATE, LOW);
      digitalWrite(PIN_VALVE_SUCTION, HIGH);
      ledcWrite(CH_INFLATE_MAIN, 0);
      ledcWrite(CH_INFLATE_AUX, 0);
      ledcWrite(CH_SUCCION_MAIN, pwm_main);
      ledcWrite(CH_SUCCION_AUX, pwm_aux);
    }
  }

  debug_data[0] = pwm_main;
  debug_data[1] = pwm_aux;
  debug_data[2] = (int16_t)(error * 10);
  debug_data[3] = (int16_t)control_mode;
  msg_debug.data.data = debug_data;
  msg_debug.data.size = 4;
  RCSOFTCHECK(rcl_publish(&pub_debug, &msg_debug, NULL));
  RCSOFTCHECK(rcl_publish(&pub_debug, &msg_debug, NULL));
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  if (!ads.begin()) { /* Error */
  }
  ads.setGain(GAIN_ONE);
  ads.setDataRate(RATE_ADS1115_860SPS); // Max speed to prevent I2C blocks from starving ROS

  const size_t digital_count = sizeof(DIAG_DIGITAL_OUTPUTS) / sizeof(DIAG_DIGITAL_OUTPUTS[0]);
  for (size_t idx = 0; idx < digital_count; idx++) {
    pinMode(DIAG_DIGITAL_OUTPUTS[idx].pin, OUTPUT);
  }
  pinMode(PIN_LED_STATUS, OUTPUT);

  const size_t pump_count = sizeof(DIAG_PUMP_OUTPUTS) / sizeof(DIAG_PUMP_OUTPUTS[0]);
  for (size_t idx = 0; idx < pump_count; idx++) {
    ledcSetup(DIAG_PUMP_OUTPUTS[idx].channel, PWM_FREQ, PWM_RES);
    ledcAttachPin(DIAG_PUMP_OUTPUTS[idx].pin, DIAG_PUMP_OUTPUTS[idx].channel);
  }

  stopActuators();

  set_microros_transports();
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "soft_robot_node", "", &support));

  RCCHECK(rclc_subscription_init_default(&sub_setpoint, &node,
                                         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                                         "pressure_setpoint"));
  RCCHECK(rclc_subscription_init_default(
      &sub_mode, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "pressure_mode"));
  RCCHECK(rclc_subscription_init_default(
      &sub_chamber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8), "active_chamber"));
  RCCHECK(rclc_subscription_init_default(
      &sub_tuning, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "tuning_params"));
  RCCHECK(rclc_subscription_init_default(
      &sub_hwtest, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "hardware_test"));

  RCCHECK(rclc_publisher_init_default(&pub_feedback, &node,
                                      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                                      "pressure_feedback"));
  RCCHECK(rclc_publisher_init_default(&pub_debug, &node,
                                      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
                                      "system_debug"));

  msg_debug.data.capacity = 4;
  msg_debug.data.size = 4;
  msg_debug.data.data = debug_data;
  msg_tuning.data.capacity = 6;
  msg_tuning.data.size = 6;
  msg_tuning.data.data = tuning_buffer;

  // 6. Initialize message buffers to safe values
  msg_mode.data = 0;
  msg_setpoint.data = 0.0f;
  msg_chamber.data = 0;
  msg_hwtest.data = 0;

  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_setpoint, &msg_setpoint, &cb_setpoint,
                                         ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_mode, &msg_mode, &cb_mode, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_chamber, &msg_chamber, &cb_chamber,
                                         ON_NEW_DATA));
  RCCHECK(
      rclc_executor_add_subscription(&executor, &sub_tuning, &msg_tuning, &cb_tuning, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_hwtest, &msg_hwtest, &cb_hwtest,
                                         ON_NEW_DATA));

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

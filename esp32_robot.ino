/*
  ESP32 Robot Motor Controller — WiFi UDP Receiver
  =================================================
  - ESP32 acts as WiFi Access Point (or STA — see config)
  - Listens for UDP packets: "L<omega_left>,R<omega_right>\n"
  - Converts rad/s → PWM duty and drives two motors via L298N / L293D
  - Motor control loop runs independently at MOTOR_HZ
  - WiFi/UDP receive loop runs at UDP_HZ (separate task on core 0)

  ArUco IDs to print:
    Robot  marker ID : 1  (4x4_50 dictionary)
    Target marker ID : 0  (4x4_50 dictionary)
  Print from: https://chev.me/arucogen/
  Dictionary: 4X4_50, marker size 10cm recommended
*/

#include <WiFi.h>
#include <WiFiUdp.h>

// ─────────────────────────────────────────────
//  NETWORK CONFIG
// ─────────────────────────────────────────────
const char* SSID     = "RobotAP";       // ESP32 Access Point SSID
const char* PASSWORD = "robot1234";     // AP password (min 8 chars)
const int   UDP_PORT = 4210;            // must match PC script

// ─────────────────────────────────────────────
//  MOTOR PIN CONFIG  (adjust to your wiring)
//  Using L298N dual H-bridge
// ─────────────────────────────────────────────

// LEFT MOTOR
#define LEFT_IN1  27
#define LEFT_IN2  26
#define LEFT_ENA  14   // PWM pin

// RIGHT MOTOR
#define RIGHT_IN1 25
#define RIGHT_IN2 33
#define RIGHT_ENB 32   // PWM pin

// PWM channels (ESP32 LEDC)
#define PWM_CHANNEL_L  0
#define PWM_CHANNEL_R  1
#define PWM_FREQ       1000   // Hz
#define PWM_RESOLUTION 8      // bits → 0-255

// ─────────────────────────────────────────────
//  ROBOT PARAMETERS  (must match PC script)
// ─────────────────────────────────────────────
const float WHEEL_RADIUS = 0.033f;   // metres
const float WHEEL_BASE   = 0.16f;    // metres
const float MAX_OMEGA    = 15.7f;    // rad/s  (≈150 RPM)

// ─────────────────────────────────────────────
//  LOOP RATES
// ─────────────────────────────────────────────
const int UDP_HZ   = 50;    // how often ESP32 checks for new UDP packets
const int MOTOR_HZ = 100;   // how often motor PWM is updated

// ─────────────────────────────────────────────
//  GLOBALS (shared between tasks)
// ─────────────────────────────────────────────
volatile float g_omega_left  = 0.0f;
volatile float g_omega_right = 0.0f;
SemaphoreHandle_t g_mutex;

WiFiUDP udp;

// ─────────────────────────────────────────────
//  MOTOR DRIVER HELPERS
// ─────────────────────────────────────────────

void motorSetup() {
  ledcSetup(PWM_CHANNEL_L, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_R, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(LEFT_ENA,  PWM_CHANNEL_L);
  ledcAttachPin(RIGHT_ENB, PWM_CHANNEL_R);

  pinMode(LEFT_IN1,  OUTPUT);
  pinMode(LEFT_IN2,  OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
}

/*
 * Drive one motor.
 * omega > 0 → forward, omega < 0 → reverse
 * Converts rad/s to 0-255 PWM duty.
 */
void driveMotor(int in1, int in2, int pwmChannel, float omega) {
  int duty = (int)(fabs(omega) / MAX_OMEGA * 255.0f);
  duty = constrain(duty, 0, 255);

  if (omega > 0.01f) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (omega < -0.01f) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    duty = 0;
  }
  ledcWrite(pwmChannel, duty);
}

// ─────────────────────────────────────────────
//  TASK: UDP RECEIVE  (runs on core 0)
// ─────────────────────────────────────────────

void udpTask(void* param) {
  const TickType_t period = pdMS_TO_TICKS(1000 / UDP_HZ);
  char buf[64];

  for (;;) {
    int packetSize = udp.parsePacket();
    if (packetSize > 0) {
      int len = udp.read(buf, sizeof(buf) - 1);
      if (len > 0) {
        buf[len] = '\0';
        float l = 0.0f, r = 0.0f;
        // Expected format: "L<float>,R<float>\n"
        if (sscanf(buf, "L%f,R%f", &l, &r) == 2) {
          if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            g_omega_left  = l;
            g_omega_right = r;
            xSemaphoreGive(g_mutex);
          }
        }
      }
    }
    vTaskDelay(period);
  }
}

// ─────────────────────────────────────────────
//  TASK: MOTOR CONTROL  (runs on core 1)
// ─────────────────────────────────────────────

void motorTask(void* param) {
  const TickType_t period = pdMS_TO_TICKS(1000 / MOTOR_HZ);

  for (;;) {
    float ol = 0.0f, or_ = 0.0f;
    if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      ol  = g_omega_left;
      or_ = g_omega_right;
      xSemaphoreGive(g_mutex);
    }

    driveMotor(LEFT_IN1,  LEFT_IN2,  PWM_CHANNEL_L, ol);
    driveMotor(RIGHT_IN1, RIGHT_IN2, PWM_CHANNEL_R, or_);

    vTaskDelay(period);
  }
}

// ─────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  motorSetup();

  // Start WiFi as Access Point
  WiFi.softAP(SSID, PASSWORD);
  Serial.print("[ESP32] AP IP: ");
  Serial.println(WiFi.softAPIP());

  udp.begin(UDP_PORT);
  Serial.printf("[ESP32] UDP listening on port %d\n", UDP_PORT);

  g_mutex = xSemaphoreCreateMutex();

  // UDP task on core 0
  xTaskCreatePinnedToCore(udpTask,   "udpTask",   4096, NULL, 1, NULL, 0);
  // Motor task on core 1
  xTaskCreatePinnedToCore(motorTask, "motorTask", 2048, NULL, 2, NULL, 1);
}

// ─────────────────────────────────────────────
//  LOOP — kept minimal, status print only
// ─────────────────────────────────────────────

void loop() {
  static unsigned long last = 0;
  if (millis() - last > 1000) {
    last = millis();
    if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      Serial.printf("[ESP32] wL=%.2f  wR=%.2f\n", g_omega_left, g_omega_right);
      xSemaphoreGive(g_mutex);
    }
  }
}

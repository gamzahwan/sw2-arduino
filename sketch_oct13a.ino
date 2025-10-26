#include <Servo.h>

#define PIN_LED   9
#define PIN_TRIG  12
#define PIN_ECHO  13
#define PIN_SERVO 10

#define SND_VEL 346.0
#define INTERVAL 25
#define PULSE_DURATION 10
#define _DIST_MIN 180.0
#define _DIST_MAX 360.0

#define TIMEOUT ((unsigned long)((INTERVAL / 2.0) * 1000.0))
#define SCALE (0.001 * 0.5 * SND_VEL)

#define _EMA_ALPHA 0.3

#define _TARGET_LOW  250.0
#define _TARGET_HIGH 290.0

#define _DUTY_MIN 1000
#define _DUTY_NEU 1500
#define _DUTY_MAX 2000

float dist_ema = _DIST_MAX, dist_prev = _DIST_MAX;
unsigned long last_sampling_time = 0;

Servo myservo;

static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

static inline int mapDistToAngle(float d_mm) {
  if (d_mm <= _DIST_MIN) return 0;
  if (d_mm >= _DIST_MAX) return 180;
  float r = (d_mm - _DIST_MIN) / (_DIST_MAX - _DIST_MIN);
  return (int)(r * 180.0f + 0.5f);
}

static inline int angleToUS(int angle) {
  return _DUTY_MIN + (int)((_DUTY_MAX - _DUTY_MIN) * (angle / 180.0f) + 0.5f);
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NEU);

  Serial.begin(57600);
  last_sampling_time = millis();
}

void loop() {
  if (millis() < last_sampling_time + INTERVAL) return;
  last_sampling_time += INTERVAL;

  float dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);
  float dist_filtered;

  if ((dist_raw == 0.0) || (dist_raw > _DIST_MAX) || (dist_raw < _DIST_MIN)) {
    dist_filtered = dist_prev;
  } else {
    dist_filtered = dist_raw;
    dist_prev = dist_raw;
  }

  dist_ema = _EMA_ALPHA * dist_filtered + (1.0f - _EMA_ALPHA) * dist_ema;

  int angle = mapDistToAngle(dist_ema);
  myservo.write(angle);
  myservo.writeMicroseconds(angleToUS(angle));

  if (dist_ema >= _TARGET_LOW && dist_ema <= _TARGET_HIGH) digitalWrite(PIN_LED, HIGH);
  else digitalWrite(PIN_LED, LOW);

  Serial.print("Min:");    Serial.print(_DIST_MIN);
  Serial.print(",Low:");   Serial.print(_TARGET_LOW);
  Serial.print(",dist:");  Serial.print(dist_raw);
  Serial.print(",Servo:"); Serial.print(myservo.read());
  Serial.print(",High:");  Serial.print(_TARGET_HIGH);
  Serial.print(",Max:");   Serial.print(_DIST_MAX);
  Serial.println("");
}

float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE;
}

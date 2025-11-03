#include <Servo.h>

#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0

#define _DUTY_MIN 500
#define _DUTY_NEU 1500
#define _DUTY_MAX 2400

#define INTERVAL_MS 30
#define EMA_ALPHA 0.30f
#define RANGE_MIN_MM 100.0f
#define RANGE_MAX_MM 250.0f

Servo servo;
unsigned long t_prev = 0;
float dist_ema = NAN;
int last_duty = _DUTY_NEU;
bool have_valid = false;

static inline float clamp(float x, float lo, float hi){ return x<lo?lo:(x>hi?hi:x); }

static inline float ir_adc_to_mm(int a_value){
  return (6762.0f/(float)(a_value-9)-4.0f)*10.0f-60.0f;
}

static inline int angle_to_duty_us(float deg){
  deg = clamp(deg,0.0f,180.0f);
  float t = deg/180.0f;
  return (int)((_DUTY_MIN + (_DUTY_MAX - _DUTY_MIN)*t) + 0.5f);
}

static inline float mm_to_angle(float mm){
  float t = (mm - RANGE_MIN_MM) / (RANGE_MAX_MM - RANGE_MIN_MM);
  t = clamp(t,0.0f,1.0f);
  return 180.0f * t;
}

void setup(){
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED,HIGH);
  servo.attach(PIN_SERVO);
  servo.writeMicroseconds(_DUTY_NEU);
  Serial.begin(1000000);
  t_prev = millis();
}

void loop(){
  unsigned long now = millis();
  if(now - t_prev < INTERVAL_MS) return;
  t_prev = now;

  int a_value = analogRead(PIN_IR);
  float dist_raw = ir_adc_to_mm(a_value);
  bool in_range = (dist_raw>=RANGE_MIN_MM && dist_raw<=RANGE_MAX_MM);

  if(in_range){
    if(isnan(dist_ema)) dist_ema = dist_raw;
    else dist_ema = EMA_ALPHA*dist_raw + (1.0f-EMA_ALPHA)*dist_ema;

    float angle = mm_to_angle(dist_ema);
    int duty = angle_to_duty_us(angle);
    servo.writeMicroseconds(duty);
    last_duty = duty;
    have_valid = true;
    digitalWrite(PIN_LED,LOW);
  } else {
    digitalWrite(PIN_LED,HIGH);
  }

  Serial.print("_DUTY_MIN:"); Serial.print(_DUTY_MIN);
  Serial.print(",_DIST_MIN:"); Serial.print(RANGE_MIN_MM);
  Serial.print(",IR:"); Serial.print(a_value);
  Serial.print(",dist_raw:"); Serial.print(dist_raw,2);
  Serial.print(",ema:"); Serial.print(dist_ema,2);
  Serial.print(",servo:"); Serial.print(have_valid ? last_duty : _DUTY_NEU);
  Serial.print(",_DIST_MAX:"); Serial.print(RANGE_MAX_MM);
  Serial.print(",_DUTY_MAX:"); Serial.print(_DUTY_MAX);
  Serial.println("");
}

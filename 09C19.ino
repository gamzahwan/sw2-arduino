#define PIN_LED  9
#define PIN_TRIG 12
#define PIN_ECHO 13

#define SND_VEL 346.0
#define INTERVAL 25
#define PULSE_DURATION 10
#define _DIST_MIN 100.0
#define _DIST_MAX 300.0

#define TIMEOUT_US ((unsigned long)((INTERVAL / 2.0) * 1000.0))
#define SCALE (0.001 * 0.5 * SND_VEL)

#define MEDIAN_N 30
#if MEDIAN_N > 30
  #error "MEDIAN_N must be <= 30"
#endif

float dist_buffer[30];
int buf_count = 0;
int buf_index = 0;

const float EMA_ALPHA = 0.2f;
float dist_ema = 0.0f;
bool ema_init = false;

unsigned long last_sampling_time = 0;

float USS_measure(int trigPin, int echoPin);
float median_filter_push(float new_sample);

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
  Serial.begin(57600);
}

void loop() {
  if ((millis() - last_sampling_time) < INTERVAL) return;

  float dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  if (!ema_init) {
    dist_ema = dist_raw;
    ema_init = true;
  } else {
    dist_ema = EMA_ALPHA * dist_raw + (1.0f - EMA_ALPHA) * dist_ema;
  }

  float dist_median = median_filter_push(dist_raw);

  int brightness = 255;
  if ((dist_median >= _DIST_MIN) && (dist_median <= _DIST_MAX)) {
    if (dist_median <= 200.0) {
      brightness = map((long)dist_median, 100, 200, 255, 0);
    } else {
      brightness = map((long)dist_median, 200, 300, 0, 255);
    }
  } else {
    brightness = 255;
  }
  analogWrite(PIN_LED, brightness);

  Serial.print("Min:"); Serial.print(_DIST_MIN, 2);
  Serial.print(",raw:"); Serial.print(dist_raw, 2);
  Serial.print(",ema:"); Serial.print(dist_ema, 2);
  Serial.print(",median:"); Serial.print(dist_median, 2);
  Serial.print(",Max:"); Serial.print(_DIST_MAX, 2);
  Serial.println();

  last_sampling_time = millis();
}

float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  unsigned long dur = pulseIn(ECHO, HIGH, TIMEOUT_US);
  float dist_mm = dur * SCALE;
  return dist_mm;
}

float median_filter_push(float new_sample) {
  dist_buffer[buf_index] = new_sample;
  buf_index = (buf_index + 1) % MEDIAN_N;
  if (buf_count < MEDIAN_N) buf_count++;

  float tmp[30];
  for (int i = 0; i < buf_count; ++i) tmp[i] = dist_buffer[i];

  for (int i = 1; i < buf_count; ++i) {
    float key = tmp[i];
    int j = i - 1;
    while (j >= 0 && tmp[j] > key) {
      tmp[j + 1] = tmp[j];
      j--;
    }
    tmp[j + 1] = key;
  }

  if (buf_count == 0) return 0.0;
  if (buf_count % 2 == 1)
    return tmp[buf_count / 2];
  else
    return (tmp[buf_count / 2 - 1] + tmp[buf_count / 2]) / 2.0;
}

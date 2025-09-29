#define PIN_LED  9
#define PIN_TRIG 12
#define PIN_ECHO 13

#define SND_VEL 346.0
#define INTERVAL 25
#define PULSE_DURATION 10
#define _DIST_MIN 100.0
#define _DIST_MAX 300.0

#define TIMEOUT ((INTERVAL / 2) * 1000.0)
#define SCALE (0.001 * 0.5 * SND_VEL)

unsigned long last_sampling_time = 0;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
  Serial.begin(57600);
}

void loop() {
  float distance;

  if (millis() - last_sampling_time < INTERVAL) {
    return;
  }

  distance = USS_measure(PIN_TRIG, PIN_ECHO);

  int brightness = 255;

  if ((distance >= _DIST_MIN) && (distance <= _DIST_MAX)) {
    if (distance <= 200.0) {
      brightness = map((long)distance, 100, 200, 255, 0);
    } else {
      brightness = map((long)distance, 200, 300, 0, 255);
    }
  } else {
    brightness = 255;
  }

  analogWrite(PIN_LED, brightness);

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" mm, Brightness: ");
  Serial.println(brightness);

  last_sampling_time = millis();
}

float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);

  unsigned long dur = pulseIn(ECHO, HIGH, (unsigned long)TIMEOUT);
  return dur * SCALE;
}

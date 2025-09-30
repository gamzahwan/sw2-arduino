#define PIN_LED 7

const int MIN_PERIOD_US = 100;
const int MAX_PERIOD_US = 10000;

int pwm_period_us = 1000;
int pwm_duty = 0;

const int HALF_STEPS = 101;
const int TOTAL_STEPS = HALF_STEPS + (HALF_STEPS - 1);
const unsigned long TRIANGLE_US = 1000000UL;
const unsigned long STEP_TIME_US = TRIANGLE_US / TOTAL_STEPS;

void set_period(int period_us) {
  if (period_us < MIN_PERIOD_US) period_us = MIN_PERIOD_US;
  if (period_us > MAX_PERIOD_US) period_us = MAX_PERIOD_US;
  pwm_period_us = period_us;
}

void set_duty(int duty) {
  if (duty < 0) duty = 0;
  if (duty > 100) duty = 100;
  pwm_duty = duty;
}

void softwarePWM_once(unsigned long high_time_us, unsigned long low_time_us) {
  if (high_time_us > 0) {
    digitalWrite(PIN_LED, HIGH);
    delayMicroseconds((unsigned int)high_time_us);
  }
  if (low_time_us > 0) {
    digitalWrite(PIN_LED, LOW);
    delayMicroseconds((unsigned int)low_time_us);
  }
}

void applyDutyForStep(unsigned long step_time_us) {
  unsigned long period = (unsigned long)pwm_period_us;
  unsigned long high_time = (period * (unsigned long)pwm_duty) / 100UL;
  unsigned long low_time = period - high_time;
  unsigned long full_cycles = step_time_us / period;

  if (full_cycles > 0) {
    for (unsigned long i = 0; i < full_cycles; i++) {
      softwarePWM_once(high_time, low_time);
    }
    unsigned long used = full_cycles * period;
    unsigned long rem = step_time_us - used;
    if (rem > 0) {
      unsigned long h_rem = (high_time < rem) ? high_time : rem;
      if (h_rem > 0) {
        digitalWrite(PIN_LED, HIGH);
        delayMicroseconds((unsigned int)h_rem);
      }
      unsigned long rem2 = rem - h_rem;
      if (rem2 > 0) {
        digitalWrite(PIN_LED, LOW);
        delayMicroseconds((unsigned int)rem2);
      }
    }
  } else {
    unsigned long high_for_step = (step_time_us * (unsigned long)pwm_duty) / 100UL;
    if (high_for_step > 0) {
      digitalWrite(PIN_LED, HIGH);
      delayMicroseconds((unsigned int)high_for_step);
    }
    unsigned long low_for_step = step_time_us - high_for_step;
    if (low_for_step > 0) {
      digitalWrite(PIN_LED, LOW);
      delayMicroseconds((unsigned int)low_for_step);
    }
  }
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
}

void loop() {
  set_period(10000);   // ✅ 고정된 값 (100us)

  for (int d = 0; d <= 100; d++) {
    set_duty(d);
    applyDutyForStep(STEP_TIME_US);
  }
  for (int d = 99; d >= 0; d--) {
    set_duty(d);
    applyDutyForStep(STEP_TIME_US);
  }

  delay(300);
}

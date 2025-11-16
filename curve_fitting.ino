#include <stdlib.h>
#include <math.h>

#define PIN_IR A0
#define SAMPLE_N 10
#define MAX_POINTS 6 
#define MAX_DEGREE 3

float coeff[MAX_DEGREE + 1];

int compare(const void *a, const void *b) {
  return (*(unsigned int *)a - *(unsigned int *)b);
}

unsigned int ir_sensor_filtered(unsigned int n, float position, int verbose) {
  unsigned int *ir_val, ret_val;
  unsigned int start_time;

  if (verbose >= 2) start_time = millis();
  if ((n == 0) || (n > 100) || (position < 0.0) || (position > 1)) return 0;
  if (position == 1.0) position = 0.999;

  ir_val = (unsigned int *)malloc(sizeof(unsigned int) * n);
  if (ir_val == NULL) return 0;

  for (int i = 0; i < (int)n; i++) ir_val[i] = analogRead(PIN_IR);

  qsort(ir_val, n, sizeof(unsigned int), compare);
  ret_val = ir_val[(unsigned int)(n * position)];

  free(ir_val);

  if (verbose >= 2) {
    Serial.print("Elapsed time: ");
    Serial.print(millis() - start_time);
    Serial.println("ms");
  }
  return ret_val;
}

void polyfit(float *x, float *y, int n, int degree, float *outCoeff) {
  if (degree > MAX_DEGREE) degree = MAX_DEGREE;
  if (degree >= n) degree = n - 1;

  int m = degree + 1;
  float A[MAX_DEGREE + 1][MAX_DEGREE + 1];
  float B[MAX_DEGREE + 1];

  for (int k = 0; k < m; k++) {
    for (int l = 0; l < m; l++) {
      double sum = 0.0;
      for (int i = 0; i < n; i++) {
        double v = 1.0;
        int p = k + l;
        for (int t = 0; t < p; t++) v *= x[i];
        sum += v;
      }
      A[k][l] = (float)sum;
    }
  }

  for (int k = 0; k < m; k++) {
    double sum = 0.0;
    for (int i = 0; i < n; i++) {
      double v = 1.0;
      for (int t = 0; t < k; t++) v *= x[i];
      sum += y[i] * v;
    }
    B[k] = (float)sum;
  }

  for (int i = 0; i < m; i++) {
    int pivot = i;
    for (int j = i + 1; j < m; j++) {
      if (fabs(A[j][i]) > fabs(A[pivot][i])) pivot = j;
    }
    if (pivot != i) {
      for (int k = 0; k < m; k++) {
        float tmp = A[i][k];
        A[i][k] = A[pivot][k];
        A[pivot][k] = tmp;
      }
      float tmpB = B[i];
      B[i] = B[pivot];
      B[pivot] = tmpB;
    }

    float diag = A[i][i];
    if (fabs(diag) < 1e-8) continue;

    for (int k = i; k < m; k++) A[i][k] /= diag;
    B[i] /= diag;

    for (int j = i + 1; j < m; j++) {
      float factor = A[j][i];
      for (int k = i; k < m; k++) A[j][k] -= factor * A[i][k];
      B[j] -= factor * B[i];
    }
  }

  for (int i = m - 1; i >= 0; i--) {
    float sum = B[i];
    for (int k = i + 1; k < m; k++) sum -= A[i][k] * outCoeff[k];
    outCoeff[i] = sum / (fabs(A[i][i]) < 1e-8 ? 1.0 : A[i][i]);
  }

  for (int i = m; i <= MAX_DEGREE; i++) outCoeff[i] = 0.0;
}

float volt_to_distance(unsigned int a_value) {
  float x = (float)a_value;
  float y = 0.0;
  float xp = 1.0;
  for (int i = 0; i <= MAX_DEGREE; i++) {
    y += coeff[i] * xp;
    xp *= x;
  }
  return y;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("=== IR Sensor Curve Fitting ===");
  Serial.println("Enter polynomial degree (1~3):");

  int degree = 0;
  while (degree < 1 || degree > 3) {
    while (Serial.available() == 0) {}
    degree = Serial.parseInt();
    while (Serial.available() > 0) Serial.read();
  }

  float x[MAX_POINTS];
  float y[MAX_POINTS];

  for (int i = 0; i < MAX_POINTS; i++) {
    float dist = 5.0 * i;
    y[i] = dist;

    Serial.print("Place at ");
    Serial.print(dist);
    Serial.println(" cm and press Enter");

    while (Serial.available() > 0) Serial.read();
    while (Serial.available() == 0) {}
    while (Serial.available() > 0) Serial.read();

    unsigned int filtered = ir_sensor_filtered(SAMPLE_N, 0.5, 0);
    x[i] = (float)filtered;

    Serial.print("DIST: ");
    Serial.print(dist);
    Serial.print(", FLT:");
    Serial.println(filtered);
  }

  polyfit(x, y, MAX_POINTS, degree, coeff);

  Serial.println("=== Result Equation ===");
  Serial.print("distance = ");
  for (int i = 0; i <= degree; i++) {
    if (i == 0) Serial.print(coeff[i], 6);
    else {
      if (coeff[i] >= 0) Serial.print(" + ");
      else Serial.print(" - ");
      float c = fabs(coeff[i]);
      Serial.print(c, 6);
      Serial.print("*x");
      if (i >= 2) {
        Serial.print("^");
        Serial.print(i);
      }
    }
  }
  Serial.println(";");

  Serial.println();
  Serial.println("float volt_to_distance(unsigned int a_value) {");
  Serial.println("  float x = (float)a_value;");
  Serial.print("  return ");
  for (int i = 0; i <= degree; i++) {
    if (i == 0) Serial.print(coeff[i], 6);
    else {
      if (coeff[i] >= 0) Serial.print(" + ");
      else Serial.print(" - ");
      float c = fabs(coeff[i]);
      Serial.print(c, 6);
      Serial.print(" * x");
      if (i >= 2) {
        for (int k = 1; k < i; k++) Serial.print(" * x");
      }
    }
  }
  Serial.println(";");
  Serial.println("}");
  Serial.println();
  Serial.println("Done.");
}

void loop() {}

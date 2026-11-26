/*
  ____  _  _    __     __            _             
 |  _ \| || |   \ \   / /__ _ __ ___(_) ___  _ __  
 | |_) | || |_   \ \ / / _ \ '__/ __| |/ _ \| '_ \ 
 |  _ <|__   _|   \ V /  __/ |  \__ \ | (_) | | | |
 |_| \_\  |_|      \_/ \___|_|  |___/_|\___/|_| |_|
                                                   
*/
#include <ArduinoJson.h>
#include "ArduinoGraphics.h"
#include "Arduino_LED_Matrix.h"
#include <Modulino.h>

#define VERSION "1.4.0"
#define BAUD_RATE 115200
// Main lood delay: this is typically a small value, used to save power
#define DELAY 40UL  // microseconds
// Sampling period: this is the time between samples, must be larger than DELAY
#define TIMESTEP 160UL  // milliseconds
// Topmost field in the output JSON
#define DATA_FIELD "data"
// IMU zero calibration time at startup
#define CALIB_TIME 2000UL  // milliseconds

ArduinoLEDMatrix matrix;
ModulinoMovement imu;
JsonDocument doc;
String out;
// Rotation from sensor to calibrated frame (rows are calibrated X, Y, Z axes)
float R[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
float gyro_bias[3] = {0, 0, 0};
bool swap_xy = true;  // swap X and Y axes in the bubble display

void cross(const float a[3], const float b[3], float out[3]) {
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}

void normalize(float v[3]) {
  float n = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  for (int i = 0; i < 3; i++) v[i] /= n;
}

void rotate(const float v[3], float out[3]) {
  for (int i = 0; i < 3; i++)
    out[i] = R[i][0] * v[0] + R[i][1] * v[1] + R[i][2] * v[2];
}

// Average readings for CALIB_TIME (IMU must be still): mean acceleration is
// gravity and defines Z; the sensor axis X or Y closer to horizontal, projected
// on the horizontal plane, defines the calibrated X or Y; mean gyro is the bias
void calibrate() {
  float g[3] = {0, 0, 0};
  unsigned long n = 0, t0 = millis();
  Serial.print("# Calibrating IMU, keep it still...\n");
  while (millis() - t0 < CALIB_TIME) {
    if (imu.update()) {
      g[0] += imu.getX(); g[1] += imu.getY(); g[2] += imu.getZ();
      gyro_bias[0] += imu.getRoll();
      gyro_bias[1] += imu.getPitch();
      gyro_bias[2] += imu.getYaw();
      n++;
    }
    delay(10);
  }
  if (n == 0) {
    Serial.print("# ERROR: no IMU readings, calibration skipped\n");
    return;
  }
  for (int i = 0; i < 3; i++) gyro_bias[i] /= n;
  float *x = R[0], *y = R[1], *z = R[2];
  memcpy(z, g, sizeof(g));
  normalize(z);
  if (fabs(z[0]) <= fabs(z[1])) {  // sensor X is closer to horizontal
    for (int i = 0; i < 3; i++) x[i] = (i == 0) - z[0] * z[i];
    normalize(x);
    cross(z, x, y);
  } else {
    for (int i = 0; i < 3; i++) y[i] = (i == 1) - z[1] * z[i];
    normalize(y);
    cross(y, z, x);
  }
  Serial.print("# Calibration done\n");
}

// 2x2 bubble on the 12x8 matrix: centered when level, at the edge when the
// calibrated X (columns) or Y (rows) axis is tilted by 90°
void draw_bubble(const float a[3]) {
  static byte frame[8][12];
  float n = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  if (n == 0) return;
  float tx = degrees(asin(constrain(a[0] / n, -1.0f, 1.0f)));
  float ty = -degrees(asin(constrain(a[1] / n, -1.0f, 1.0f)));
  if (swap_xy) {
    float t = tx;
    tx = ty;
    ty = t;
  }
  int c = 5 + round(tx / 90.0 * 5);
  int r = 3 - round(ty / 90.0 * 3);
  memset(frame, 0, sizeof(frame));
  frame[r][c] = frame[r][c + 1] = frame[r + 1][c] = frame[r + 1][c + 1] = 1;
  matrix.loadPixels(&frame[0][0], 12 * 8);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUD_RATE);
  Serial.print("# Starting JSON reader v" VERSION "\n");

  // LED Matrix
  matrix.begin();

  // Modulino 6-DoF IMU
  Modulino.begin();
  if (!imu.begin()) Serial.print("# ERROR: Modulino Movement not found\n");
  calibrate();

  pinMode(LED_BUILTIN, OUTPUT);
}

template<typename T>
T threshold_value(T value, T threshold) {
  return fabs(value) > threshold ? value : 0;
}

void loop() {
  static unsigned long prev_time = 0;
  static unsigned long timestep_us = TIMESTEP * 1000;
  static unsigned long delay = DELAY;
  static unsigned int threshold = 1;
  static bool onoff = LOW, pause = false, raw = false;
  unsigned long now = micros();
  static unsigned long v = 0;  // accumulator for serial values

    // Serial input
    // This allows to change parameters on the fly. Toggle parameters are read
    // as a single character, while commands that take a numeric argument
    // (integer) follow this convention: 123X, where 123 is the value and X is the
    // command. The commands are always single characters and correspond
    // to the following switch cases:
    char ch;
  // Read serial in
  if (Serial.available()) {
    ch = Serial.read();
    switch (ch) {
      case '0' ... '9':  // numeric value accumulator
        v = v * 10 + ch - '0';
        break;
      case 'p':  // set timestep_us to the current accumulator value
        timestep_us = constrain(v * 1000, 1000, 1E6);
        v = 0;
        break;
      case 'd':  // set delay to the current accumulator value
        delay = constrain(v, 1, timestep_us / 10.0);
        v = 0;
        break;
      case 't':  // set threshold to the current accumulator value
        threshold = constrain(v, 0, 16384);
        v = 0;
        break;
      case 'x':  // toggle pause mode
        pause = !pause;
        prev_time = micros();  // don't integrate yaw over the pause
        break;
      case 'r':  // toggle raw mode
        raw = !raw;
        break;
      case 's':  // toggle X/Y swap in the bubble
        swap_xy = !swap_xy;
        break;
      case '?':  // provide inline help
        Serial.print("Version: " VERSION "\n");
        Serial.print("Usage:\n");
        Serial.print("- 10p  set sampling period to 10 milliseconds (now ");
        Serial.print(timestep_us / 1000);
        Serial.print(" ms)\n- 30d  set loop delay to 30 microseconds (now ");
        Serial.print(delay);
        Serial.print(" us)\n- 280t set threshold to 280 thousandths of g or dps (now ");
        Serial.print(threshold);
        Serial.print(")\n");
        Serial.print("- x    toggle pause\n");
        Serial.print("- r    toggle raw output\n");
        Serial.print("- s    toggle X/Y swap in the LED bubble\n");
        break;
      default:
        v = 0;
    }
  }

  if (pause) return;

  if (now - prev_time >= timestep_us) {
    bool active = false;
    digitalWrite(LED_BUILTIN, onoff);
    onoff = !onoff;
    doc["millis"] = millis();

    // IMU: acceleration in g, angular rate in dps, both in calibrated frame
    imu.update();
    float th = threshold / 1000.0;
    float a[3] = {imu.getX(), imu.getY(), imu.getZ()};
    float w[3] = {imu.getRoll() - gyro_bias[0], imu.getPitch() - gyro_bias[1],
                  imu.getYaw() - gyro_bias[2]};
    float vals[6];
    rotate(a, vals);
    rotate(w, vals + 3);
    draw_bubble(vals);
    JsonArray acc = doc[DATA_FIELD]["acceleration"].to<JsonArray>();
    JsonArray gyro = doc[DATA_FIELD]["gyro"].to<JsonArray>();
    for (int i = 0; i < 6; i++) {
      vals[i] = threshold_value<float>(vals[i], th);
      (i < 3 ? acc : gyro).add(vals[i]);
      active = active || (vals[i] != 0);
    }
    // Angles in degrees: roll and pitch from gravity, yaw by integrating the
    // (thresholded, so the threshold acts as a dead band) Z angular rate
    static float yaw = 0;
    float dt = prev_time ? (now - prev_time) * 1e-6 : 0;
    yaw += vals[5] * dt;
    float angles[3] = {
      atan2(vals[1], vals[2]),
      atan2(-vals[0], sqrt(vals[1] * vals[1] + vals[2] * vals[2])),
      yaw / 180.0 * PI};
    JsonArray ang = doc[DATA_FIELD]["angles"].to<JsonArray>();
    for (int i = 0; i < 3; i++) ang.add(angles[i]);
    if (active > 0) {
      if (raw) {
        for (int i = 0; i < 6; i++) {
          Serial.print(vals[i], 4);
          Serial.print(" ");
        }
        for (int i = 0; i < 3; i++) {
          Serial.print(angles[i], 2);
          Serial.print(i < 2 ? " " : "\n");
        }
      } else {
        serializeJson(doc, out);
        Serial.print(out);
        Serial.print("\n");
      }
    }
    prev_time = now;
  }
  delayMicroseconds(delay);
}

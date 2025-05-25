#include <Wire.h>
#include <math.h>

#define mpu_address 0x68
#define pitch       0
#define roll        1
#define yaw         2

const int pin_fl = 9;
const int pin_fr = 3;
const int pin_bl = 11;
const int pin_br = 10;

const float complementary_filter = 0.95;
const float pid_filter           = 0.92;

const float rad_to_deg = 180.0 / 3.141592654;

float throttle = 0;

float gain_p[3]            = { 1.2, 1.2, 1.2 };
float gain_i[3]            = { 0.0, 0.0, 0.0 };
float gain_d[3]            = { 0.6, 0.6, 0.6 };
float angle_desired[3]     = { 0.0, 0.0, 0.0 };

float error_current[3]     = { 0.0, 0.0, 0.0 };
float error_prev[3]        = { 0.0, 0.0, 0.0 };

float pid_p[3]             = { 0.0, 0.0, 0.0 };
float pid_i[3]             = { 0.0, 0.0, 0.0 };
float pid_d[3]             = { 0.0, 0.0, 0.0 };
float pid_current[3]       = { 0.0, 0.0, 0.0 };

float angle_current[3]     = { 0.0, 0.0, 0.0 };
float angle_acc[3]         = { 0.0, 0.0, 0.0 };
float angle_gyro[3]        = { 0.0, 0.0, 0.0 };
float angle_acc_raw[3]     = { 0.0, 0.0, 0.0 };
float angle_gyro_raw[3]    = { 0.0, 0.0, 0.0 };

float angle_acc_offset[3]  = { 0.0, 0.0, 0.0 };
float angle_gyro_offset[3] = { 0.0, 0.0, 0.0 };

unsigned long time_current = 0;
unsigned long time_prev    = 0;
double time_elapsed        = 0.0;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  Wire.begin();
  Wire.beginTransmission(mpu_address);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Wire.beginTransmission(mpu_address);
  Wire.write(0x1A);
  Wire.write(0x06);
  Wire.endTransmission();

  pinMode(pin_fl, OUTPUT);
  pinMode(pin_fr, OUTPUT);
  pinMode(pin_bl, OUTPUT);
  pinMode(pin_br, OUTPUT);

  time_current = millis();

  calibrate_angle_offsets();
  set_all_motors(0);
}

void calibrate_angle_offsets() {
  const int samples = 100;

  float gyro_sum[3] = { 0.0, 0.0, 0.0 };
  float acc_sum[3] = { 0.0, 0.0, 0.0 };

  for (int i = 0; i < samples; i++) {
    read_gyro();
    gyro_sum[pitch] += angle_gyro[pitch];
    gyro_sum[roll] += angle_gyro[roll];
    gyro_sum[yaw] += angle_gyro[yaw];

    read_accelerometer();
    acc_sum[pitch] += angle_acc[pitch];
    acc_sum[roll] += angle_acc[roll];
    acc_sum[yaw] += angle_acc[yaw];

    delay(10);
  }

  angle_gyro_offset[pitch] = gyro_sum[pitch] / samples;
  angle_gyro_offset[roll] = gyro_sum[roll] / samples;
  angle_gyro_offset[yaw] = gyro_sum[yaw] / samples;

  angle_acc_offset[pitch] = acc_sum[pitch] / samples;
  angle_acc_offset[roll] = acc_sum[roll] / samples;
  angle_acc_offset[yaw] = acc_sum[yaw] / samples;
}

void set_all_motors(int value) {
  analogWrite(pin_fr, constrain(value, 0, 255));
  analogWrite(pin_fl, constrain(value, 0, 255));
  analogWrite(pin_bl, constrain(value, 0, 255));
  analogWrite(pin_br, constrain(value, 0, 255));
}

void loop() {
  time_prev = time_current;
  time_current = millis();
  time_elapsed = (time_current - time_prev) / 1000.0;

  read_gyro();
  read_accelerometer();

  filter_angle();

  calculate_pid();

  if (Serial.available()) {
    int new_throttle = Serial.parseInt();

    if (new_throttle >= 0 && new_throttle <= 255) {
      throttle = new_throttle;

      Serial.print("Throttle set to: ");
      Serial.println(throttle);
    }

  }

  set_motor_pids();
}

void read_gyro() {
  Wire.beginTransmission(mpu_address);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(mpu_address, 6, true);

  angle_gyro_raw[pitch] = (Wire.read() << 8 | Wire.read()) / 131.0;
  angle_gyro_raw[roll] = (Wire.read() << 8 | Wire.read()) / 131.0;
  angle_gyro_raw[yaw] = (Wire.read() << 8 | Wire.read()) / 131.0;

  angle_gyro[pitch] = angle_gyro_raw[pitch];
  angle_gyro[roll] = angle_gyro_raw[roll];
  angle_gyro[yaw] = angle_gyro_raw[yaw];

  angle_gyro[pitch] -= angle_gyro_offset[pitch];
  angle_gyro[roll] -= angle_gyro_offset[roll];
  angle_gyro[yaw] -= angle_gyro_offset[yaw];
}

void read_accelerometer() {
  Wire.beginTransmission(mpu_address);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(mpu_address, 6, true);

  angle_acc_raw[pitch] = (Wire.read() << 8 | Wire.read()) / 16384.0;
  angle_acc_raw[roll] = (Wire.read() << 8 | Wire.read()) / 16384.0;
  angle_acc_raw[yaw] = (Wire.read() << 8 | Wire.read()) / 16384.0;

  angle_acc[pitch] = atan2(angle_acc_raw[roll], sqrt(pow(angle_acc_raw[pitch], 2) + pow(angle_acc_raw[yaw], 2))) * rad_to_deg;
  angle_acc[roll] = atan2(-angle_acc_raw[pitch], sqrt(pow(angle_acc_raw[roll], 2) + pow(angle_acc_raw[yaw], 2))) * rad_to_deg;
  angle_acc[yaw] = 0.0;

  angle_acc[pitch] -= angle_acc_offset[pitch];
  angle_acc[roll] -= angle_acc_offset[roll];
  angle_acc[yaw] -= angle_acc_offset[yaw];
}

void filter_angle() {
  float angle_new[3];

  angle_new[pitch] = -(complementary_filter * (-angle_current[pitch] + angle_gyro[pitch] * time_elapsed) + (1 - complementary_filter) * angle_acc[pitch]);
  angle_new[roll] = complementary_filter * (angle_current[roll] + angle_gyro[roll] * time_elapsed) + (1 - complementary_filter) * angle_acc[roll];
  angle_new[yaw] = complementary_filter * (angle_current[yaw] + angle_gyro[yaw] * time_elapsed) + (1 - complementary_filter) * angle_acc[yaw];

  float blend = 0.5;
  angle_current[pitch] = blend * angle_current[pitch] + (1 - blend) * angle_new[pitch];
  angle_current[roll] = blend * angle_current[roll] + (1 - blend) * angle_new[roll];
  angle_current[yaw] = blend * angle_current[yaw] + (1 - blend) * angle_new[yaw];
}

void calculate_pid() {
  error_prev[pitch] = error_current[pitch];
  error_prev[roll] = error_current[roll];
  error_prev[yaw] = error_current[yaw];

  error_current[pitch] = angle_current[pitch] - angle_desired[pitch];
  error_current[roll] = angle_current[roll] - angle_desired[roll];
  error_current[yaw] = angle_current[yaw] - angle_desired[yaw];

  pid_p[pitch] = gain_p[pitch] * error_current[pitch];
  pid_p[roll] = gain_p[roll] * error_current[roll];
  pid_p[yaw] = gain_p[yaw] * error_current[yaw];

  float pid_d_new[3];
  pid_d_new[pitch] = gain_d[pitch] * ((error_current[pitch] - error_prev[pitch]) / time_elapsed);
  pid_d_new[roll] = gain_d[roll] * ((error_current[roll] - error_prev[roll]) / time_elapsed);
  pid_d_new[yaw] = gain_d[yaw] * ((error_current[yaw] - error_prev[yaw]) / time_elapsed);

  pid_d[pitch] = pid_filter * pid_d[pitch] + (1 - pid_filter) * pid_d_new[pitch];
  pid_d[roll] = pid_filter * pid_d[roll] + (1 - pid_filter) * pid_d_new[roll];
  pid_d[yaw] = pid_filter * pid_d[yaw] + (1 - pid_filter) * pid_d_new[yaw];

  pid_current[pitch] = pid_p[pitch] + pid_i[pitch] + pid_d[pitch];
  pid_current[roll] = pid_p[roll] + pid_i[roll] + pid_d[roll];
  pid_current[yaw] = pid_p[yaw] + pid_i[yaw] + pid_d[yaw];
}

void set_motor_pids() {
  int motor_fr = constrain(throttle + pid_current[pitch] + pid_current[roll] + pid_current[yaw], 0, 255);
  int motor_fl = constrain(throttle + pid_current[pitch] - pid_current[roll] - pid_current[yaw], 0, 255);
  int motor_bl = constrain(throttle - pid_current[pitch] - pid_current[roll] + pid_current[yaw], 0, 255);
  int motor_br = constrain(throttle - pid_current[pitch] + pid_current[roll] - pid_current[yaw], 0, 255);

  analogWrite(pin_fr, motor_fr);
  analogWrite(pin_fl, motor_fl);
  analogWrite(pin_bl, motor_bl);
  analogWrite(pin_br, motor_br);
}

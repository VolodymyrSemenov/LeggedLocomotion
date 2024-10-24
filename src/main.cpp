#include <Arduino.h>
#include <Wire.h>
#include "AS5600.h"

#include <stdlib.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <cmath>

#include "headers.hpp"

struct Filter {
  int prev_time;
  float prev_filtered;
  float prev_noisy;
  float omega_c;

  Filter(float c) {
    prev_time = micros();
    prev_filtered = 0;
    prev_noisy = 0;
    omega_c = c;
  }

  float next_value(float new_noisy) {
    int current_time = micros();
    float dt = (current_time - prev_time) / 1000000;  // Not Used
    float ret = (1.0 - omega_c) * prev_filtered + omega_c * prev_noisy;

    prev_time = current_time;
    prev_filtered = ret;
    prev_noisy = new_noisy;

    return ret;
  }
} leg_filter(0.05), leg_speed_filter(0.05), rotary_speed_filter(0.05), height_speed_filter(0.05), no_filter(1);

struct Motor {
  char name;
  int p0;
  int p1;
  float pwm_value;
  bool backwards;
  AS5600L *encoder;
  Filter *filter;
  float offset;
  float integration_error;
  float previous_error;
  float previous_time;
  bool initialized;


  void drive_motor(int offset = 0, int cutoff = 0) {  // Default Values of 0
    // Cutoff is value to stop motor
    // Offset can be used to eliminate motor deadzone
    int speed;  // Rounds Down,       -1.5 -> 2
    if (backwards) {
      speed = floor(-1 * pwm_value);
    } else {
      speed = floor(pwm_value);
    }
    if (speed >= 0) {        // Positive Speed
      if (speed < cutoff) {  // If Very Low speed turn motor Off
        digitalWrite(p0, LOW);
        digitalWrite(p1, LOW);
      } else {  // Otherwise drive forward
        digitalWrite(p0, LOW);
        analogWrite(p1, speed + offset);
      }
    } else {                      // Negative Speed (Drive Motors Backward)
      if (speed > cutoff * -1) {  // If very low speed turn motor off
        digitalWrite(p1, LOW);
        digitalWrite(p0, LOW);
      } else {  // Otherwise drive backward
        digitalWrite(p1, LOW);
        analogWrite(p0, (speed + offset) * -1);
      }
    }
  }
};

const double degree_to_rotation = 1.0 / 360;

Adafruit_LSM9DS1 imu = Adafruit_LSM9DS1(&Wire1, 0x6B);

AS5600L as5600_leg(0x36, &Wire);
AS5600L as5600_rotary(0x36, &Wire1);
AS5600L as5600_height(0x40, &Wire1);

Motor m1 = { '1', 2, 3, 0, false, &as5600_leg, &leg_filter, 0 };

void IMU_init() {
  if (!imu.begin()) {
    Serial.println("Can't find IMU");
  } else {
    Serial.println("Found IMU");
    // ACCELRANGE 10, 119, 476, 952
    // ACCELDATARATE 2, 4, 8, 16
    imu.setupAccel(imu.LSM9DS1_ACCELRANGE_2G, imu.LSM9DS1_ACCELDATARATE_952HZ);
    // MAGGAIN 4 8 12 16
    imu.setupMag(imu.LSM9DS1_MAGGAIN_4GAUSS);
    // GYROSCALE 245, 500, 2000
    imu.setupGyro(imu.LSM9DS1_GYROSCALE_245DPS);
  }
}


void motor_init(Motor m) {
  m.encoder->setOffset(m.offset);
  pinMode(m.p0, OUTPUT);
  pinMode(m.p1, OUTPUT);
  analogWriteResolution(12);
  analogWriteFreq(100000);
  if (!m.encoder->detectMagnet()) {
    Serial.println("Cant detect Magnet");
  }
  if (m.encoder->magnetTooWeak()) {
    Serial.println("Magnet too weak");
  }
  if (m.encoder->magnetTooStrong()) {
    Serial.println("Magnet too Strong");
  }
}


void i2c_init() {
  Wire.setSDA(8);
  Wire.setSCL(9);
  Wire.setClock(400000);
  Wire.begin();
  Wire1.setSDA(26);
  Wire1.setSCL(27);
  Wire1.setClock(400000);
  Wire1.begin();
}


int pid_speed(Motor &m) {
  float desired_speed = -9;

  if (!m.initialized) {
    m.initialized = true;
    m.integration_error = 0;
    m.previous_error = desired_speed;
    m.previous_time = micros();
  }
  const float proportional_factor = 1;
  const float integral_factor = 0.0000;
  const float derivative_factor = 0.000000;

  float angular_speed = m.encoder->getAngularSpeed() * degree_to_rotation;
  float current_time = micros();
  //desired_speed = sin(current_time / 100000.0) * 1.5 + 6;
  float filtered = m.filter->next_value(angular_speed);
  float error = desired_speed - filtered;

  m.integration_error += (error + m.previous_error) * (current_time - m.previous_time) / 2;
  float derivative_error = (error - m.previous_error) / (current_time - m.previous_time);

  m.previous_error = error;
  m.previous_time = current_time;

  m.pwm_value += proportional_factor * error + integral_factor * m.integration_error + derivative_factor * derivative_error;
  m.drive_motor();

  Serial.print(m.name);
  Serial.print("_speed:");
  Serial.print(filtered);
  Serial.print(" ");
  Serial.print(m.name);
  Serial.print("_pwm:");
  Serial.println(m.pwm_value);
  delayMicroseconds(20);
  return error;
}


void pid_angle(Motor &m) {
  static float proportional_factor = 23;  // 23 works well
  static float integral_factor = 5;       // 5 works well   // Integral Factor gets divided later
  static float derivative_factor = 40;    // 2 works well   // Derivative Factor doesn't use time

  static float desired_angle = m.encoder->readAngle() * AS5600_RAW_TO_DEGREES;
  static float integration_error = 0;
  static float previous_error = 0;
  static float previous_time = micros();

  if (Serial.available()) {  // Controls to change PID and angle
    const int buffer_size = 10;
    char buffer[buffer_size] = {};  // All 0s
    Serial.readBytesUntil(10, buffer, buffer_size);

    switch (buffer[0]) {
      case 'p':
        proportional_factor = atof(buffer + 1);
        break;
      case 'i':
        integral_factor = atof(buffer + 1);
        break;
      case 'd':
        derivative_factor = atof(buffer + 1);
        break;
      case 'a':  // Switch Angle 90 degrees
        desired_angle = desired_angle < 160 ? (desired_angle < 40 ? 90 : 180) : (desired_angle < 240 ? 270 : 0);
        break;
      case 's':  // Switch Angle 180 degrees
        desired_angle = desired_angle < 160 ? 180 : 0;
        break;
      default:  // Set Angle
        desired_angle = atof(buffer);
        break;
    }
  }

  float current_angle = m.encoder->readAngle() * AS5600_RAW_TO_DEGREES;
  float current_time = micros();
  float error = desired_angle - current_angle;
  if (error > 180.0) {  // Change the error to the (shorter) complementary angle if it's above 180 or below -180
    error = error - 360;
  } else if (error < -180.0) {
    error = error + 360;
  }


  integration_error += (error + previous_error) * (current_time - previous_time) / 2;
  float derivative_error = (error - previous_error);  // / (current_time - previous_time)     // Works better without time

  previous_error = error;
  previous_time = current_time;

  if (abs(error) < 0.3) {
    m.pwm_value = 0;
    m.drive_motor(0);
  }
  m.pwm_value = error * proportional_factor + integration_error * integral_factor / 100000 + derivative_error * derivative_factor;  // (error > 0 ? 20: -20 );
  m.drive_motor(160);

  // Logging
  Serial.print(m.name);
  Serial.print("Current_Angle:");
  Serial.print(current_angle);
  Serial.print(" ");
  Serial.print(m.name);
  Serial.print("Desired_Angle:");
  Serial.print(desired_angle);
  Serial.println(" T:500 B:0");
  delayMicroseconds(300);
}


void sensor_average_reading() {
  imu.readAccel(); /* ask it to read in the data */

  /* Get a new sensor event */
  sensors_event_t a, m, g, temp;

  imu.getEvent(&a, &m, &g, &temp);
  static float x[100];
  static float y[100];
  static float z[100];
  static int i = 0;

  x[i] = a.acceleration.x;
  y[i] = a.acceleration.y;
  z[i] = a.acceleration.z;
  i = (i + 1) % 100;

  float xs = 0.0;
  float ys = 0.0;
  float zs = 0.0;
  for (int i = 0; i < 100; i++) {
    xs += x[i];
    ys += y[i];
    zs += z[i];
  }
  static float min_x = 0;
  static float max_x = 0;
  static float min_y = 0;
  static float max_y = 0;
  static float min_z = 0;
  static float max_z = 0;
  min_x = min(min_x, xs);
  max_x = max(max_x, xs);
  min_y = min(min_y, ys);
  max_y = max(max_y, ys);
  min_z = min(min_z, zs);
  max_z = max(max_z, zs);
  Serial.print("T:1000 B:0 ");
  Serial.print(min_x / 100);
  Serial.print(" ");
  Serial.print(max_x / 100);
  Serial.print(" ");
  Serial.print(min_y / 100);
  Serial.print(" ");
  Serial.print(max_y / 100);
  Serial.print(" ");
  Serial.print(min_z / 100);
  Serial.print(" ");
  Serial.print(max_z / 100);
  Serial.println(" ");
}


void get_acceleration(sensors_event_t a) {
  imu.readAccel(); /* ask it to read in the data */
  sensors_event_t m, g, temp;

  imu.getEvent(&a, &m, &g, &temp);
  float x = a.acceleration.x;
  float y = a.acceleration.y;
  float z = a.acceleration.z;
  Serial.print(0.2 + x * 19.62 / 19.6);
  Serial.print(" ");
  Serial.print(-0.15 + y * 19.62 / 19.7);
  Serial.print(" ");
  Serial.print(0.225 + z * 19.62 / 19.65);
  Serial.println(" ");
}


void sensor_reading() {
  imu.read(); /* ask it to read in the data */

  /* Get a new sensor event */
  sensors_event_t a, m, g, temp;
  static float vx = 0;
  static float vy = 0;
  static float vz = 0;
  imu.getEvent(&a, &m, &g, &temp);

  // Serial.print("Accel_X:"); Serial.print(a.acceleration.x); //Serial.print(" m/s^2");
  // Serial.print("\tAccel_Y:"); Serial.print(a.acceleration.y);     //Serial.print(" m/s^2 ");
  // Serial.print("\tAccel_Z:"); Serial.print(a.acceleration.z);     //Serial.println(" m/s^2 ");

  // Serial.print("\tMag_X:"); Serial.print(m.magnetic.x);   //Serial.print(" uT");
  // Serial.print("\tMag_Y:"); Serial.print(m.magnetic.y);     //Serial.print(" uT");
  // Serial.print("\tMag_Z:"); Serial.println(m.magnetic.z);     //Serial.println(" uT");
  Serial.print("T:1000 B:0 ");
  Serial.print(vx);
  Serial.print(" ");
  Serial.print(vy);
  Serial.print(" ");
  Serial.print(vz);
  Serial.print(" ");
  Serial.print(a.acceleration.x);
  Serial.print(" ");
  Serial.print(a.acceleration.y);
  Serial.print(" ");
  Serial.print(a.acceleration.z);
  Serial.println(" ");
  //vx += a.acceleration.x;
  //vy += a.acceleration.y;
  //vz += a.acceleration.z;
  //Serial.print("Accel:"); Serial.println(a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z);
  // Serial.print("Gyro_X:"); Serial.print(g.gyro.x);   //Serial.print(" rad/s");
  // Serial.print("\tGyro_Y:"); Serial.print(g.gyro.y);      //Serial.print(" rad/s");
  // Serial.print("\tGyro_Z:"); Serial.print(g.gyro.z);      //Serial.println(" rad/s");

  delay(1);
}


void print_angle(Motor &m) {
  m.pwm_value = 20000000;
  m.drive_motor();
  Serial.print(m.name);
  Serial.print(":");
  Serial.println(m.encoder->readAngle() * AS5600_RAW_TO_DEGREES);
  delayMicroseconds(20);
}


int get_touchdown(double vx, double vy, double theta) {
  double v0 = pow((vx * vx + vy * vy), 1 / 2);
  constexpr float candidate_tds[TDS] = TD_CANDIDATES;
  constexpr float lut[BETAS][TDS] = LUT;
  return 40;
}


void setup() {
  i2c_init();
  //while(!Serial); // Wait for serial monitor to open
  //Serial.println("-------");
  motor_init(m1);
  IMU_init();
  delay(2000);

  Serial1.setTX(12);
  Serial1.setRX(13);
  Serial1.begin(9600); //38400
  Serial.begin(115200);
}


double get_speed(AS5600L &encoder, Filter &f, bool no_distance=false) {
  const double degree_to_distance = (0.34 * 2 * 3.14159) / 360.0;
  double current_angular_speed = encoder.getAngularSpeed();
  double filtered_speed = f.next_value(current_angular_speed) * (no_distance ? 1 : degree_to_distance);
  return filtered_speed;
}



template <typename T>
void print_diagnostics(T uart) {
  // uart.print(">leg_speed:");
  // uart.println(get_speed(as5600_leg, leg_speed_filter));
  // uart.print(">rotary_speed:");
  // uart.println(get_speed(as5600_rotary, rotary_speed_filter));
  // uart.print(">height_speed:");
  // uart.println(get_speed(as5600_height, height_speed_filter, true));
  uart.print(">current_height:");
  uart.println(as5600_height.readAngle() * AS5600_RAW_TO_DEGREES);
  uart.print(">current_leg:");
  uart.println(as5600_leg.readAngle() * AS5600_RAW_TO_DEGREES);
  uart.print(">current_rotary:");
  uart.println(as5600_rotary.readAngle() * AS5600_RAW_TO_DEGREES);
}

float clip(float val, float max) {
  if (val <= -1 * max) {
    return max * -1;
  }
  if (val >= max) {
    return max;
  }
  return val;
}


void pid_desired_angle(Motor &m, float desired_angle) {
  static float proportional_factor = 23;  // 23 works well
  static float integral_factor = 5;       // 5 works well   // Integral Factor gets divided later
  static float derivative_factor = 2;    // 2 works well   // Derivative Factor doesn't use time

  static float integration_error = 0;
  static float previous_error = 0;
  static float previous_time = micros();


  float current_angle = m.encoder->readAngle() * AS5600_RAW_TO_DEGREES;
  float current_time = micros();
  float error = desired_angle - current_angle;
  if (error > 180.0) {  // Change the error to the (shorter) complementary angle if it's above 180 or below -180
    error = error - 360;
  } else if (error < -180.0) {
    error = error + 360;
  }

  integration_error += (error + previous_error) * (current_time - previous_time) / 2;
  float derivative_error = (error - previous_error);  // / (current_time - previous_time)     // Works better without time

  previous_error = error;
  previous_time = current_time;

  if (abs(error) < 0.3) {
    m.pwm_value = 0;
    m.drive_motor(0);
  }
  m.pwm_value = error * proportional_factor + integration_error * integral_factor / 100000 + derivative_error * derivative_factor;  // (error > 0 ? 20: -20 );
  m.drive_motor(160);

}


void controller() {
  if (as5600_height.readAngle() * AS5600_RAW_TO_DEGREES < 58) {
    pid_desired_angle(m1, 30);
  } else {
    m1.pwm_value = 4095;
    m1.drive_motor();
  }  
}

void update_sensor_readings(){ 

}

void loop() {
  static int counter = 0;
  // Serial.print(as5600_leg.readAngle() * AS5600_RAW_TO_DEGREES);
  // Serial.print(" ");
  // Serial.print(as5600_height.readAngle() * AS5600_RAW_TO_DEGREES);
  // Serial.print(" ");
  // Serial.println(as5600_rotary.readAngle() * AS5600_RAW_TO_DEGREES);

  //Serial.print(" ");
  // Serial.print("T:5 B:-5 Rotation:");
  // Serial.print(get_speed(as5600_rotary, f2));
  // Serial.print(" Height:");
  // Serial.println(get_speed(as5600_height, f3));
  // Serial1.println("Hi");
  // if(Serial.available()) {
  //   char read = Serial.read();
  //   Serial.print(read);
  //   Serial1.print(read);
  // }

  // if(Serial1.available()) {
  //   char read = Serial1.read();
  //   Serial.print(read);
  // }

  // Serial1.print("Hello World");
  update_sensor_readings();
  controller();

  if (counter = 100) {
    print_diagnostics(Serial1);
  }

  counter = counter == 100 ? 0 : counter++;
  delayMicroseconds(2);
  //delay(2);
}

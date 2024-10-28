#include <Arduino.h>
#include <Wire.h>
#include "AS5600.h"

#include <stdlib.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <cmath>

#include "headers.hpp"


const double degree_to_rotation = 1.0 / 360;

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


struct Encoder : public AS5600L{ 
  float current_angle;
  
  Encoder(uint8_t address, TwoWire *wire)
    : AS5600L(address, wire)
    , current_angle(0)
  {}

  void update_angle() {
    float angle = readAngle();
    current_angle = angle * AS5600_RAW_TO_DEGREES;
  }
};


struct Motor {
  char name;
  int p0;
  int p1;
  Encoder *encoder;
  float offset;
  float pwm_value;
  bool backwards;


  void drive_motor(int offset = 0, int cutoff = 0) {  // Default Values of 0
    // Cutoff is value to stop motor
    // Offset can be used to eliminate motor deadzone
    bool reverse = pwm_value < 0;
    int speed = floor(abs(pwm_value));  

    int lowpin = p0;
    int highpin = p1;

    if (backwards ^ reverse) { 
      int temp = p0;
      p0 = p1;
      p1 = temp;
    } 

    speed=abs(speed);

    if (speed < cutoff) { // If Very Low speed turn motor Off
      digitalWrite(lowpin, LOW);
      digitalWrite(highpin, LOW);
    } else {              // Otherwise drive forward
      digitalWrite(lowpin, LOW);
      analogWrite(highpin, speed + offset);
    }
  }

  void move_to_angle(float desired_angle) {
    static float proportional_factor = 23;  // 23 works well
    static float integral_factor = 5;       // 5 works well   // Integral Factor gets divided later
    static float derivative_factor = 2;    // 2 works well   // Derivative Factor doesn't use time

    static float integration_error = 0;
    static float previous_error = 0;
    static float previous_time = micros();

    float current_time = micros();
    float error = desired_angle - encoder->current_angle;

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
      pwm_value = 0;
      drive_motor();
    }
    pwm_value = error * proportional_factor + integration_error * integral_factor / 100000 + derivative_error * derivative_factor;
    drive_motor(160);
  }
};


Adafruit_LSM9DS1 imu = Adafruit_LSM9DS1(&Wire1, 0x6B);

Encoder as5600_leg(0x36, &Wire);
Encoder as5600_rotary(0x36, &Wire1);
Encoder as5600_height(0x40, &Wire1);

Motor m1 = {'1', 2, 3, &as5600_leg};


void IMU_init() {
  if (!imu.begin()) {
    Serial.println("Can't find IMU");
  } else {
    Serial.println("Found IMU");
    // ACCELRANGE 2, 4, 8, 16
    // ACCELDATARATE 10, 119, 476, 952
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


double get_speed(Encoder &encoder, Filter &f, bool no_distance=false) {
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
  uart.println(as5600_height.current_angle);
  uart.print(">current_leg:");
  uart.println(as5600_leg.current_angle);
  uart.print(">current_rotary:");
  uart.println(as5600_rotary.current_angle);
  uart.print(">pwm:");
  uart.println(m1.pwm_value);
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


void controller() {
  if (as5600_height.current_angle < 58) {
    m1.move_to_angle(30);
  } else {
    m1.pwm_value = 4095;
    m1.drive_motor();
  }  
}

void update_sensor_readings(){ 
  as5600_leg.update_angle();
  as5600_rotary.update_angle();
  as5600_height.update_angle();
}

void loop() {
  static int counter = 0;
  update_sensor_readings();
  controller();
    print_diagnostics(Serial);

  if (counter == 100) {
    print_diagnostics(Serial);
  }

  counter = counter == 100 ? 0 : counter++;
  delayMicroseconds(200);
}

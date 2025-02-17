#include <Arduino.h>
#include <Wire.h>
#include "AS5600.h"

#include <stdlib.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <cmath>

#include "headers.hpp"
#define GLOBALSERIAL Serial

const double degree_to_rotation = 1.0 / 360;
double imu_acceleration = 0;
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
} leg_filter(0.05), leg_speed_filter(0.05), rotary_speed_filter(0.05), height_speed_filter(0.05);


struct Encoder : public AS5600L{ 
  double current_angle;
  double filtered_speed;
  Filter speed_filter;
  
  Encoder(uint8_t address, TwoWire *wire)
    : AS5600L(address, wire)
    , current_angle(0)
    , filtered_speed(0)
    , speed_filter(Filter(0.05))
  {}

  void update_angle() {
    float angle = readAngle();
    current_angle = angle * AS5600_RAW_TO_DEGREES;
  }

  void update_speed(bool no_distance=false) {
    const double degree_to_distance = (0.34 * 2 * 3.14159) / 360.0;
    const double degree_to_rps = 1.0/360.0;
    double current_angular_speed = getAngularSpeed();
    filtered_speed = speed_filter.next_value(current_angular_speed) * (no_distance ? degree_to_rps : degree_to_distance);
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
      int temp = lowpin;
      lowpin = highpin;
      highpin = temp;
    } 


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
    static float integral_factor = 5 / 100000;       // 5 works well   // Integral Factor gets divided later
    static float derivative_factor = 2;    // 2 works well   // Derivative Factor doesn't use time

    static float integration_error = 0;
    static float previous_error = 0;
    static float previous_time = micros();

    float current_time = micros();
    float option1 = desired_angle;
    float option2 = (desired_angle + 180.0);
    if (option2 > 360) {
      option2 -= 360;
    }

    float error1 = (option1 - encoder->current_angle);
    float error2 = (option2 - encoder->current_angle);
    if (error1 > 180.0) {  // Change the error to the (shorter) complementary angle if it's above 180 or below -180
      error1 = error1 - 360;
    } else if (error1 < -180.0) {
      error1 = error1 + 360;
    }

    if (error2 > 180.0) {  // Change the error to the (shorter) complementary angle if it's above 180 or below -180
      error2 = error2 - 360;
    } else if (error2 < -180.0) {
      error2 = error2 + 360;
    }

    float error = abs(error1) < abs(error2) ? error1 : error2;

    integration_error += (error + previous_error) * (current_time - previous_time) / 2;
    float derivative_error = (error - previous_error);  // / (current_time - previous_time)     // Works better without time

    previous_error = error;
    previous_time = current_time;

    pwm_value = error * proportional_factor + integration_error * integral_factor + derivative_error * derivative_factor;
    drive_motor();
  }
};


Adafruit_LSM9DS1 imu = Adafruit_LSM9DS1(&Wire1, 0x6B);

Encoder as5600_leg(0x36, &Wire);
Encoder as5600_rotary(0x40, &Wire1);
Encoder as5600_height(0x36, &Wire1);

Motor m1 = {'1', 2, 3, &as5600_leg, 0, 0, false};


void IMU_init() {
  if (!imu.begin()) {
    GLOBALSERIAL.println("Can't find IMU");
  } else {
    GLOBALSERIAL.println("Found IMU");
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
  //analogWriteFreq(100000);
  if (!m.encoder->detectMagnet()) {
    GLOBALSERIAL.println("Cant detect Magnet");
  }
  if (m.encoder->magnetTooWeak()) {
    GLOBALSERIAL.println("Magnet too weak");
  }
  if (m.encoder->magnetTooStrong()) {
    GLOBALSERIAL.println("Magnet too Strong");
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
  GLOBALSERIAL.print(0.2 + x * 19.62 / 19.6);
  GLOBALSERIAL.print(" ");
  GLOBALSERIAL.print(-0.15 + y * 19.62 / 19.7);
  GLOBALSERIAL.print(" ");
  GLOBALSERIAL.print(0.225 + z * 19.62 / 19.65);
  GLOBALSERIAL.println(" ");
}


double current_acceleration() {
  imu.readAccel();
  imu.read(); /* ask it to read in the data */
  sensors_event_t a, m, g, temp;
  imu.getEvent(&a, &m, &g, &temp);
  return a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z;
}


int get_touchdown(double vx, double vy, double theta) {
  const double leg_length = 0.016;
  constexpr float candidate_tds[TDS] = TD_CANDIDATES;
  constexpr float lut[BETAS][TDS] = LUT;

  double takeoff_height = sin(theta) * leg_length;
  double v0 = pow((vx * vx + vy * vy), 1 / 2);

  double best_td = 0;
  double best_difference = 100000000000;
  for(int i=0; i<TDS ; i++) {
    double evaluate_td = candidate_tds[i];
    double final_height = sin(evaluate_td) * leg_length;
    double final_y_v = -1.0 * sqrt(vy * vy + 2 * (final_height - takeoff_height));
    double beta = final_y_v / vx;
    int beta_index_lower = floor((beta - BETA_START) / BETA_DIFFERENCE);
    int beta_index_upper = ceil((beta - BETA_START) / BETA_DIFFERENCE);
    double calculated_velocity = (lut[beta_index_lower][i] + lut[beta_index_upper][i]) / 2;
    double calculated_difference = abs(calculated_velocity - v0);
    if(calculated_difference < best_difference) {
      best_td = evaluate_td;
      best_difference = calculated_difference;
    }
    
  }

  
  return best_td * 57.2958;
}


void setup() {
  i2c_init();
  motor_init(m1);
  IMU_init();
  delay(2000);

  Serial1.setTX(12);
  Serial1.setRX(13);
  Serial1.begin(9600); //38400
  Serial.begin(115200); //15200
}


template <typename T>
void print_diagnostics(T uart) {
  uart.print(">leg_speed:");
  uart.println(as5600_leg.filtered_speed);
  uart.print(">rotary_speed:");
  uart.println(as5600_rotary.filtered_speed);
  uart.print(">height_speed:");
  uart.println(as5600_height.filtered_speed);
  uart.print(">current_height:");
  uart.println(as5600_height.current_angle);
  uart.print(">current_leg:");
  uart.println(as5600_leg.current_angle);
  uart.print(">current_rotary:");
  uart.println(as5600_rotary.current_angle);
  uart.print(">pwm:");
  uart.println(m1.pwm_value);  
  uart.print(">imu_acceleration:");
  uart.println(imu_acceleration);
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
  // static int flight_phase = 0;
  // if(flight_phase == 0) {
  //   if(imu_acceleration > 200) {
  //     flight_phase = 0;
  //   }
  // } else {
  //   if(imu_acceleration < 94) {
  //     flight_phase = 1;
  //   }
  // }

  if (as5600_height.current_angle > 297) {
    m1.move_to_angle(30);
  } else {
    m1.pwm_value = 2000;
    m1.drive_motor();
  }  
}

void update_sensor_readings(){ 
  as5600_leg.update_angle();
  as5600_rotary.update_angle();
  as5600_height.update_angle();
  as5600_leg.update_speed(true);
  as5600_rotary.update_speed();
  as5600_height.update_speed();
  //imu_acceleration = current_acceleration();
}

void loop() {
  static int counter = 0;
  update_sensor_readings();
  controller(); 
  const int max_count = 10;
  if (counter == max_count) {
    print_diagnostics(GLOBALSERIAL);
  }
  counter = counter == max_count ? 0 : ++counter;
  delayMicroseconds(100);
}

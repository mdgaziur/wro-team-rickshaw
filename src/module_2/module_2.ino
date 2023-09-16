#include <NewPing.h>
#include <basicMPU6050.h>

// Ultrasonic sensors
#define FRONT_T 13
#define FRONT_E 12
#define RIGHT_T 11
#define RIGHT_E 10
#define LEFT_T   9
#define LEFT_E   8
#define BACK_T   7
#define BACK_E   6

// Motor driver
#define ENA      5
#define IN2      4
#define IN3      3
#define ENB     23
#define IN4      2
#define IN5     22

// Object detection
#define LEFT   'L'
#define RIGHT  'R'
#define NONE   'N'

// Constants
#define MIN_DIST_THRES        20 // cm
#define MAX_DIST             150 // cm
#define TURN_ANGLE            45 // deg
#define TURN_DELAY_OBSTACLE  200 // ms
#define ALIGNMENT_DELAY      100 // ms
#define LR_ERROR_MARGIN        1 // cm
#define TURN_POWER           155
#define NORMAL_POWER         200
#define TURN_MOTOR_POWER     150
#define STOP_DELAY           300 // ms

basicMPU6050<> imu;
int turns = 0;
NewPing front_sensor(FRONT_T, FRONT_E, MAX_DIST);
NewPing right_sensor(RIGHT_T, RIGHT_E, MAX_DIST);
NewPing left_sensor(LEFT_T, LEFT_E, MAX_DIST);
NewPing back_sensor(BACK_T, BACK_E, MAX_DIST);

void set_power(int power) {
  analogWrite(ENB, power);
}

void go_forward() {
  digitalWrite(IN4, HIGH);
  digitalWrite(IN5, LOW);
}

void go_backward() {
  digitalWrite(IN4, LOW);
  digitalWrite(IN5, HIGH);
}

void stop_motor() {
  digitalWrite(IN4, LOW);
  digitalWrite(IN5, LOW);
}

void turn_left_no_delay() {
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
}

void turn_right_no_delay() {
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
}

void stop_turn() {
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
}

void turn_left(int delay_) {
  turn_left_no_delay();
  delay(delay_);
  stop_turn();
}

void turn_right(int delay_) {
  turn_right_no_delay();
  delay(delay_);
  stop_turn();
}

void turn_right_with_angle(float angle) {
  float last_yaw = read_yaw();

  turn_right_no_delay();
  while (abs(read_yaw() - last_yaw) < angle) {}
  stop_turn();
}

void turn_left_with_angle(float angle) {
  float last_yaw = read_yaw();

  turn_left_no_delay();
  while (abs(read_yaw() - last_yaw) < angle) {}
  stop_turn();
}

char read_serial() {
  return (char)Serial.read();
}

void turn_opposite() {
  set_power(TURN_POWER);
  float last_yaw = read_yaw();
  
  while (abs(read_yaw() - last_yaw) < 180.0) {
    turn_left_no_delay();
    go_backward();
    while (back_sensor.ping_cm() > MIN_DIST_THRES) {}

    go_forward();
    turn_right_no_delay();
    while (front_sensor.ping_cm() > MIN_DIST_THRES) {}
    stop_motor();
  }

  stop_turn();
  go_forward();
  set_power(NORMAL_POWER);
}

float read_yaw() {
  return imu.gy() * 180 / PI;
}

void set_up_motor(int enable_pin, int in1, int in2, int power) {
  pinMode(enable_pin, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  digitalWrite(enable_pin, HIGH);
  analogWrite(enable_pin, power);
}

void set_up_imu() {
  imu.setup();
  imu.setBias();
}

void setup() {
  Serial.begin(9600);

  // Set up the motor driver
  set_up_motor(ENA, IN2, IN3, TURN_MOTOR_POWER);
  set_up_motor(ENB, IN4, IN4, NORMAL_POWER);
}

void loop() {
  imu.updateBias();
  if (turns == 12) {
    delay(STOP_DELAY);
    stop_motor();
  } else {
    if (turns == 8) {
      if (read_serial() == RIGHT) {
        // getting RIGHT means we've detected
        // a red cube
        turn_opposite();
        return;
      }
    }

    unsigned long left_sonar = left_sensor.ping_cm();
    unsigned long right_sonar = right_sensor.ping_cm();
    unsigned long front_sonar = front_sensor.ping_cm();
    unsigned long back_sonar = back_sensor.ping_cm();
    char object_detection = read_serial();

    // Turning
    if (front_sonar < MIN_DIST_THRES) {
      set_power(TURN_POWER);
      if (left_sonar < MIN_DIST_THRES || left_sonar < right_sonar) {
        turn_right_with_angle(TURN_ANGLE);
      } else {
        turn_left_with_angle(TURN_ANGLE);
      }

      turns += 1;
      set_power(NORMAL_POWER);
    }

    left_sonar = left_sensor.ping_cm();
    right_sonar = right_sensor.ping_cm();
    front_sonar = front_sensor.ping_cm();
    object_detection = read_serial();

    // lane changing
    if (object_detection == LEFT) {
      turn_left(TURN_DELAY_OBSTACLE);
    } else if (object_detection == RIGHT) {
      turn_right(TURN_DELAY_OBSTACLE);
    }

    left_sonar = left_sensor.ping_cm();
    right_sonar = right_sensor.ping_cm();

    // center alignment
    while (abs(left_sonar - right_sonar) > LR_ERROR_MARGIN) {
      left_sonar = left_sensor.ping_cm();
      right_sonar = right_sensor.ping_cm();
      unsigned long diff = left_sonar - right_sonar;

      if (diff < 0) {
        turn_right(ALIGNMENT_DELAY);
      } else if (diff > 1) {
        turn_left(ALIGNMENT_DELAY);
      }
    }
  }
}

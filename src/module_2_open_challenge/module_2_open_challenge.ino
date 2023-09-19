#include <NewPing.h>
#include <MPU6050_light.h>

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
#define MAX_DIST             300 // cm
#define TURN_ANGLE            45 // deg
#define TURN_DELAY_OBSTACLE  100 // ms
#define ALIGNMENT_DELAY       50 // ms
#define LR_ERROR_MARGIN        5 // cm
#define TURN_POWER           255
#define NORMAL_POWER         255
#define TURN_MOTOR_POWER     255
#define STOP_DELAY           200 // ms

#define PRINT_DISTANCE(IDENT, SENSOR) \
  Serial.print(#IDENT); \
  Serial.print(": "); \
  Serial.print(SENSOR.ping_cm()); \
  Serial.println("cm"); \

MPU6050 imu(Wire);
int turns = 0;
NewPing front_sensor(FRONT_T, FRONT_E, MAX_DIST);
NewPing right_sensor(RIGHT_T, RIGHT_E, MAX_DIST);
NewPing left_sensor(LEFT_T, LEFT_E, MAX_DIST);
NewPing back_sensor(BACK_T, BACK_E, MAX_DIST);

void set_power(int power) {
  analogWrite(ENB, power);
}

void go_forward() {
  digitalWrite(IN4, LOW);
  digitalWrite(IN5, HIGH);
}

void go_backward() {
  digitalWrite(IN4, HIGH);
  digitalWrite(IN5, LOW);
}

void stop_motor() {
  digitalWrite(IN4, LOW);
  digitalWrite(IN5, LOW);
}

void turn_left() {
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
}

void turn_right() {
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
}

void stop_turn() {
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
}

void turn_with_angle(float angle, void(*turn_function)()) {
//  set_power(TURN_POWER);
  float last_yaw = read_yaw();

  turn_function();
  while (abs(read_yaw() - last_yaw) < angle) {
    unsigned long left_sonar = left_sensor.ping_cm();
    unsigned long right_sonar = right_sensor.ping_cm();

    // Turn left or right depending on the distance from left/right
    // until the vehicle is far enough from them(if applicable).
    // If the vehicle is about to hit the front wall during this procedure,
    // it goes backwards until it's close enough to wall from backside.
    // After that, it keeps going forward in the direction it's supposed to
    // go towards.
    if (left_sonar < MIN_DIST_THRES) {
      turn_right();
      while (left_sensor.ping_cm() < MIN_DIST_THRES) {
        if (front_sensor.ping_cm() < MIN_DIST_THRES) {
          go_backward();
          turn_left();
          while (back_sensor.ping_cm() > MIN_DIST_THRES) {}
          go_forward();
          turn_right();
        }
      }
      turn_function();
    } else if (right_sonar < MIN_DIST_THRES) {
      turn_left();
      while (right_sensor.ping_cm() < MIN_DIST_THRES) {
        if (front_sensor.ping_cm() < MIN_DIST_THRES) {
          go_backward();
          turn_right();
          while (back_sensor.ping_cm() > MIN_DIST_THRES) {}
          go_forward();
          turn_left();
        }
      }
      turn_function();
    }
  }
  stop_turn();
//  set_power(NORMAL_POWER);
}

float read_yaw() {
  imu.update();
  return abs(imu.getAngleZ());
}

void set_up_motor(int enable_pin, int in1, int in2, int power) {
  pinMode(enable_pin, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  digitalWrite(enable_pin, HIGH);
  analogWrite(enable_pin, power);
}

void set_up_imu() {
  Wire.begin();
  byte status = imu.begin();
  while(status != 0) {}
  imu.calcOffsets();
}

void setup() {
  Serial.begin(9600);
  Serial.println("konichiwaaaa >w<");

  // Set up the IMU
  set_up_imu();

  // Set up the motor driver
  set_up_motor(ENA, IN2, IN3, TURN_MOTOR_POWER);
  set_up_motor(ENB, IN4, IN5, NORMAL_POWER);
  go_forward();
}

void loop() {
  PRINT_DISTANCE(FRONT, front_sensor);
  PRINT_DISTANCE(LEFT, left_sensor);
  PRINT_DISTANCE(RIGHT, right_sensor);
  PRINT_DISTANCE(BACK, back_sensor);

  imu.update();
  Serial.println(read_yaw());
  if (false) {
    delay(STOP_DELAY);
    stop_motor();
    Serial.println("3 Laps done");
    while(1) {}
  } else {
    unsigned long left_sonar = left_sensor.ping_cm();
    unsigned long right_sonar = right_sensor.ping_cm();
    unsigned long front_sonar = front_sensor.ping_cm();
    unsigned long back_sonar = back_sensor.ping_cm();

    // Turning
    if (front_sonar < MIN_DIST_THRES) {
      Serial.print("Turning ");
      set_power(TURN_POWER);
      if (left_sonar < right_sonar) {
        Serial.println("right");
        turn_with_angle(TURN_ANGLE, turn_right);
      } else {
        Serial.println("left");
        turn_with_angle(TURN_ANGLE, turn_left);
      }

      turns += 1;
      set_power(NORMAL_POWER);
    }

    left_sonar = left_sensor.ping_cm();
    right_sonar = right_sensor.ping_cm();

    // center alignment
    if (abs(left_sonar - right_sonar) > LR_ERROR_MARGIN) {
      left_sonar = left_sensor.ping_cm();
      right_sonar = right_sensor.ping_cm();

      if (left_sonar < right_sonar) {
        turn_right();
        while (left_sensor.ping_cm() < right_sensor.ping_cm()) {
          if (front_sensor.ping_cm() <= MIN_DIST_THRES) {
            break;
          }
        }
      } else if (left_sonar > right_sonar) {
        turn_left();
        while (left_sensor.ping_cm() > right_sensor.ping_cm()) {
          if (front_sensor.ping_cm() <= MIN_DIST_THRES) {
            break;
          }
        }
      }
      stop_turn();
    }
  }
}

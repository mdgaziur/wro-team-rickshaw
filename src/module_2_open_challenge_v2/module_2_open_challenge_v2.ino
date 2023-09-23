#include <NewPing.h>

// Ultrasonic sensors
#define FRONT_T 13              // Trigger pin of the front ultrasonic sensor
#define FRONT_E 12              // Echo pin of the front ultrasonic sensor
#define RIGHT_T 11              // Trigger pin of the right ultrasonic sensor
#define RIGHT_E 10              // Echo pin of the right ultrasonic sensor
#define LEFT_T   9              // Trigger pin of the left ultrasonic sensor
#define LEFT_E   8              // Echo pin of the left ultrasonic sensor

// Motor driver pins
#define ENA      5              // Enable pin for the steering motor
#define IN2      4              // Pin1 of the steering motor
#define IN3      3              // Pin2 of the steering motor
#define ENB     23              // Enable pin for the driving motor
#define IN4      2              // Pin1 of the driving motor
#define IN5     22              // Pin2 of the driving motor

// Constants
#define NORMAL_DRIVE_POWER  255 // Power for normal driving operations
#define TURNING_DRIVE_POWER 255 // Power for driving motor when the vehicle is turning
#define STEERING_POWER      255 // Power for the steering motor
#define MIN_DIST_FROM_WALL   60 // Minimum allowed distance from wall(in cm)
#define TURN_DURATION_BASE  200 // Base value used as the duration for which the vehicle will turn.
                                // Will be multiplied by "turn duration factor" when turning.
#define MAX_DIST            300 // Maximum distance from any wall(in cm)
#define LR_ERROR_MARGIN       2 // Error margin while aligning the vehicle in the center among the right and left walls(in cm)
#define TURN_STEERING_STOP_DELAY 400 // The delay which is required for the vehicle to be in perfect position after turn(in ms)
#define CRASH_THRES          25 // Distance that's counted as crashing(in cm)


#define PRINT_DISTANCE(IDENT, SENSOR) \
  Serial.print(#IDENT); \
  Serial.print(": "); \
  Serial.print(SENSOR.ping_cm()); \
  Serial.println("cm"); \

int     turns = 0;              // Amount of turns the vehicle has completed
NewPing front_sensor(FRONT_T, FRONT_E, MAX_DIST);
NewPing right_sensor(RIGHT_T, RIGHT_E, MAX_DIST);
NewPing left_sensor(LEFT_T, LEFT_E, MAX_DIST);

void drive_forward() {
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
}

void drive_backward() {
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
}

void stop_driving() {
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
}

void steer_left() {
  digitalWrite(IN4, HIGH);
  digitalWrite(IN5, LOW);
}

void steer_right() {
  digitalWrite(IN4, LOW);
  digitalWrite(IN5, HIGH);
}

void stop_steering() {
  digitalWrite(IN4, LOW);
  digitalWrite(IN5, LOW);
}

void set_power(int power) {
  analogWrite(ENA, power);
}

int ping_cm(NewPing sensor) {
  int res = 0;
  int c = 5;
  while(c--) res = max(res, sensor.ping_cm());
  if (res == 0) res = 300;
  return res; 
}

// Sets up motor located at given pins and sets given power(0-255) to the motor
void set_up_motor(int en, int in1, int in2, int power) {
  pinMode(en, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  analogWrite(en, power);
}

void setup() {
  // Set baud rate as 9600 for Serial communication(mainly for debug purpose)
  Serial.begin(9600);
  Serial.println("٩(◕‿◕｡)۶ Konichiwaaaa");

  // Set up the steering motor and the driving motor
  set_up_motor(ENA, IN2, IN3, NORMAL_DRIVE_POWER);
  set_up_motor(ENB, IN4, IN5, STEERING_POWER);

  drive_forward();
}

void loop() {
  // Each lap consists of 4 turns(the track is rectangular). So 3 laps consists of 12 turns
//  if (turns == 12) {
//    Serial.println("(ﾉ◕ヮ◕)ﾉ*:･ﾟ✧ 3 laps done");
//    stop_driving();
//    while (1) {}
//  }
  PRINT_DISTANCE(Front, front_sensor);
  PRINT_DISTANCE(Right, right_sensor);
  PRINT_DISTANCE(Left, left_sensor);

  if (ping_cm(front_sensor) < CRASH_THRES) {
    drive_backward();
  } else if (ping_cm(front_sensor) > CRASH_THRES) {
    drive_forward();
  } else if (ping_cm(front_sensor) < MIN_DIST_FROM_WALL) {
    if (ping_cm(left_sensor) < ping_cm(right_sensor)) {
      steer_right();
    } else {
      steer_left();
    }
  } else if (abs(ping_cm(left_sensor) - ping_cm(right_sensor)) > LR_ERROR_MARGIN) {
    if (ping_cm(left_sensor) < ping_cm(right_sensor)) {
      steer_right();
    } else {
      steer_left();
    }
  } else {
    stop_steering();
  }
  
  delay(50);
}

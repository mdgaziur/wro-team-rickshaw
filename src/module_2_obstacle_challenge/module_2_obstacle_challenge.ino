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
#define TURNING_DRIVE_POWER 175 // Power for driving motor when the vehicle is turning
#define STEERING_POWER      255 // Power for -+the steering motor
#define MIN_DIST_FROM_WALL   30 // Minimum allowed distance from wall(in cm)
#define TURN_DURATION_BASE  200 // Base value used as the duration for which the vehicle will turn.
// Will be multiplied by "turn duration factor" when turning.
#define MAX_DIST            300 // Maximum distance from any wall(in cm)
#define LR_ERROR_MARGIN       2 // Error margin while aligning the vehicle in the center among the right and left walls

#define BLOCK_UNITL_POSSIBLE_COLLISION(expr) \
      while (expr && ping_cm(front_sensor) > LR_ERROR_MARGIN && get_object_detection_status() == 'N') {}

int     turns = 0;              // Amount of turns the vehicle has completed
NewPing front_sensor(FRONT_T, FRONT_E, MAX_DIST);
NewPing right_sensor(RIGHT_T, RIGHT_E, MAX_DIST);
NewPing left_sensor(LEFT_T, LEFT_E, MAX_DIST);

void drive_forward() {
  digitalWrite(IN4, LOW);
  digitalWrite(IN5, HIGH);
}

void drive_backward() {
  digitalWrite(IN4, HIGH);
  digitalWrite(IN5, LOW);
}

void stop_driving() {
  digitalWrite(IN4, LOW);
  digitalWrite(IN5, LOW);
}

void steer_left() {
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
}

void steer_right() {
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
}

void stop_steering() {
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
}

char get_object_detection_status() {
  if (!Serial.available()) return 'N';

  return (char)Serial.read();
}

// Sets up motor located at given pins and sets given power(0-255) to the motor
void set_up_motor(int en, int in1, int in2, int power) {
  Serial.println(en);
  Serial.println(in1);
  Serial.println(in2);
  Serial.println(power);
  pinMode(en, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  analogWrite(en, power);
}

int ping_cm(NewPing sensor) {
  int res = 0;
  int c = 5;
  while ( c-- ) res = max(res, sensor.ping_cm());
  if ( res == 0 ) res = 1000;
  return res;
}

void setup() {
  // Set baud rate as 9600 for Serial communication(mainly for debug purpose)
  Serial.begin(9600);
  Serial.println("٩(◕‿◕｡)۶ Konichiwaaaa");

  // Set up the steering motor and the driving motor
  set_up_motor(ENA, IN2, IN3, STEERING_POWER);
  set_up_motor(ENB, IN4, IN5, NORMAL_DRIVE_POWER);

  drive_forward();
}

void loop() {
  const char object_detection = get_object_detection_status();
  if (ping_cm(front_sensor) < MIN_DIST_FROM_WALL) {
    if (ping_cm(left_sensor) > ping_cm(right_sensor)) {
      steer_left();
    } else {
      steer_right();
    }

    while (ping_cm(front_sensor) < MIN_DIST_FROM_WALL) {}
    stop_steering();
  } else if (object_detection == 'L' && ping_cm(left_sensor) > MIN_DIST_FROM_WALL) {
    steer_left();
    while (ping_cm(left_sensor) > MIN_DIST_FROM_WALL) {
      if (ping_cm(front_sensor) < MIN_DIST_FROM_WALL) {
        break;
      }
    }
    steer_right();
    delay(200);
    stop_steering();
  } else if (object_detection == 'R' && ping_cm(right_sensor) > MIN_DIST_FROM_WALL) {
    steer_right();
    while (ping_cm(right_sensor) > MIN_DIST_FROM_WALL) {
      if (ping_cm(front_sensor) < MIN_DIST_FROM_WALL) {
        break;
      }
    }
    steer_left();
    delay(200);
    stop_steering();
  } else if (ping_cm(left_sensor) < MIN_DIST_FROM_WALL) {
    steer_right();
    BLOCK_UNITL_POSSIBLE_COLLISION(ping_cm(left_sensor) < MIN_DIST_FROM_WALL);
    stop_steering();
  } else if (ping_cm(right_sensor) < MIN_DIST_FROM_WALL) {
    steer_right();
    BLOCK_UNITL_POSSIBLE_COLLISION(ping_cm(left_sensor) < MIN_DIST_FROM_WALL);
    stop_steering();
  } else if (abs(ping_cm(left_sensor) - ping_cm(right_sensor)) > LR_ERROR_MARGIN) {
    if (ping_cm(left_sensor) > ping_cm(right_sensor)) {
      steer_left();
    } else {
      steer_right();
    }

    BLOCK_UNITL_POSSIBLE_COLLISION(abs(ping_cm(left_sensor) - ping_cm(right_sensor)) > LR_ERROR_MARGIN);
    stop_steering();
  }
}

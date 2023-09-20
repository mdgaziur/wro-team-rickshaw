#include <NewPing.h>
#include <MPU6050_light.h>

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
#define NORMAL_DRIVE_POWER  200 // Power for normal driving operations
#define TURNING_DRIVE_POWER 175 // Power for driving motor when the vehicle is turning
#define STEERING_POWER      255 // Power for the steering motor
#define MIN_DIST_FROM_WALL   50 // Minimum allowed distance from wall(in cm)
#define TURN_DURATION_BASE  200 // Base value used as the duration for which the vehicle will turn.
                                // Will be multiplied by "turn duration factor" when turning.
#define MAX_DIST            300 // Maximum distance from any wall(in cm)
#define LR_ERROR_MARGIN       2 // Error margin while aligning the vehicle in the center among the right and left walls

int     turns = 0;              // Amount of turns the vehicle has completed
MPU6050 mpu(Wire);
NewPing front_sensor(FRONT_T, FRONT_E, MAX_DIST);
NewPing right_sensor(RIGHT_T, RIGHT_E, MAX_DIST);
NewPing left_sensor(LEFT_T, LEFT_E, MAX_DIST);

void drive_forward() {
  pinMode(IN4, LOW);
  pinMode(IN5, HIGH);
}

void stop_driving() {
  pinMode(IN4, LOW);
  pinMode(IN5, LOW);
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

void get_yaw() {
  mpu.update();

  return mpu.getAngleX();
}

char get_object_detection_status() {
  if (!Serial.available()) return 'N';
  
  return (char)Serial.read();
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

  // Set up the MPU(Motion Processing Unit)
  Wire.begin();
  byte status = mpu.begin();
  Serial.println("(∩ ⌣̀_⌣́) MPU6050 status: ");
  while(status != 0) {}
  mpu.calcOffsets();

  // Set up the steering motor and the driving motor
  set_up_motor(ENA, IN2, IN3, STEERING_POWER);
  set_up_motor(ENB, IN4, IN5, NORMAL_DRIVE_POWER);

  drive_forward();
}

void loop() {
  // Each lap consists of 4 turns(the track is rectangular). So 3 laps consists of 12 turns
  if (turns == 12) {
    Serial.println("(ﾉ◕ヮ◕)ﾉ*:･ﾟ✧ 3 laps done");
    stop_driving();
    while (1) {}
  }

  char object_detection = get_object_detection_status();

  if (front_sensor.ping_cm() < MIN_DIST_FROM_WALL) {
    float dst_from_left_wall = left_sensor.ping_cm();
    float dst_from_right_wall = right_sensor.ping_cm();
    float turn_delay_factor = dst_from_left_wall / dst_from_right_wall;

    // The reason why we're increasing "turns" variable inside the if-else statements is due to
    // the fact that sometimes this branch may get executed when the vehicle is somehow closer
    // to the front wall than it should be but still not in a valid position to make a turn.
    if (dst_from_left_wall > dst_from_right_wall) {
      steer_left();
      delay(TURN_DURATION_BASE * turn_delay_factor);
      stop_steering();
      turns += 1;
    } else if (dst_from_left_wall < dst_from_right_wall) {
      steer_right();
      delay(TURN_DURATION_BASE * (1 / turn_delay_factor)); // We're inversing the delay factor because we always want it to be greater than 1.
      stop_steering();
      turns += 1;
    }
  } else if (object_detection != 'N') {
    if (object_detection == 'L') {
      steer_left();
    } else if (object_detection == 'R') {
      steer_right();
    }

    // Keep turning until we've crossed the object
    while (object_detection != 'N') {
      // We're gonna hit the front wall. PRIORITIZE TURNING!
      if (front_sensor.ping_cm() < MIN_DIST_FROM_WALL) {
        break;  
      }
    }
    stop_steering();
  }
}

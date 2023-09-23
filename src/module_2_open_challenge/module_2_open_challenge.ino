#include <NewPing.h>

// Ultrasonic sensors
#define FRONT_T 13              // Trigger pin of the front ultrasonic sensor
#define FRONT_E 12              // Echo pin of the front ultrasonic sensor
#define RIGHT_T 11              // Trigger pin of the right ultrasonic sensor
#define RIGHT_E 10              // Echo pin of the right ultrasonic sensor
#define LEFT_T   9              // Trigger pin of the left ultrasonic sensor
#define LEFT_E   8              // Echo pin of the left ultrasonic sensor


// Motor driver pins
#define ENA     23              // Enable pin for the driving motor
#define IN2     22              // Pin1 of the driving motor
#define IN3      2              // Pin2 of the driving motor
#define ENB      5              // Enable pin for the steering motor
#define IN4      4              // Pin1 of the steering motor
#define IN5      3              // Pin2 of the steering motor


// Constants
#define NORMAL_DRIVE_POWER  200 // Power for normal driving operations
#define TURNING_DRIVE_POWER 200 // Power for driving motor when the vehicle is turning
#define STEERING_POWER      255 // Power for the steering motor
#define MIN_DIST_FROM_WALL   60 // Minimum allowed distance from wall(in cm)
#define TURN_DURATION_BASE  200 // Base value used as the duration for which the vehicle will turn.
                                // Will be multiplied by "turn duration factor" when turning.
#define MAX_DIST            1000 // Maximum distance from any wall(in cm)
#define LR_ERROR_MARGIN       3 // Error margin while aligning the vehicle in the center among the right and left walls(in cm)
#define TURN_STEERING_STOP_DELAY 700 // The delay which is required for the vehicle to be in perfect position after turn(in ms)
#define CRASH_THRESHOLD           20
#define CRASH_THRESHOLD2           10
#define TURN_THRESHOLD            60

NewPing left_sensor(LEFT_T, LEFT_E, MAX_DIST);
NewPing front_sensor(FRONT_T, FRONT_E, MAX_DIST);
NewPing right_sensor(RIGHT_T, RIGHT_E, MAX_DIST);

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
  Serial.println("GOING LEFT");
  digitalWrite(IN4, HIGH);
  digitalWrite(IN5, LOW);
}

void steer_right() {
 
  Serial.println("GOING RIGHT");
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

// Sets up motor located at given pins and sets given power(0-255) to the motor
void set_up_motor(int en, int in1, int in2, int power) {
  pinMode(en, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  analogWrite(en, power);
}

int ping_cm(NewPing sensor) {
  int res = 0;
  int c = 5;
  while( c-- ) res = max(res, sensor.ping_cm());
  if( res == 0 ) res = 1000;
  return res;
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  set_up_motor(ENA, IN2, IN3, NORMAL_DRIVE_POWER);
  set_up_motor(ENB, IN4, IN5, STEERING_POWER);
  
}

void loop() {
  char bias = 'R';
  bool back_flag = 0;
  const int back_flag_thres = 10;
  while(1) {
    int f_dist = ping_cm(front_sensor);
    int l_dist = ping_cm(left_sensor);
    int r_dist = ping_cm(right_sensor);
    
    Serial.println(f_dist);
    Serial.println(l_dist);
    Serial.println(r_dist);
    Serial.println("-------");
    Serial.println("-------");
    
    if( l_dist < 8 || r_dist < 8 ) {
      stop_steering();
      drive_backward();
      delay(100);  
      continue;
    }
    
    if( f_dist < CRASH_THRESHOLD ) {
      if( r_dist < CRASH_THRESHOLD2 ) steer_right();
      else if( l_dist < CRASH_THRESHOLD2 ) steer_left();
      else stop_steering();
      drive_backward();
      delay(150);
      continue;
    }
    
    if( abs(l_dist - r_dist) > 0 ) {
      if( l_dist > r_dist ) steer_left();
      else steer_right(); 
      drive_forward();
      continue;
    }
    
    drive_forward();
    stop_steering();
  }
}

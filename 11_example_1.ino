#include <Servo.h>
#define _EMA_ALPHA 0.5

// Arduino pin assignment
#define PIN_LED   9   // LED active-low
#define PIN_TRIG  12  // sonar sensor TRIGGER
#define PIN_ECHO  13  // sonar sensor ECHO
#define PIN_SERVO 10  // servo motor

// Configurable parameters for sonar
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 180.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 360.0   // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // coefficient to convert duration to distance

// Duty duration for myservo.writeMicroseconds()
// NEEDS TUNING (servo by servo)
#define _DUTY_MIN 300 // servo full clockwise position (0 degree) 
#define _DUTY_MAX 3000 // servo full counterclockwise position (180 degree)

// Global variables
float dist_ema = 0, dist_prev = _DIST_MAX; // unit: mm
unsigned long last_sampling_time = 0; // unit: ms

Servo myservo;

void setup() {
  // Initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);    // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);     // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar 

  myservo.attach(PIN_SERVO); 
  myservo.write(0); // Initialize servo to 0°

  // Initialize USS related variables
  dist_prev = _DIST_MIN; // raw distance output from USS (unit: mm)

  // Initialize serial port
  Serial.begin(57600);
}

float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
}

void loop() {
  float dist_raw;

  // Wait until next sampling time. 
  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO); // Read distance

  // Handle invalid distance readings
  if ((dist_raw == 0.0) || (dist_raw > _DIST_MAX)) {
    dist_raw = dist_prev;   
    dist_raw = 300; // Cut higher than maximum
    digitalWrite(PIN_LED, HIGH); // LED OFF
  } else if (dist_raw < _DIST_MIN) {
    dist_raw = dist_prev;  
    dist_raw = 0; // Cut lower than minimum
    digitalWrite(PIN_LED, HIGH); // LED OFF
  } else { // In desired Range
    dist_prev = dist_raw;
    digitalWrite(PIN_LED, LOW); // LED ON      
  }

  // Apply EMA filter here  
  dist_ema = dist_raw * _EMA_ALPHA + dist_prev * (1 - _EMA_ALPHA);
  dist_prev = dist_ema;

  // Control servo position based on distance
  int servo_angle;
  if (dist_ema < _DIST_MIN) {
    servo_angle = _DUTY_MIN; // 0°
  } else if (dist_ema < _DIST_MAX) {
    servo_angle = map(dist_ema, _DIST_MIN, _DIST_MAX, 0, 180); // 0° to 180°
  } else {
    servo_angle = _DUTY_MAX; // 180°
  }

  myservo.write(servo_angle); // Set servo to calculated angle
  

  // Output the distance and EMA to the serial port
  Serial.print("Min: "); Serial.print(_DIST_MIN);
  Serial.print(", dist: "); Serial.print(dist_raw);
  Serial.print(", ema: "); Serial.print(dist_ema);
  Serial.print(", Servo: "); Serial.println(servo_angle);
}

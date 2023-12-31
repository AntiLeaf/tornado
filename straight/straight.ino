#include <Servo.h>

const int A_PWM = 6, A_DIR = 7;  
const int B_PWM = 5, B_DIR = 4;
const int STEER = 9;
Servo steer;

void setup() {
  pinMode(A_DIR, OUTPUT);
  pinMode(A_PWM, OUTPUT);
  pinMode(B_DIR, OUTPUT);
  pinMode(B_PWM, OUTPUT);

  steer.attach(STEER);

	Serial.begin(9600);
	while (!Serial);

  digitalWrite(A_DIR, LOW);
  digitalWrite(B_DIR, LOW);

  analogWrite(A_PWM, 350);
  analogWrite(B_PWM, 350);

  steer.write(78);
}

void loop() {
}

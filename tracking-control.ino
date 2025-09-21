#include <Servo.h>

String inputStringServo;
Servo horizontal;
Servo armServo;

int greenLED = 5;
int redLED = 3;

void setup() {
  // pins for each servo
  horizontal.attach(8);
  armServo.attach(10);

  Serial.begin(9600);

  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);

  digitalWrite(greenLED, LOW);
  digitalWrite(redLED, LOW);
  
  Serial.println("LED control ready");
}

void loop() {
  if(Serial.available()){
    inputStringServo = Serial.readStringUntil('\r');
    inputStringServo.trim();
    Serial.println(inputStringServo);

    if (inputStringServo.indexOf(',') > 0) {
      int x_axis = inputStringServo.substring(0, inputStringServo.indexOf(',')).toInt();

      int x = map(x_axis, 0, 2560, 180, 0); // 1920

      horizontal.write(x);
    }
    else if (inputStringServo.startsWith("ANGLE:")) { // writes to arm Servo with degree
      int angleValue = inputStringServo.substring(6).toInt();
      angleValue = constrain(angleValue, 0, 180);
      armServo.write(angleValue);
    }
    else {
      processInput(inputStringServo);
    }
  }

}

// Function to process serial command
void processInput(String cmd) {
  cmd.trim();  // remove spaces/newlines

  if (cmd == "CORRECT") {
    digitalWrite(greenLED, HIGH);
    digitalWrite(redLED, LOW);
    Serial.println("Green LED ON - Correct form");
  }
  else if (cmd == "INCORRECT") {
    digitalWrite(greenLED, LOW);
    digitalWrite(redLED, HIGH);
    Serial.println("Red LED ON - Incorrect form");
  }
  else if (cmd == "OFF") {
    digitalWrite(greenLED, LOW);
    digitalWrite(redLED, LOW);
    Serial.println("All LEDs OFF");
  }
}
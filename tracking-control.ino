#include <Servo.h>

String inputStringServo;
Servo horizontal;
Servo vertical;

int greenLED = 5;
int redLED = 3;

void setup() {
  // attaches horizontal and verticle servos to correct pins on breadboard
  horizontal.attach(10);
  vertical.attach(8);

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
      int y_axis = inputStringServo.substring(inputStringServo.indexOf(',') + 1).toInt();

      int y = map(y_axis, 0, 1600, 180, 0); // 1080
      int x = map(x_axis, 0, 2560, 180, 0); // 1920

      horizontal.write(x);
      vertical.write(y);
      
      // Print the parsed values
      Serial.print("First Integer: ");
      Serial.println(x);
      Serial.print("Second Integer: ");
      Serial.println(y);
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
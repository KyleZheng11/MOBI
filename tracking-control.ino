#include <Servo.h>

// Servo servo1;
// Servo servo2;

int greenLED = 8;
int redLED = 11;

String inputString = "";   // buffer for input
boolean stringComplete = false;

void setup() {
  Serial.begin(9600);
  pinMode(greenLED, OUTPUT)
  pinMode(redLED, OUTPUT)

  digitalWrite(greenLED, LOW);
  digitalWrite(redLED, LOW);
  
  Serial.println("LED control ready");

  // delay(1000);
  // servo1.attach(8);   // Servo 1 on pin 8
  // servo2.attach(11);  // Servo 2 on pin 11
 
  // Serial.println("Enter positions in format: S1:90 or S2:45");


  // Serial.println("Orig. Set!!!");
}

void loop() {
  // Check if a command is ready
  if (stringComplete) {
    processInput(inputString);
    inputString = "";
    stringComplete = false;
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

  // if (cmd.startsWith("S1:")) {
  //   int pos = cmd.substring(3).toInt();
  //   servo1.write(90);
  //   pos = constrain(pos, 0, 180); // keep within servo limits
  //   servo1.write(pos);
  //   Serial.print("Servo 1 moved to: ");
  //   Serial.println(pos);

  // } else if (cmd.startsWith("S2:")) {
  //   int pos = cmd.substring(3).toInt();
  //   servo2.write(180);
  //   pos = constrain(pos, 0, 360);
  //   servo2.write(pos);
  //   Serial.print("Servo 2 moved to: ");
  //   Serial.println(pos);

  // } else {
  //   Serial.println("Invalid command. Use S1:angle or S2:angle");
  // }
}

// Serial event handler
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}









//Bad code
// #include <Servo.h>

// Servo Servo1;
// Servo Servo2;

// // Store incoming serial data
// String inputString = "";  
// boolean stringComplete = false;

// void setup(){
//   Serial.begin(9600);   // start serial communication // baud rate = 9600
//   Servo1.attach(8);     // attach first servo to pin 8
//   Servo2.attach(11);    // attach second servo to pin 11
//   Serial.println("Enter angles as: angle1,angle2");
// }

// void loop(){
//   //check if line was received
//   if (stringComplete){
//     stringComplete = false;
  
//    // remove newline characters
//     inputString.trim();

//     // find the comma separator
//     int commaIndex = inputString.indexOf(',');
//     if (commaIndex > 0) {

//       //Split the string into two portions
//       String angle1Str = inputString.substring(0, commaIndex);
//       String angle2Str = inputString.substring(commaIndex + 1);

//       // convert to integers
//       int angle1 = angle1Str.toInt();
//       int angle2 = angle2Str.toInt();

//       //setting angle limitationsd
//       angle1 = constrain(angle1, 0, 180);
//       angle2 = constrain(angle2, 0, 360);

//       //initializing servo motors
//       Servo1.write(angle1);
//       Servo2.write(angle2);
//     }
//     //clear buffer
//     inputString = "";
//   }
// }


//Simulatenous control
// #include <Servo.h>

// Servo servo1;
// Servo servo2;

// int servo1Pin = 8;
// int servo2Pin = 11;

// String inputString = "";  // store serial input
// bool inputComplete = false;

// void setup() {
//   Serial.begin(9600);          // start serial communication
//   servo1.attach(servo1Pin);
//   servo2.attach(servo2Pin);

//   Serial.println("Enter angle (0 - 180):");
// }

// void loop() {
//   // check if input was received
//   if (inputComplete) {
//     int angle = inputString.toInt();   // convert string to integer

//     if (angle >= 0 && angle <= 180) {
//       servo1.write(angle);             // Servo 1 moves to input angle
//       servo2.write(180 - angle);       // Servo 2 mirrors the angle
//       Serial.print("Servo1: ");
//       Serial.print(angle);
//       Serial.print(" | Servo2: ");
//       Serial.println(180 - angle);
//     } else {
//       Serial.println("Invalid angle. Enter 0 - 180 only.");
//     }

//     // clear input
//     inputString = "";
//     inputComplete = false;
//     Serial.println("Enter angle (0 - 180):");
//   }
// }

// // This runs whenever new serial data comes in
// void serialEvent() {
//   while (Serial.available()) {
//     char inChar = (char)Serial.read();
//     if (inChar == '\n') {       // end of line = input complete
//       inputComplete = true;
//     } else {
//       inputString += inChar;    // build string
//     }
//   }
// }
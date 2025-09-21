# to run type   "py -3.12 .\arm-detection.py"

import cv2
import mediapipe as mp
import pickle
import numpy as np
from skimage.transform import resize
import glob
import os
import serial
import time
import serial
import serial.tools.list_ports
import math

def send_coordinates_to_arduino(x, y):
    # Convert the coordinates to a string and send it to Arduino
    coordinates = f"{x},{y}\r"
    arduino.write(coordinates.encode())
    print(f"X{x}Y{y}\n")

tracking_counter = 0
tracking_interval = 10  # Every 15 frames (about 2 times per second)

webcam = cv2.VideoCapture(0) 

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()

last_tracking_time = 0
tracking_delay = 1.0  # 1 second delay

correct_image_count = 0
# correct_folder = '/Users/k1105/MOBI/MOBI_PhysicalTherapyAssistant/arm-flexion-library/correct-arm-flexion-photos/'
correct_folder = './arm-flexion-library/correct-arm-flexion-photos/'
existing_correct = len(glob.glob(os.path.join(correct_folder, "*.jpg")))
correct_image_count = existing_correct

incorrect_image_count = 0
# incorrect_folder = '/Users/k1105/MOBI/MOBI_PhysicalTherapyAssistant/arm-flexion-library/incorrect-arm-flexion-photos/'
incorrect_folder = './arm-flexion-library/incorrect-arm-flexion-photos/'
existing_incorrect = len(glob.glob(os.path.join(incorrect_folder, "*.jpg")))
incorrect_image_count = existing_incorrect

# model = pickle.load(open('/Users/k1105/MOBI/MOBI_PhysicalTherapyAssistant/arm-model.p', 'rb'))
model = pickle.load(open('./arm-model.p', 'rb'))

print("Available COM ports:")
ports = serial.tools.list_ports.comports()
for port in ports:
    print(f" {port.device} - {port.description}")

try:
    print("Attempting to connect to COM6...")
    arduino = serial.Serial('COM6', 9600, timeout=1)  # Windows
    time.sleep(3)  # Wait for Arduino to initialize
    print("Arduino connected")
except serial.SerialException as e:
    arduino = None
    print(f"Serial connection error: {e}")
except Exception as e:
    arduino = None
    print(f"Other connection error: {e}")

while True:
    ret, frame = webcam.read()
    
    if ret:
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = pose.process(rgb_frame)
        if result.pose_landmarks:
            height, width, _ = frame.shape

            #mediaPipe landmarks for right shoulder, elbow, and wrist
            shoulder = result.pose_landmarks.landmark[12]
            elbow = result.pose_landmarks.landmark[14]
            wrist = result.pose_landmarks.landmark[16]

            #pixel coordinates
            shoulder_x = int(shoulder.x * width)
            shoulder_y = int(shoulder.y * height)
            elbow_x = int(elbow.x * width)
            elbow_y = int(elbow.y * height)
            wrist_x = int(wrist.x * width)
            wrist_y = int(wrist.y * height)

            # Vector from elbow to wrist
            v1 = np.array([wrist_x - elbow_x, wrist_y - elbow_y])
            # Vector from elbow to shoulder
            v2 = np.array([shoulder_x - elbow_x, shoulder_y - elbow_y])
            cos_theta = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
            elbow_angle = math.degrees(np.arccos(cos_theta))
            
            #circles at wrist, elbow, and shoulder marks
            cv2.circle(frame, (shoulder_x, shoulder_y), 5, (180, 180, 0), -2)

            text_position = (elbow_x, elbow_y - 10)  # 10 pixels above the elbow
            cv2.putText(frame, f"{int(elbow_angle)}deg", text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)          

            cv2.circle(frame, (elbow_x, elbow_y), 5, (180, 180, 0), -2)
            cv2.circle(frame, (wrist_x, wrist_y), 5, (180, 180, 0), -2)

            #lines from wrist-elbow and elbow-shoulder
            cv2.line(frame,(wrist_x, wrist_y), (elbow_x, elbow_y), (255, 255, 255), 4)
            cv2.line(frame, (elbow_x, elbow_y), (shoulder_x, shoulder_y), (255, 255, 255), 4)        

            # Create bounding rectangle around all three points
            min_x = min(shoulder_x, elbow_x, wrist_x) - 50  # Add padding
            max_x = max(shoulder_x, elbow_x, wrist_x) + 50
            min_y = min(shoulder_y, elbow_y, wrist_y) - 50
            max_y = max(shoulder_y, elbow_y, wrist_y) + 50

            # Add boundary checking
            min_x = max(0, min_x)
            min_y = max(0, min_y)
            max_x = min(width, max_x)
            max_y = min(height, max_y)

            if min_x < max_x and min_y < max_y:
                cropped_image = frame[min_y:max_y, min_x:max_x]  
                
                if cropped_image.size > 0:
                    # Draw rectangle around the arm. uses model to switch rectangel to gree/blue whether its correct/incorrect
                    rgb_cropped_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2RGB)
                    rgb_cropped_image = resize(rgb_cropped_image, (64, 64))
                    rgb_cropped_image = rgb_cropped_image.flatten()
          
                    prediction_input = rgb_cropped_image.reshape(1, -1)
                    prediction = model.predict(prediction_input) #prediction[0] = 0 means incorrect     prediction[0] = 1 means correct

                    if prediction[0] == 0:
                        print("Prediction: Incorrect")
                        rectangle_color = (0, 0, 255)
                        if arduino:
                            arduino.write(b"INCORRECT\r")  # Turn on red LED
                            arduino.flush()
                    else:
                        print("Prediction: Correct")
                        rectangle_color = (0, 255, 0)
                        if arduino:
                            arduino.write(b"CORRECT\r")  # Turn on green LED
                            arduino.flush()
                else:
                    rectangle_color = (255, 255, 255)
            else:
                rectangle_color = (255, 255, 255)

            #sending coordinaftes of average of rectangle coordinates to arduino
            if arduino:
                tracking_counter += 1
                if tracking_counter >= tracking_interval:
                        # sends degrees to arduino
                    elbow_angle_msg = f"ANGLE:{int(elbow_angle)}\r" 
                    arduino.write(elbow_angle_msg.encode())
                        # send_coordinates_to_arduino( ((min_x + max_x) / 2), ((min_y+max_y) / 2) )
                    send_coordinates_to_arduino((shoulder_x + elbow_x + wrist_x) / 3, (shoulder_y + elbow_y + wrist_y) / 3)
                    tracking_counter = 0

            cv2.rectangle(frame, (min_x, min_y), (max_x, max_y), rectangle_color, 2)       


        cv2.imshow("frame", frame) #im = image image show
        key = cv2.waitKey(1) # waits at most 1 millisecond for user to press a key on keyboard

        #if press c, saves frame into correct arm 
        # if key == ord("c"):
        #     if cropped_image.size > 0:
        #         cv2.imwrite('/Users/k1105/MOBI/MOBI_PhysicalTherapyAssistant/arm-flexion-library/correct-arm-flexion-photos/'f'correct_{correct_image_count:03d}.jpg', cropped_image)
        #         correct_image_count += 1
        # if key ==ord("z"):
        #     if cropped_image.size > 0:
        #         cv2.imwrite('/Users/k1105/MOBI/MOBI_PhysicalTherapyAssistant/arm-flexion-library/incorrect-arm-flexion-photos/'f'incorrect_{incorrect_image_count:03d}.jpg', cropped_image)
        #         incorrect_image_count += 1
        # #if press z, saves frame into incorrect arm flexion
        if key == ord("q"):
            arduino.write(b"OFF\r")  
            arduino.flush()
            break
    
webcam.release()
cv2.destroyAllWindows()
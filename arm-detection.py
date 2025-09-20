import cv2
import mediapipe as mp
import pickle
import numpy as np
from skimage.transform import resize
from PIL import Image

# model = pickle.load(open('./arm-model.p', 'rb'))

webcam = cv2.VideoCapture(0) 

# mp_drawing = mp.solutions.drawing_utils
# mp_hands = mp.solutions.hands
# hand = mp_hands.Hands()
# # might have to switch to arms

while True:
    ret, frame = webcam.read()

    if ret:
        RGB_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = hand.process(RGB_frame)
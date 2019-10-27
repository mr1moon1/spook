"""
Stems from help received by a StackOverflow page:
https://stackoverflow.com/questions/32353033/eye-blink-detection
"""
# ~~IMPORTS~~
#webcam imports:
import numpy as np
import dlib
from math import hypot
import time
import cv2
from scipy.spatial import distance as dist #for euclidean distance
#robot imports:
import pyfirmata

#===================================================================
# ~~INTRO INITS~~
#get capture:
cap = cv2.VideoCapture(0)
#other stuff:
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")
font = cv2.FONT_HERSHEY_SIMPLEX

#robot inits:
LED_PIN = 13
#board stuff:
port = "COM3"
board = pyfirmata.Arduino(port)
#servo stuff:
neck = board.get_pin('d:8:s')
jaw = board.get_pin('d:7:s')

#===================================================================
# ~~FUNCTIONS~~
def midpoint(p1,p2): #used for eye stuff
    return int((p1.x + p2.x)/2),int((p1.y + p2.y)/2)

#Eyes:
def get_blinking_ratio(eye_points, facial_landmarks):
    left_point = (facial_landmarks.part(eye_points[0]).x, facial_landmarks.part(eye_points[0]).y)
    right_point = (facial_landmarks.part(eye_points[3]).x, facial_landmarks.part(eye_points[3]).y)
    hor_line = cv2.line(frame, left_point, right_point,(0,255,0), 1)

    center_top = midpoint(facial_landmarks.part(eye_points[1]), facial_landmarks.part(eye_points[2]))
    center_bottom = midpoint(facial_landmarks.part(eye_points[5]), facial_landmarks.part(eye_points[4]))
    ver_line = cv2.line(frame, center_top, center_bottom,(0,255,0), 1)

    #length of the line
    hor_line_length = hypot((left_point[0] - right_point[0]), (left_point[1] - right_point[1]))
    ver_line_length = hypot((center_top[0] - center_bottom[0]), (center_top[1] - center_bottom[1]))
    ratio = hor_line_length/ ver_line_length, ver_line_length
    return ratio

#Mouth:
def get_mouth_gap(mouth_points, facial_landmarks):
	a_top = (facial_landmarks.part(mouth_points[0]).x, facial_landmarks.part(mouth_points[0]).y)
	a_low = (facial_landmarks.part(mouth_points[1]).x, facial_landmarks.part(mouth_points[1]).y)
	a_line = cv2.line(frame, a_top, a_low, (0,255,0), 1) #for show
	a_dist = dist.euclidean(a_top, a_low)
	
	b_top = (facial_landmarks.part(mouth_points[2]).x, facial_landmarks.part(mouth_points[2]).y)
	b_low = (facial_landmarks.part(mouth_points[3]).x, facial_landmarks.part(mouth_points[3]).y)
	b_line = cv2.line(frame, b_top, b_low, (0,255,0), 1) #for show
	b_dist = dist.euclidean(b_top, b_low) #could be this in isolation
	
	c_top = (facial_landmarks.part(mouth_points[4]).x, facial_landmarks.part(mouth_points[4]).y)
	c_low = (facial_landmarks.part(mouth_points[5]).x, facial_landmarks.part(mouth_points[5]).y)
	c_line = cv2.line(frame, a_top, a_low, (0,255,0), 1) #for show
	c_dist = dist.euclidean(c_top, c_low) #could be this in isolation
	
	abc = (a_dist + b_dist + c_dist) / 3.0
	return b_dist
	
#Yaw:
def get_yaw(face_points, facial_landmarks):
	middle_point = 	(facial_landmarks.part(face_points[0]).x, facial_landmarks.part(face_points[0]).y)
	left_point 	= 	(facial_landmarks.part(face_points[1]).x, facial_landmarks.part(face_points[1]).y)
	right_point = 	(facial_landmarks.part(face_points[2]).x, facial_landmarks.part(face_points[2]).y)
	
	left_line = cv2.line(frame, middle_point, left_point, (0,255,0), 1)
	right_line = cv2.line(frame, middle_point, right_point, (0,255,0), 1)
	
	#length of the line
	left_dist = dist.euclidean(middle_point, left_point)
	right_dist = dist.euclidean(middle_point, right_point)
	
	yaw = right_dist - left_dist #how should yaw be represented?
	return yaw

#Servo:
def ard_map(x, in_min, in_max, out_min, out_max): #"ard" is for "Arduino"
	y = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
	return y
	
#M&T:
def is_distracted(yaw, blink):
	if abs(yaw) > 4:
		return True
	if blink==1:
		return True
	else:
		return False #participant is not distracted.

#=================================================================
# ~~LOOP~~	
toggle = 1 # 1--robot, 2--M&T, 3--"paperless"

blink = 1
close = 1
TOTAL = 0
thres = 5.1

NECK_MID=40
neck.write(NECK_MID)
current_yaw = NECK_MID
skull_yaw = current_yaw
tick = 0

while True:
	_, frame = cap.read()
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)#for gray images(lightweight)
	faces = detector(gray)
	for face in faces:
		#Eyes:
		landmarks = predictor(gray, face)
		left_eye_ratio,_ = get_blinking_ratio([36,37,38,39,40,41], landmarks)
		right_eye_ratio, myVerti = get_blinking_ratio([42,43,44,45,46,47], landmarks)
		blinking_ratio = (left_eye_ratio+right_eye_ratio)/2
		personal_threshold = 0.67 * myVerti #0.67 is just the best constant I found with experimentation
		if (left_eye_ratio>personal_threshold or right_eye_ratio>personal_threshold) and blink==1:
			TOTAL += 1
			#time.sleep(0.3)#average persons blinking time
		if (left_eye_ratio>personal_threshold or right_eye_ratio>personal_threshold):
			blink = 0
		else:
			blink = 1
		cv2.putText(frame, "Blinks: {}".format(TOTAL), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

		#Mouth:
		mouth_marks = [61,67,62,66,63,65]
		mouth_gap = get_mouth_gap(mouth_marks, landmarks)
		personal_threshold = 0.67 * mouth_gap #0.67 is a constant from blink.py
		print(mouth_gap)
		
		#Yaw:
		face_marks = [33, 2, 14]
		yaw = get_yaw(face_marks, landmarks)
		personal_threshold = 0.67 * yaw #0.67 is a constant from blink.py
		print(yaw)
		
		#ROBOTIC:
		if toggle==1:
			#SkullEyes:
			if blink==1:
				board.digital[LED_PIN].write(1)
			else:
				board.digital[LED_PIN].write(0)
			#Jaw:
			jaw_gap = ard_map(mouth_gap, 0, 30, 50, 20)
			jaw.write(jaw_gap)
			#Yaw:
			if tick==3:
				skull_yaw = ard_map(yaw, -125, 125, 30, 70)
				tick = 0
			else:
				tick += 1
			
			current_yaw = neck.read()
			if current_yaw != skull_yaw:
				neck.write(skull_yaw)
		
		#M&T:
		if toggle==2:
			if blink==0:
				cv2.putText(frame, "Distracted", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
			else:
				if yaw <= 7:
					cv2.putText(frame, "Attentive", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
				else:
					cv2.putText(frame, "Distracted", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
			"""
			if blink==1 or yaw < 4:
				cv2.putText(frame, "Attentive", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
			else:
				cv2.putText(frame, "Distracted", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
			"""
			
			
		#"paperless"
		if toggle==3:
			board.digital[LED_PIN].write(0)
			time.sleep(5)
			board.digital[LED_PIN].write(1)
			time.sleep(5)
			neck.write(30)
			neck.write(60)
			neck.write(45)

	cv2.imshow("Frame", frame)

	key = cv2.waitKey(5)
	if key == 49:
		toggle=1
	if key == 50:
		toggle=2
	if key == 51:
		toggle=3
	if key == 27:
		break

cap.release()
cv2.destroyAllWindow()
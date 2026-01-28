import cv2
import mediapipe as mp
import serial
import time
import math

# Check ESP32 is there or not (I use udev rules, you can change it into COM or /dev/ttyUSBX)
try:
	arduino = serial.Serial(port='/dev/myESP32', baudrate=115200, timeout=0.1)
	time.sleep(2)
	print("ESP32 Connected.")
except: 
	print("WARNING: Couldn't detect ESP32")
	arduino = None

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1)
mp_drawing = mp.solutions.drawing_utils
pTime = 0
last_time = 0
cap = cv2.VideoCapture(0)

# Low Pass Filter
smoothed_pwm = [0, 0, 0, 0, 0] 
alpha = 0.5 # <-- You can adjust this (I use 0.25)
# Higher: more fast, but jitter
# Lower: more stable, but slow


print("Press 'q' to exit.")

# Calculate distance difference using the Pythagorean theorem.

def get_distance(p1, p2, width, height):
	x1, y1 = p1.x * width, p1.y * height
	x2, y2 = p2.x * width, p2.y * height
	return math.hypot(x2 - x1, y2 - y1)

def map_range(value, in_min, in_max, out_min, out_max):
	result = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
	return max(out_min, min(out_max, int(result)))

def get_pwm_values(image, hand_landmarks):
	h, w, c = image.shape

	# Calculate palm size (Scale Reference)
	# Distance from Wrist (0) to Middle Finger (9)

	palm_size = get_distance(hand_landmarks.landmark[0], 
                             hand_landmarks.landmark[9], w, h)

	if palm_size == 0: return [0,0,0,0,0], 0

	pwm_values = []

	# Tips: [Index(8), Middle(12), Ring(16), Pinky(20)]
	finger_tips = [8, 12, 16, 20]

	# Finger rati
	# Closed -> palm_size * ~1.0
	# Open -> palm_size * ~1.8
	# For thumb, it's has special case
	# 	Closed -> palm_size * ~0.6
	# 	Open -> palm_size * ~1.3
	min_ratio = 1.0 
	max_ratio = 1.8
	thumb_min_ratio = 0.6
	thumb_max_ratio = 1.3

	thumb_tip = hand_landmarks.landmark[4]
	pinky_base = hand_landmarks.landmark[17]
	thumb_dist = get_distance(thumb_tip, pinky_base, w, h)
	thumb_ratio = thumb_dist / palm_size

	thumb_pwm = map_range(thumb_ratio, thumb_min_ratio, thumb_max_ratio, 0, 255)
	pwm_values.append(thumb_pwm)

	# For another finger
	wrist = hand_landmarks.landmark[0]
	for tip_id in finger_tips:
		tip = hand_landmarks.landmark[tip_id]
		dist = get_distance(tip, wrist, w, h)

		ratio = dist / palm_size

		finger_pwm = map_range(ratio, min_ratio, max_ratio, 0, 255)
		pwm_values.append(finger_pwm)

	return pwm_values, palm_size


while cap.isOpened():
	success, image = cap.read()
	if not success: break

	image = cv2.flip(image, 1)
	# RGB Conversion
	results = hands.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

	if results.multi_hand_landmarks:
		for hand_landmarks in results.multi_hand_landmarks:
			mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
			pwm_list_raw, ref_size = get_pwm_values(image, hand_landmarks)

			# Measure PWM (and smooth it with EMU/Low Pass Filter)
			pwm_list, ref_size = get_pwm_values(image, hand_landmarks)
			for i in range(len(pwm_list_raw)):
				smoothed_pwm[i] = (alpha * pwm_list_raw[i]) + ((1 - alpha) * smoothed_pwm[i])
			pwm_list_smoothed = [int(p) for p in smoothed_pwm]

			# Send data to Arduino
			current_time = time.time()
			if arduino and (current_time - last_time > 0.05): # 20fps cap
				msg = "$" + ",".join(map(str, pwm_list_smoothed)) + "\n"
				arduino.write(msg.encode('utf-8'))
				last_time = current_time
				print(f"PWM: {pwm_list_smoothed} (Size: {ref_size:.1f})")

	# FPS Counter
	cTime = time.time()
	diff = cTime - pTime
	fps = 1 / diff if diff > 0 else 0
	pTime = cTime
	cv2.putText(image, f"FPS: {int(fps)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

	cv2.imshow('PWM Finger Control', image)
	if cv2.waitKey(5) & 0xFF == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()


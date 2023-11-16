import cv2
import numpy as np

# Load the template image
template = cv2.imread('nurse.jpg')

# Convert the template to HSV color space
hsv_template = cv2.cvtColor(template, cv2.COLOR_BGR2HSV)

# Define the range for red color in HSV
lower_red = np.array([0, 100, 100])
upper_red = np.array([10, 255, 255])


# Create a binary mask for the red color
mask = cv2.inRange(hsv_template, lower_red, upper_red)

# 闭操作
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

# Save the mask as Template.jpg
cv2.imwrite('Template.jpg', mask)

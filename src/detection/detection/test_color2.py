import cv2
import numpy as np

# Load the image from the rosbag
image = cv2.imread('/home/rosuser/dd2419_ws/src/detection/detection/multObjects.png')  # Replace 'your_image.jpg' with the actual file path

# Convert the image from BGR to HSV
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Define initial HSV values (you can change these values)
initial_hsv_lower = np.array([0, 0, 0])
initial_hsv_upper = np.array([255, 255, 255])

# Create a window to display trackbars
cv2.namedWindow('HSV Trackbars')

def on_trackbar_change(x):
    pass

# Create trackbars for adjusting HSV values
cv2.createTrackbar('Hue Lower', 'HSV Trackbars', initial_hsv_lower[0], 255, on_trackbar_change)
cv2.createTrackbar('Saturation Lower', 'HSV Trackbars', initial_hsv_lower[1], 255, on_trackbar_change)
cv2.createTrackbar('Value Lower', 'HSV Trackbars', initial_hsv_lower[2], 255, on_trackbar_change)

cv2.createTrackbar('Hue Upper', 'HSV Trackbars', initial_hsv_upper[0], 255, on_trackbar_change)
cv2.createTrackbar('Saturation Upper', 'HSV Trackbars', initial_hsv_upper[1], 255, on_trackbar_change)
cv2.createTrackbar('Value Upper', 'HSV Trackbars', initial_hsv_upper[2], 255, on_trackbar_change)

while True:
    # Get current trackbar positions
    hue_lower = cv2.getTrackbarPos('Hue Lower', 'HSV Trackbars')
    saturation_lower = cv2.getTrackbarPos('Saturation Lower', 'HSV Trackbars')
    value_lower = cv2.getTrackbarPos('Value Lower', 'HSV Trackbars')

    hue_upper = cv2.getTrackbarPos('Hue Upper', 'HSV Trackbars')
    saturation_upper = cv2.getTrackbarPos('Saturation Upper', 'HSV Trackbars')
    value_upper = cv2.getTrackbarPos('Value Upper', 'HSV Trackbars')

    # Update the lower and upper HSV values
    hsv_lower = np.array([hue_lower, saturation_lower, value_lower])
    hsv_upper = np.array([hue_upper, saturation_upper, value_upper])

    # Create a mask based on the current HSV values
    mask = cv2.inRange(hsv_image, hsv_lower, hsv_upper)

    # Apply the mask to the original image
    result = cv2.bitwise_and(image, image, mask=mask)

    # Display the original image, mask, and result
    cv2.imshow('Original Image', image)
    cv2.imshow('HSV Mask', mask)
    cv2.imshow('Result', result)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

cv2.destroyAllWindows()

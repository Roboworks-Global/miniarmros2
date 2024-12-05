import cv2
import numpy as np

# Global variables to store HSV ranges for multiple colors
hsv_ranges = {
    "green": [],
    "blue": [],
    "yellow": [],
    "red": [],
}

colour_codes = {
    "red": (0, 0, 255),
    "green": (0, 255, 0),
    "blue": (255, 0, 0),
    "yellow": (0, 255, 255),
}

# Variable to store the current selected color
current_color = None
H_TOLERANCE = 10
S_TOLERANCE = 20
V_TOLERANCE = 20

# Function to handle mouse click events
def get_hsv(event, x, y, flags, param):
    global hsv_ranges, current_color
    if event == cv2.EVENT_LBUTTONDOWN and current_color is not None:
        # Get the BGR value of the clicked pixel
        bgr_pixel = frame[y, x]
        
        # Convert the BGR pixel to HSV
        hsv_pixel = cv2.cvtColor(np.uint8([[bgr_pixel]]), cv2.COLOR_BGR2HSV)[0][0]
        
        # Set the HSV range to +/- 15 for H, +/- 40 for S and V
        hsv_range = (
            np.array([max(hsv_pixel[0] - H_TOLERANCE, 0), max(hsv_pixel[1] - S_TOLERANCE, 0), max(hsv_pixel[2] - V_TOLERANCE, 0)]),
            np.array([min(hsv_pixel[0] + H_TOLERANCE, 179), min(hsv_pixel[1] + S_TOLERANCE, 255), min(hsv_pixel[2] + V_TOLERANCE, 255)])
        )

        hsv_ranges[current_color].append(hsv_range)
        print(f"Clicked Pixel at ({x}, {y}) - BGR: {bgr_pixel}, HSV: {hsv_pixel}")
        print(f"Added HSV Range: {hsv_range} to {current_color}")

        current_color = None

# Function to handle key presses for selecting color
def select_color(key):
    global current_color
    if key == ord('g'):
        current_color = "green"
        print(f"Selected Color: {current_color}")
    elif key == ord('b'):
        current_color = "blue"
        print(f"Selected Color: {current_color}")
    elif key == ord('r'):
        current_color = "red"
        print(f"Selected Color: {current_color}")
    elif key == ord('y'):
        current_color = "yellow"
        print(f"Selected Color: {current_color}")


# Open a connection to the USB camera (0 is usually the built-in camera, 1 or higher for external USB cameras)
cap = cv2.VideoCapture(0)

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Create windows and set the mouse callback function
cv2.namedWindow('Mask')
cv2.namedWindow('Camera Feed')
cv2.setMouseCallback('Camera Feed', get_hsv)

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    # If frame is read correctly, ret is True
    if not ret:
        print("Error: Failed to grab frame.")
        break
    
    # Display the camera feed

    # Create a copy of the frame to draw bounding boxes
    detected_cubes = frame.copy()
    
    if hsv_ranges:
        # Initialize a mask with zeros
        combined_combined_mask = np.zeros(frame.shape, dtype=np.uint8)
        
        for colour in hsv_ranges.keys():
            combined_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
            for hsv_range in hsv_ranges[colour]:
                hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv_frame, hsv_range[0], hsv_range[1])
                combined_mask = cv2.bitwise_or(combined_mask, mask)

            
            # Detect contours in the combined mask
            contours, _ = cv2.findContours(combined_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            mask_with_contours = cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR)
            cv2.drawContours(mask_with_contours, contours, -1, colour_codes[colour], 2)

            combined_combined_mask = cv2.bitwise_or(combined_combined_mask, mask_with_contours)

            for contour in contours:
                if cv2.contourArea(contour) > 200:  # Filter small areas
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(detected_cubes, (x, y), (x + w, y + h), colour_codes[colour], 2)
    
        # Display the detected cubes
        cv2.imshow('Mask', combined_combined_mask)
        cv2.imshow('Camera Feed', detected_cubes)
    else:
        cv2.imshow('Camera Feed', frame)

    
    # Handle key presses for color selection
    key = cv2.waitKey(1)
    if key & 0xFF == 27:  # 27 is the ESC key
        break
    select_color(key)

# When everything is done, release the capture and close windows
cap.release()
cv2.destroyAllWindows()

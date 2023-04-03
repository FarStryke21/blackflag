import cv2
import numpy as np

# Open the default camera
cap = cv2.VideoCapture(0)

# Set the resolution of the camera
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Sharpen Image
kernel = np.array([[-1,-1,-1],[-1,9,-1],[-1,-1,-1]])


# Loop over frames from the camera
while True:
    # Read a frame from the camera
    ret, frame = cap.read()

    # Check if the frame was successfully read
    if not ret:
        print("Error reading frame from camera")
        break
    frame2 = cv2.filter2D(frame, -1, kernel)

    # Display the frame
    cv2.imshow("Camera", frame)

    # Wait for a key press
    if cv2.waitKey(1) == ord('q'):
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()

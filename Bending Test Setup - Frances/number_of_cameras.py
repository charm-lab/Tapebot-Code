import cv2

# Function to capture and show the camera feed


def show_camera_feed():
    # Create a VideoCapture object to access the camera
    # 0 represents the default camera, you can change it to a different camera index if you have multiple cameras
    cap = cv2.VideoCapture(0)

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        if not ret:
            print("Failed to grab a frame.")
            break

        # Display the frame in a window called "Camera Feed"
        cv2.imshow("Camera Feed", frame)

        # Check for 'q' key to exit the loop and close the window
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the VideoCapture and close the window
    cap.release()
    cv2.destroyAllWindows()


# Call the function to start showing the camera feed
show_camera_feed()

import cv2
from matplotlib import pyplot as plt

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
output_filename = 'output_video.avi'
fps = 30.0  # You can adjust the frames per second (fps) as needed
output_video = cv2.VideoWriter(output_filename, fourcc, fps, (640, 480))

while True:
    ret, frame = vid.read()

    # check if frames is empty
    if not ret:
        print("Error: Unable to read frame")
        break

    print(frame.shape)

    # Display the live video stream
    cv2.imshow('frame', frame)

    # Write the frame to the output video file
    output_video.write(frame)

    # Exit the loop and stop recording when 's' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('s'):
        break

# Release the video objects and close the OpenCV windows

output_video.release()
cv2.destroyAllWindows()

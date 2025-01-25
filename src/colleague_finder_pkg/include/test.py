import cv2
from ColleagueFinder import ColleagueFinder

# Initialize the ColleagueFinder
colleague_finder = ColleagueFinder()

# Open the video capture (for example, from a file or external source)
cap = cv2.VideoCapture(0)  # Replace with the external video source

if not cap.isOpened():
    print("[ERROR] Could not open video source.")
    exit()

# Loop over the video frames
while True:
    # Read a frame from the video
    ret, frame = cap.read()

    if not ret:
        print("[ERROR] Failed to read frame from video source.")
        break

    # Get colleague info (Aruco markers detected in the frame)
    markers = colleague_finder.getColleagueInfo(frame)

    # Loop over each detected marker and display its information
    for marker in markers:
        # Print the marker details (optional)
        marker.describe()

        # Draw a red dot at the center of the marker (cX, cY)
        cv2.circle(frame, (marker.cX, marker.cY), 5, (0, 0, 255), -1)  # red dot at (cX, cY)

        # Display the marker ID next to the red dot
        cv2.putText(frame, f"ID: {marker.id}", (marker.cX - 10, marker.cY - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Show the updated frame with marker positions and IDs
    cv2.imshow("Frame", frame)

    # Press 'q' to quit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture object and close any open windows
cap.release()
cv2.destroyAllWindows()

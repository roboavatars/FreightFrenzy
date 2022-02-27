import cv2
import numpy as np
import time

def filter_frame(img):
    """
    filter_frame(img): processes input frame
    changes to hsv colorspace
    implements color filtering
    uses morphology to reduce noise
    :param img: input frame
    :returns: filteredRed (filtered image for red balls);
    filteredBlue (filtered image for blue balls);
    filtered (OR combination of filteredRed and filteredBlue);
    mask (combination of filtered and img)
    """

    # HSV Color Filtering
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    filteredRed = cv2.bitwise_or(cv2.inRange(hsv, (0, 120, 60), (1, 255, 255)), cv2.inRange(hsv, (160, 120, 60), (180, 255, 255)))
    filteredBlue = cv2.inRange(hsv, (100, 100, 70), (135, 255, 255))

    # Reduce Background Noise
    cv2.morphologyEx(filteredRed, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    cv2.morphologyEx(filteredRed, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    cv2.morphologyEx(filteredBlue, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    cv2.morphologyEx(filteredBlue, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))

    # Combine Red and Blue Filtered Images
    filtered = cv2.bitwise_or(filteredRed, filteredBlue)

    # Generate Mask Image
    mask = cv2.bitwise_and(img, img, mask = filtered)

    return filteredRed, filteredBlue, filtered, mask

def detect_circle(img, filtered, master, color, text_position):
    """
    detect_circle(img, filtered, master, color, text_position): detects balls in frame
    finds and filters contours
    bounds contours with ellipse and continues filtering contours
    draws circle outline on valid contours
    marks circle center and puts circle center telemetry on master frame
    :param img: input frame
    :param filtered: color-specific filtered frame
    :param master: master frame for writing text
    :param color: BGR value for drawing circle
    :param text_position: frame position for ball pixel coordinate telemetry
    """

    # Generate Contours
    contours = cv2.findContours(filtered, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)[0]

    # Loop for Every Contour
    for contour in contours:
        # Contour Size Threshold
        if cv2.contourArea(contour) > 100:
            ellipse = cv2.fitEllipse(contour)
            # Ellipse Roundness Threshold
            if 0.8 < ellipse[1][0] / ellipse[1][1]:
                # Draw Circle
                cv2.ellipse(img, ellipse, color, 4)

                # Draw / Label Center
                center = (int(ellipse[0][0]), int(ellipse[0][1]))
                cv2.circle(img, center, 3, (0, 255, 0), -1)
                cv2.putText(master, str(center), text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 1, cv2.LINE_AA)

def main():
    cam = cv2.VideoCapture(0)

    # Loop for Each Test Image
    while True:
        # for i in list(range(1481, 1483)) + list(range(1484, 1514)) + list(range(1515, 1552)):

        # Read Image
        ts1 = time.time()
        img = cam.read()[1]
        # img = cv2.imread(f'imgs/img{i}.jpg')
        img = cv2.resize(img, (480, 360))

        # Create Master Output Image
        master = 255 * np.ones((870, 960, 3), dtype = np.uint8)
        ts2 = time.time()

        # Process Image
        filteredRed, filteredBlue, filtered, mask = filter_frame(img)

        # Detect Balls
        output = img.copy()
        detect_circle(output, filteredRed, master, (0, 0, 255), (590, 840))
        detect_circle(output, filteredBlue, master, (255, 0, 0), (760, 840))
        ts3 = time.time()

        # Display Images
        master[30:390, 0:480] = img
        master[30:390, 480:960] = cv2.cvtColor(filtered, cv2.COLOR_GRAY2BGR)
        master[420:780, 480:960] = mask
        master[420:780, 0:480] = output

        # Add Labels
        cv2.putText(master, 'Input Image', (195, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(master, 'Filtered Image', (675, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(master, 'Output Image', (195, 410), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(master, 'Image Mask', (675, 410), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(master, 'Image Reading', (65, 800), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(master, 'Image Processing', (260, 800), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(master, 'Ball Pixel Coordinates', (655, 800), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

        cv2.putText(master, f'{round(1000 * (ts2 - ts1))} ms', (95, 830), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(master, '{:.2f} fps'.format(1 / (ts2 - ts1)), (95, 860), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(master, '{:.2f} ms'.format(1000 * (ts3 - ts2)), (300, 830), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(master, f'{round(1 / (ts3 - ts2))} fps', (300, 860), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

        cv2.imshow('Ball Detection', master)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Cleanup
    cam.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

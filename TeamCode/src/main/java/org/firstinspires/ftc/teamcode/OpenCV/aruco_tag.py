import cv2

aruco_tags = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
cam = cv2.VideoCapture(1) # Might need to change to 0 if you only have 1 webcam on laptop

while True:
    img = cam.read()[1]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_tags, parameters = cv2.aruco.DetectorParameters_create())

    if ids is not None:
        for tag in range(len(ids[0])):
            if ids[tag] in [0]:
                tag_corners = [(int(corner[0]), int(corner[1])) for corner in corners[0][tag]]
                cv2.line(img, tag_corners[0], tag_corners[1], (0, 255, 0), 3)
                cv2.line(img, tag_corners[1], tag_corners[2], (0, 255, 0), 3)
                cv2.line(img, tag_corners[2], tag_corners[3], (0, 255, 0), 3)
                cv2.line(img, tag_corners[3], tag_corners[0], (0, 255, 0), 3)

    cv2.imshow('Detections', img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cam.release()
cv2.destroyAllWindows()

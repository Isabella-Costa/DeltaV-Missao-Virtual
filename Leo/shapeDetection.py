import cv2 as cv

target_width, target_height = 500, 500
img = cv.imread('Leo\media\missao-shapes.jpg')
img = cv.resize(img, (target_width, target_height), interpolation=cv.INTER_AREA)

gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
_, thresh = cv.threshold(gray, 220, 255, cv.THRESH_BINARY)
contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
for i, contour in enumerate(contours):
    if i == 0: # Ignore the first contour (the whole image)
        continue

    # Approximate the shape we want
    epsilon = 0.01*cv.arcLength(contour, True)
    approx = cv.approxPolyDP(contour, epsilon, True)

    
    cv.drawContours(img, contour, -1, (255,255,0), 3)
    # Coordinates of the contour
    x,y,w,h = cv.boundingRect(approx)
    x_mid = int(x)
    y_mid = int(y+30)

    coords = (x_mid, y_mid)
    colour = (255,0,0)
    font = cv.FONT_HERSHEY_DUPLEX
    font_scale = 0.9
    font_thickness = 2

    # Recognize the shape based on amount of corners
    if len(approx) == 3:
        cv.putText(img, "Triangulo", coords, font, font_scale, colour, font_thickness)
    elif len(approx) == 4:
        cv.putText(img, "Quadrilatero", coords, font, font_scale, colour, font_thickness)
    elif len(approx) == 5:
        cv.putText(img, "Pentagono", coords, font, font_scale, colour, font_thickness)
    elif len(approx) == 6:
        cv.putText(img, "Hexagono", coords, font, font_scale, colour, font_thickness)
    elif len(approx) == 10:
        cv.putText(img, "Estrela", coords, font, font_scale, colour, font_thickness)
    elif len(approx) == 12:
        cv.putText(img, "Cruz", coords, font, font_scale, colour, font_thickness)
    else:
        cv.putText(img, "Circulo", coords, font, font_scale, colour, font_thickness)

cv.imshow('Shape', img)
cv.waitKey(0)
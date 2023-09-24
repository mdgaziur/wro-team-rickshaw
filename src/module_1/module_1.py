from picamera.array import PiRGBArray
from picamera import PiCamera
from serial import Serial
import cv2 as cv
import imutils
import time
 
# Minimum height a contour must have before it is deemed valid for detection
MIN_HEIGHT = 210
MIN_WIDTH = 120

def mask_img(img, a_channel, thres_min, thres_max, flag):
    thres = cv.threshold(a_channel, thres_min, thres_max, flag)[1]
    masked = cv.cvtColor(cv.bitwise_and(img, img, mask=thres), cv.COLOR_BGR2GRAY) 
    contours = imutils.grab_contours(cv.findContours(masked, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE))
 
    if len(contours) > 0:
        return max(contours, key=lambda contour: cv.boundingRect(contour)[1] and cv.boundingRect(contour)[2] > 40)
    else:
        return None
 
 
def nothing(x): pass

def main():
    camera = PiCamera()
    camera.resolution = (336, 240)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(336, 240))
    serial = Serial('/dev/ttyACM0', 9600, timeout=1)
    time.sleep(0.1)
 
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        img = imutils.rotate_bound(frame.array, 180)

        lab = cv.cvtColor(img, cv.COLOR_BGR2LAB)
        a_channel = lab[:, :, 1]  # WHAT THE FUCK IS THIS?!
 
        green_contour = mask_img(img, a_channel, 120, 255, cv.THRESH_BINARY_INV)
        _, _, gc_width, gc_height = cv.boundingRect(green_contour)
 
        red_contour = mask_img(img, a_channel, 150, 160, cv.THRESH_BINARY)
        _, _, rc_width, rc_height = cv.boundingRect(red_contour)

        print(f"{gc_height} {gc_width} {rc_height} {rc_width}")

        if gc_height >= MIN_HEIGHT and gc_width >= MIN_WIDTH and gc_width > rc_width:
            print('\rDirection: LEFT', flush=True, end='')
            serial.write(b'L')
        elif rc_height >= MIN_HEIGHT and rc_width >= MIN_WIDTH and rc_width > gc_width:
            print('\rDirection: RIGHT ', flush=True, end='')
            serial.write(b'R')
        else:
            print('\rNo detection    ', flush=True, end='')
            serial.wrote(b'N')

        if green_contour is not None:
            cv.drawContours(img, [green_contour], 0, (50, 250, 50), 4)
        if red_contour is not None:
            cv.drawContours(img, [red_contour], 0, (50, 250, 50), 4)
        cv.imshow('Result', img)
        key = cv.waitKey(1) & 0xFF
 
        rawCapture.truncate(0)
 
        if key == ord('q'):
            break
 
    cv.destroyAllWindows()
 
 
if __name__ == "__main__":
    main()
    print()

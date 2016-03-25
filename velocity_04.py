import RPi.GPIO as GPIO
import datetime
import pygame, sys
from pygame.locals import *
import time
import pygame.camera

MS_2_KMH = 3.6 #conversion metros/segundos a km/h
LASERS_DISTANCE = 8.0 #metros de laser1 a laser2 
SPEED_LIMIT = 30.0 # [km/h]

GPIO.setmode(GPIO.BCM)
GPIO.setup(4,GPIO.IN) ## laser 1 al raspi gpio 4
GPIO.setup(23,GPIO.IN)## laser 2 al raspi gpio 23

def initialize_camera():
    pygame.init()
    pygame.camera.init()
    camlist = pygame.camera.list_cameras()
    if camlist:
        cam = pygame.camera.Camera(camlist[0],(640,480))
        cam.start()
        return cam

def publish():
    pass

def take_photo(cam):
    image = cam.get_image()
    pygame.image.save(image, datetime.datetime.now().isoformat() + ".jpg")

if __name__ == "__main__":
    cam = initialize_camera()
    while True:
        while GPIO.input(4) == 0:
            pass
		
        print "paso sensor 1"
        start_time = time.time()
        time.sleep(0.57)
        while GPIO.input(23) == 0:
            pass

        end_time = time.time()
        print "paso sensor 2"

        print end_time-start_time
        delta_time = end_time - start_time
        print delta_time, "delta_time"
        speed = MS_2_KMH * (LASERS_DISTANCE / delta_time)
        if speed > SPEED_LIMIT:
            take_photo(cam)
            publish()
        print "SPEED = ", speed
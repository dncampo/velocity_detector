import datetime
import pygame,sys
from pygame.locals import *

import pygame.camera


pygame.init()
pygame.camera.init()


camlist=pygame.camera.list_cameras()
if camlist:
	cam= pygame.camera.Camera(camlist[0],(640,480))
	cam.start() 
	image=cam.get_image()
	## ir cambiando el nombre del archivo    
	pygame.image.save(image,datetime.datetime.now().isoformat() + ".jpg")


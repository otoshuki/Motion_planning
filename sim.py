import random
import time
import numpy as np
from graphics import *
import math

def draw_windows(w, h):
    # create windows
    win=GraphWin('swarm', w, h)
    win.setCoords(0,0,w,h)
    return win


class robot:
	max_range=50
	def create_bot(self,win,x,y):
		self.shape=Circle(Point(x, y), 10)
		self.shape.draw(win)

		self.forward=Line(Point(x,y),Point(x,y+self.max_range))
		self.forward.draw(win)

		new_x=self.max_range*math.sin(60)
		new_y=self.max_range*math.cos(60)
		self.ping[i]=Line(Point(x,y),Point(new_x,new_y))
		self.ping[i].draw(win)





	#p = Polygon(Point(1,1), Point(5,3), Point(2,7))
    #p.draw(win)








if __name__ == '__main__':
	win=draw_windows(400,400)
	bee=robot()
	bee.create_bot(win,20,20)


win.getMouse()

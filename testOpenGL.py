import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

class Demo(object):

    def __init__(self) -> None:
        glutInit()
        glutInitDisplayMode(GLUT_SINGLE | GLUT_RGBA)
        glutInitWindowSize(400, 400)
        glutCreateWindow("Demo")
        glutDisplayFunc(self.draw_geometry)
        self.init_condition()
        glutMainLoop()
    
    def init_condition(self):
        glClearColor(1.0, 1.0, 1.0, 1.0)
        gluOrtho2D(-8.0, 8.0, -8.0, 8.0)
    
    def draw_geometry(self):
        glClear(GL_COLOR_BUFFER_BIT)
        glColor3f(1.0, 0.0, 0.0)
        glBegin(GL_QUADS)
        glVertex2f(-2, 2)
        glVertex2f(-2, 5)
        glVertex2f(-5, 5)
        glVertex2f(-5, 2)
        glEnd()
        glFlush()

if __name__ == "__main__":
    Demo()


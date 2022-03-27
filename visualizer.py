from PyQt5 import QtOpenGL, QtWidgets, QtCore, QtGui
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

import math
import argparse
from Skeleton import Skeleton

WIDTH = 640
HEIGHT = 480

bvh_path = None
# bvh_path = "result.bvh"

class bvh_visual(QtOpenGL.QGLWidget):

    def __init__(self, parent=None) -> None:
        self.parent = parent
        self.frame = 0
        self.skeleton = Skeleton(bvh_path, 0.25)
        self.skeleton.updateFrame(self.frame)
        self.offset = self.skeleton.root.worldpos
        QtOpenGL.QGLWidget.__init__(self, parent)
    
    def initializeGL(self):
        glClearColor(0.5, 0.5, 0.5, 0.0)
        glClearDepth(1.0)
        glDepthFunc(GL_LESS)
        glEnable(GL_DEPTH_TEST)
        glShadeModel(GL_SMOOTH)

    def updateFrame(self, frame):
        self.frame = frame
        self.skeleton.updateFrame(self.frame)
        print(self.skeleton.root.worldpos)
    
    def resizeGL(self, width, height):
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        aspect = width / float(height)

        gluPerspective(45.0, aspect, 1.0, 100.0)
        glMatrixMode(GL_MODELVIEW)
    
    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        glTranslatef(0.0, 1.0, -15.0)
        
        self.drawFloorPlane(-20, 20, 10, -5, True)
        self.drawBVHRig(self.skeleton)
        # glFlush()
    
    def drawFloorPlane(self, pmin, pmax, lines = 10, y = 0, faces = False):
        OFFSET = 0.2
        if faces:
            glColor3f(.3,.3,.3)
            glBegin(GL_QUADS)
            glVertex3f(pmin, y-OFFSET, pmin)
            glVertex3f(pmin, y-OFFSET, pmax)
            glVertex3f(pmax, y-OFFSET, pmax)
            glVertex3f(pmax, y-OFFSET, pmin)
            glEnd()

        size = pmax-pmin

        glLineWidth(2)
        glBegin(GL_LINES)
        for i in range(lines):
            if i == 0:
                glColor3f(.6,.3,.3)
            else:
                glColor3f(.25,.25,.25)
            pos = pmin + i*(size/lines)
            glVertex3f(pos,y,pmin)
            glVertex3f(pos,y,pmax)
            if i == 0:
                glColor3f(.3,.3,.6)
            else:
                glColor3f(.25,.25,.25)
            glVertex3f(pmin,y,pos)
            glVertex3f(pmax,y,pos)
        glEnd()
    
    def getPosition(self, joint):
        return [joint.worldpos[0] - self.offset[0],
                joint.worldpos[1] - self.offset[1],
                joint.worldpos[2] - self.offset[2]]
    
    def drawSphere(self, r, lats, longs, wireFrame = False):
        i = 0
        j = 0
        for i in range(lats+1):
            lat0 = math.pi * ((-0.5 + float(i) - 1) / lats)
            z0  = math.sin(lat0)
            zr0 =  math.cos(lat0)

            lat1 = math.pi * (-0.5 + float(i) / lats)
            z1 = math.sin(lat1)
            zr1 = math.cos(lat1)

            if wireFrame:
                glPolygonMode (GL_FRONT_AND_BACK, GL_LINE)
                glLineWidth(1)
                r = r + 0.0001

            glBegin(GL_QUAD_STRIP)
            for j in range(longs+1):
                lng = 2 * math.pi * float(j - 1) / longs
                x = math.cos(lng)
                y = math.sin(lng)

                glNormal3f(x * zr0 * r, y * zr0 * r, z0 * r)
                glVertex3f(x * zr0 * r, y * zr0 * r, z0 * r)
                glNormal3f(x * zr1 * r, y * zr1 * r, z1 * r)
                glVertex3f(x * zr1 * r, y * zr1 * r, z1 * r)
            glEnd()

            glPolygonMode (GL_FRONT_AND_BACK, GL_FILL)

    def drawJoint(self, joint):
        self.offset = self.skeleton.root.worldpos
        pos = self.getPosition(joint)
        #if pos[0] == pos[1] == pos[2] == 0:
        #    print joint.name
        RADIUS = 0.15

        glPushMatrix()
        glTranslatef(pos[0], pos[1], pos[2])
        glColor3f(0.7, 0.3, 0.2)
        self.drawSphere( RADIUS, 8, 8 )
        #glutSolidSphere( RADIUS, 8, 8 )
        glColor3f(0.0, 0.0, 0.0)
        #glutWireSphere( RADIUS, 8, 8 )
        self.drawSphere( RADIUS, 8, 8, True)

        glColor3f(0.7, 0.3, 0.2)
        glPopMatrix()

        if joint.parent:
            head = self.getPosition(joint.parent)
            tail = pos
            glLineWidth(3)
            glBegin(GL_LINES)
            glVertex3f(head[0], head[1], head[2])
            glVertex3f(tail[0], tail[1], tail[2])
            glEnd()

        for child in joint.children:
            self.drawJoint(child)
    
    def drawBVHRig(self, skeleton):
        glColor3f(0.7, 0.3, 0.2)
        self.drawJoint(skeleton.root)

class MainWindow(QtWidgets.QMainWindow):

    def __init__(self):
        QtWidgets.QMainWindow.__init__(self)    # call the init for the parent class
        
        self.resize(WIDTH, HEIGHT)
        self.setWindowTitle('BVH visualizer')

        self.glWidget = bvh_visual(self)
        self.initGUI()
        
        timer = QtCore.QTimer(self)
        timer.setInterval(20)   # period, in milliseconds
        timer.timeout.connect(self.glWidget.updateGL)
        timer.start()
        
    def initGUI(self):
        central_widget = QtWidgets.QWidget()
        gui_layout = QtWidgets.QVBoxLayout()
        central_widget.setLayout(gui_layout)

        self.setCentralWidget(central_widget)

        gui_layout.addWidget(self.glWidget)

        frames = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        frames.valueChanged.connect(lambda val: self.glWidget.updateFrame(val))

        gui_layout.addWidget(frames)

def parse_args():
    parser = argparse.ArgumentParser(description='bvh file visualizer')
    parser.add_argument('--bvh_path', type=str, help='file path', default="cmu_mb_01_01.bvh")
    args = parser.parse_args()
    return args

if __name__ == '__main__':
    arg = parse_args()
    bvh_path = arg.bvh_path
    app = QtWidgets.QApplication(sys.argv)
    
    win = MainWindow()
    win.show()

    sys.exit(app.exec_())
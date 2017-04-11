#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('jsk_laser')
from jsk_laser.msg import *
##pyqtgraph lib
from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg


#QtGui.QApplication.setGraphicsSystem('raster')

app = QtGui.QApplication([])
mw = QtGui.QMainWindow()
mw.setWindowTitle('JSK Laser Demo')
mw.resize(800,800)
cw = QtGui.QWidget()
mw.setCentralWidget(cw)
l = QtGui.QVBoxLayout()
cw.setLayout(l)

pw = pg.PlotWidget(name='JSK Laser Logger')  ## giving the plots names allows us to link their axes together
l.addWidget(pw)

mw.show()
## Create an empty plot curve to be filled later, set its pen
#then is buff_a  we have 5... 
p1 = pw.plot()
p1.setPen((255,0,0))
p2 = pw.plot()
p2.setPen((100,100,0))
p3 = pw.plot()
p3.setPen((200,40,40))
p4 = pw.plot()
p4.setPen((155,255,0))
p5 = pw.plot()
p5.setPen((100,200,0))
#then is buff_b  we also have 5... 
p6 = pw.plot()
p6.setPen((0,0,255))
p7 = pw.plot()
p7.setPen((40,40,255))
p8 = pw.plot()
p8.setPen((0,255,150))
p9 = pw.plot()
p9.setPen((0,200,100))
p10 = pw.plot()
p10.setPen((0,100,100))
#p11 is the distance
p11 = pw.plot()
p11.setPen((255,255,255))

## Add in some extra graphics
rect = QtGui.QGraphicsRectItem(QtCore.QRectF(0, 0, 1, 5e-11))
rect.setPen(QtGui.QPen(QtGui.QColor(100, 200, 100)))
pw.addItem(rect)

pw.setLabel('left', 'Distance', units='cm')
pw.setLabel('right', 'Charge data', units='milivolt/10')
pw.setLabel('bottom', 'Pixels', units='')
pw.setXRange(0, 272)
pw.setYRange(0, 300)

def rand(n):
    data = np.random.random(n)
    data[int(n*0.1):int(n*0.13)] += .5
    data[int(n*0.18)] += 2
    data[int(n*0.1):int(n*0.13)] *= 5
    data[int(n*0.18)] *= 20
    data *= 1e-12
    return data, np.arange(n, n+len(data)) / float(n)


def updateData():
    yd, xd = rand(10000)
    p1.setData(y=yd, x=xd)


def clicked():
    print("curve clicked")

def callback(data):
#    rospy.loginfo("%f", data.distances[100])
    data.distances = list(data.distances)
    for i in range(0, 20):
        data.distances[i] = np.asarray(data.distances[i]) * 0
    for i in range(210,271):
        data.distances[i] = np.asarray(data.distances[i]) * 0
    data.distances = np.asarray(data.distances)
    p11.setData(data.distances)

def rawdatacallback(data):

    mw.show()
    for j in range(0, 5):
        data_a = []
        data_b = []
        for i in range(0, 272):
            data_a.append(data.data_a[j*272+i])
            data_b.append(data.data_b[j*272+i])
        
        if j == 0:
            p1.setData(data_a)
            p6.setData(data_b)
        elif j == 1:
            p2.setData(data_a)
            p7.setData(data_b)
        elif j == 2:
            p3.setData(data_a)
            p8.setData(data_b)
        elif j == 3:
            p4.setData(data_a)
            p9.setData(data_b)
        elif j == 4:
            p5.setData(data_a)
            p10.setData(data_b)
        else:
            rospy.loginfo("Data error.....")

        mw.show()

def logger():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('laser_logger', anonymous=True)

    rospy.Subscriber("/laser_data", JskLaser, callback)
    rospy.Subscriber("/laserraw_data", JskLaserRaw, rawdatacallback)

    # spin() simply keeps python from exiting until this node is stopped


if __name__ == '__main__':
    logger()
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
        rospy.spin()

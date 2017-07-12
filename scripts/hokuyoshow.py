#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('jsk_laser')
roslib.load_manifest('sensor_msgs')
from jsk_laser.msg import *
from sensor_msgs.msg import *
##pyqtgraph lib
from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
import math
from sklearn.cluster import DBSCAN

normalshow = 1
enabelDBSCAN = 1
PI = math.atan(1)*4
p_length = 360

QtGui.QApplication.setGraphicsSystem('raster')

app = QtGui.QApplication([])
mw = QtGui.QMainWindow()
mw.setWindowTitle('Hokuyo to 2D Demo')
mw.resize(800,800)
cw = QtGui.QWidget()
mw.setCentralWidget(cw)
l = QtGui.QVBoxLayout()
cw.setLayout(l)

pw = pg.PlotWidget(name='Hokuyo laser 2D show')  ## giving the plots names allows us to link their axes together
l.addWidget(pw)

mw.show()
p11 = pw.plot()
p11.setPen((255,255,255),width=3)
p12 = pw.plot()
p12.setPen((255,0,255),width=5)

## Add in some extra graphics
rect = QtGui.QGraphicsRectItem(QtCore.QRectF(0, 0, 1, 5e-11))
rect.setPen(QtGui.QPen(QtGui.QColor(100, 200, 100)))
pw.addItem(rect)

pw.setLabel('left', 'Distance', units='cm')
#pw.setLabel('right', 'Charge data', units='milivolt/10')
pw.setLabel('bottom', 'Points', units='')
pw.setXRange(0, p_length)
pw.setYRange(0, 300)



def clicked():
    print("curve clicked")

def cluster_dist(data,eps_in, min_sample):
    #calculate the distance points..
    #the data from 1 to 256 is a different axis..
    
    points = np.zeros((p_length,2))
    angle_min =  PI/4
    angle_increment = PI/p_length /2
    for i in range(8,p_length-8):
        distance = 0
        if data.ranges[i]>0:
            distance = data.ranges[i] + 10  # set a offset to 0...
        points[i][0] = math.cos(PI-angle_min-angle_increment*i)*distance
        points[i][1] = math.sin(PI-angle_min-angle_increment*i)*distance
    db = DBSCAN(eps=eps_in, min_samples=min_sample).fit(points)
    labels = db.labels_

    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
#    print('Estimated number of clusters: %d' % n_clusters_)
    return labels, n_clusters_

kk = 0

def callback(data):
#    rospy.loginfo("%f", data.distances[100])
    global kk
    kk = kk +1
    if kk < 4 :
        return
    kk = 0
    data.ranges = list(data.ranges[180:540])
    for i in range(0,30):
        data.ranges[i] = 0
    for i in range(344,360):
        data.ranges[i] = 0

    for i in range(0,p_length):
        if data.ranges[i] < 0.5: #2 meter
            data.ranges[i] = data.ranges[i] * 100
        else:
            data.ranges[i] = 0

    for i in range(0,p_length/2):
        k = data.ranges[i]
        data.ranges[i] = data.ranges[p_length-1-i]
        data.ranges[p_length-1-i] = k
        

    if enabelDBSCAN:
        labels, n_clusters = cluster_dist(data,2,8)
        labeldata = list(data.ranges)

        for i in range(8,p_length-8):
            for j in range(0,n_clusters):
                if labels[i] == -1:
                    labels[i] = labels[i-1]
                    labeldata[i] = (labels[i-1]+2)*20 + 60
                else:
                    labeldata[i] = (labels[i]+2)*20 + 60

        p12.setData(labeldata)
        p11.setData(data.ranges)




def logger():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('hokuyo_2D')

    rospy.Subscriber("/scan", LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped


if __name__ == '__main__':
    logger()
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
        rospy.spin()

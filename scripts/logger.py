#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('jsk_laser')
from jsk_laser.msg import *
##pyqtgraph lib
from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
import math
from sklearn.cluster import DBSCAN

normalshow = 1
enabelDBSCAN = 1
focus = 3.6/5.12
PI = math.atan(1)*4
laser_name = 'J'
laser_link = 'laser_link'
rawdataplot = 1

QtGui.QApplication.setGraphicsSystem('raster')

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
p1.setPen((255,0,0),width=3)
p2 = pw.plot()
p2.setPen((100,100,0),width=3)
p3 = pw.plot()
p3.setPen((200,40,40),width=3)
p4 = pw.plot()
p4.setPen((155,255,0),width=3)
p5 = pw.plot()
p5.setPen((100,200,0),width=3)
#then is buff_b  we also have 5... 
p6 = pw.plot()
p6.setPen((0,0,255),width=3)
p7 = pw.plot()
p7.setPen((40,40,255),width=3)
p8 = pw.plot()
p8.setPen((0,255,150),width=3)
p9 = pw.plot()
p9.setPen((0,200,100),width=3)
p10 = pw.plot()
p10.setPen((0,100,100),width=3)
#p11 is the distance
p11 = pw.plot()
p11.setPen((255,255,255),width=3)
p12 = pw.plot()
p12.setPen((255,0,255),width=5)
p13 = pw.plot()
p13.setPen((0,255,255),width=3)

## Add in some extra graphics
rect = QtGui.QGraphicsRectItem(QtCore.QRectF(0, 0, 1, 5e-11))
rect.setPen(QtGui.QPen(QtGui.QColor(100, 200, 100)))
pw.addItem(rect)

pw.setLabel('left', 'Distance', units='cm')
pw.setLabel('right', 'Charge data', units='milivolt/10')
pw.setLabel('bottom', 'Pixels', units='')
pw.setXRange(0, 272)
pw.setYRange(0, 300)

# def rand(n):
#     data = np.random.random(n)
#     data[int(n*0.1):int(n*0.13)] += .5
#     data[int(n*0.18)] += 2
#     data[int(n*0.1):int(n*0.13)] *= 5
#     data[int(n*0.18)] *= 20
#     data *= 1e-12
#     return data, np.arange(n, n+len(data)) / float(n)


# def updateData():
#     yd, xd = rand(10000)
#     p1.setData(y=yd, x=xd)


def clicked():
    print("curve clicked")

def cluster_dist(data,eps_in, min_sample):
    #calculate the distance points..
    #the data from 1 to 256 is a different axis..
    points = np.zeros((256,2))
    angle_min = math.atan(2*focus)
    angle_increment = (PI - 2*angle_min)/256
    for i in range(0,256):
        distance = 0
        if data.distances[i+8]>0:
            distance = data.distances[i+8] + 10  # set a offset to 0...
        points[i][0] = math.cos(PI-angle_min-angle_increment*i)*distance
        points[i][1] = math.sin(PI-angle_min-angle_increment*i)*distance
    #rospy.loginfo("x is %f, y is %f", points[125][0], points[125][1])
    #the parameters need to be carefully chosen, eps is the distance, in cm...
    #min_samples is the points that to be considered as a cluster....
    db = DBSCAN(eps=eps_in, min_samples=min_sample).fit(points)
    labels = db.labels_

    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
#    print('Estimated number of clusters: %d' % n_clusters_)
    return labels, n_clusters_

def callback(data):
#    rospy.loginfo("%f", data.distances[100])
    data_backup = data
    data_backup.distances = list(data_backup.distances)
    data_backup.distances[128] = np.asarray(data_backup.distances[127])
    data_backup.distances[167] = np.asarray(data_backup.distances[166])
    p13.setData(data_backup.distances)
    data.distances = list(data.distances)
    for i in range(0, 16):
        data.distances[i] = np.asarray(data.distances[i]) * 0
    for i in range(245,271):
        data.distances[i] = np.asarray(data.distances[i]) * 0
    data.distances[17] = np.asarray(data.distances[19])
    data.distances[16] = np.asarray(data.distances[19])
    data.distances[176] = np.asarray(data.distances[177])
    data.distances[175] = np.asarray(data.distances[174])
    data.distances[128] = np.asarray(data.distances[127])
    data.distances[167] = np.asarray(data.distances[166])
    
    if enabelDBSCAN:
        labels, n_clusters = cluster_dist(data,8,10)
        ## then deal with each label... remove the edge
        index_b = 0
        index_e = 255
        for i in range(0,255):
            if labels[i] != labels[i+1] and index_b == 0 and labels[i+1] != -1:
                index_b = i
            elif labels[i] != labels[i+1] and index_b != 0 and i>index_b+5:
                index_e = i
            ##assign the mid...
            if index_b > 0 and index_e <255:
                #found one label
                mid = (index_b + index_e)/2
                #print('mid is: %d' % mid)
                #labeldata[mid+8] = 20
                #data.distances[mid+8] = 0
                for j in (index_b,0):
                    if labels[j] == -1:
                        data.distances[j+8]=0
                        data.distances[j+7]=0
                        data.distances[j+6]=0
                        data.distances[j+5]=0
                    else:
                        break
                data.distances[index_b+8] = 0
                data.distances[index_b+1+8] = 0
                data.distances[index_b+2+8] = 0
                data.distances[index_e+8] = 0
                data.distances[index_e-1+8] = 0
                data.distances[index_e-2+8] = 0
                for j in range(index_b,mid):
                    if data.distances[j+8] != 0 and (index_e - index_b)<100 and data.distances[j+8] < 50:
                        data.distances[j+8] = (data.distances[j+8] + data.distances[2*mid-j+8])/2
                        data.distances[2*mid-j+8] = data.distances[j+8]
                index_b = i
                index_e = 255


        #second time DBSCAN
        labels, n_clusters = cluster_dist(data,2.5,5)
        labeldata = list(data.distances)  #create a buffer to hold the label..
        data.reflectance = list(data.reflectance)
        # second DBSCAN find the centers...
        index_b = 0
        index_e = 255
        num = 0
        for i in range(0,255):
            if labels[i] != labels[i+1] and index_b == 0 and labels[i+1] != -1:
                index_b = i
            elif labels[i] != labels[i+1] and index_b != 0 and i>index_b+4:
                index_e = i
            ##assign the mid...
            if index_b > 0 and index_e <255:
                #found one label
                mid = (index_b + index_e)/2
                if data.distances[mid+8] != 0:
                    #print('mid is: %d' % mid)
                    #data.distances[mid+8] = 0
                    num = num + 1
                    data.reflectance[num] = mid + 8
                    data.reflectance[272-num] = index_e - index_b
                #labeldata[mid+8] = 20
                index_b = i
                index_e = 255
        #assign the first 8 reflectance data to be the num
        data.reflectance[0] = num
        #publish the label data
        for i in range(0,256):
            data.reflectance[i+8] = labels[i]
        labelpub = rospy.Publisher("/label_data_"+laser_name,JskLaser,queue_size=1)
        labelpub.publish(data)
        for i in range(0,256):
            for j in range(0,n_clusters):
                if labels[i] == -1:
                    labels[i] = labels[i-1]
                    labeldata[i+8] = (labels[i-1]+2)*50
                else:
                    labeldata[i+8] = (labels[i]+2)*50

        p12.setData(labeldata)


    if not normalshow:
        for i in range(0,136):
            tmp = data.distances[i]
            data.distances[i] = np.asarray(data.distances[271-i])
            data.distances[271-i] = np.asarray(tmp)
    p11.setData(data.distances)

def rawdatacallback(data):

    mw.show()
    for j in range(0, 2):
        data_a = []
        data_b = []
        if normalshow:
            for i in range(0, 272):
                data_a.append(data.data_a[j*272+i])
                data_b.append(data.data_b[j*272+i])
        else:
            for i in range(271, -1, -1):
                data_ap.append(data.data_a[j*272+i])
                data_b.append(data.data_b[j*272+i])

        if rawdataplot:
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

#        mw.show()

def logger():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    global laser_name
    global laser_link
    rospy.init_node('laser_logger')

    laser_name = rospy.get_param('~laser_name',laser_name)
    laser_link = rospy.get_param('~laser_link',laser_link)

    rospy.Subscriber("/laser_data_"+laser_name, JskLaser, callback)
    rospy.Subscriber("/laserraw_data_"+laser_name, JskLaserRaw, rawdatacallback)
    # spin() simply keeps python from exiting until this node is stopped


if __name__ == '__main__':
    logger()
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
        rospy.spin()

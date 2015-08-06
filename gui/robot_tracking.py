from __future__ import print_function, division

import cv2, sys, getopt, urllib, operator
import numpy as np
import cv2.cv as cv
from PIL import Image
from PIL.ImageQt import ImageQt

import io
import os
import sys
import threading

try:
    # Python2
    from urllib2 import urlopen
except ImportError:
    # Python3
    from urllib.request import urlopen

from matplotlib.backends import qt4_compat

use_pyside = qt4_compat.QT_API == qt4_compat.QT_API_PYSIDE
if use_pyside:
    from PySide import QtGui, QtCore
else:
    from PyQt4 import QtGui, QtCore

from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from marker_calibration import MarkerSet

class MyMplCanvas(FigureCanvas):
    """Ultimately, this is a QWidget (as well as a FigureCanvasAgg, etc.)."""
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        # We want the axes cleared every time plot() is called
        self.axes.hold(False)

        self.compute_initial_figure()

        #
        FigureCanvas.__init__(self, fig)
        self.setParent(parent)

        FigureCanvas.setSizePolicy(self,
                                   QtGui.QSizePolicy.Expanding,
                                   QtGui.QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)

    def compute_initial_figure(self):
        pass


class MyDynamicMplCanvas(MyMplCanvas):
    """A canvas that updates itself every second with a new plot."""
    def __init__(self, *args, **kwargs):
        MyMplCanvas.__init__(self, *args, **kwargs)

    def compute_initial_figure(self):
        self.axes.plot([0], [0], 'x')

    def update_figure(self, markers):

        markers.plot(axes=self.axes, marker='o', labels=True)
        self.draw()


''' Get the current webcam image from IP-stream '''
def get_image(url):
    img = ''
    not_found = 1
    counter = 1
    bytes=''

    # open stream with 2 sec timeout
    stream = urlopen(url, None, 2)

    # get all the bytes
    while not_found:
        counter = counter+1

        bytes+=stream.read(1024)
        a = bytes.find('\xff\xd8')
        b = bytes.find('\xff\xd9')
        # New image found
        if a!=-1 and b!=-1:
            jpg = bytes[a:b+2]
            bytes = bytes[b+2:]
            i = cv2.imdecode(np.fromstring(jpg,dtype=np.uint8),cv2.CV_LOAD_IMAGE_COLOR)
            not_found=0
            return i

        if counter > 2000:
            print("Timeout while loading stream")
            raise ValueError('Timeout')


def ndarray2pixmap(img, size=None):
    """ Converts an ndarray to a QPixmap """
    pil_image = Image.fromarray(img)
    if size is not None:
        pil_image.thumbnail(size, Image.ANTIALIAS)
    qimg = QtGui.QImage(ImageQt(pil_image)).rgbSwapped()
    return QtGui.QPixmap.fromImage(qimg)


class CalibrationWidget(QtGui.QWidget):

    def __init__(self):
        super(CalibrationWidget, self).__init__()

        self.initUI()

    def initUI(self):

        w = 640*2
        h = 480*2

        self.cam_w = 640
        self.cam_h = 480
        self.cam_pos = [(640, 0), (640, 480)]
        self.cam_refresh = 1.

        self.fig_w = 640
        self.fig_h = 480
        self.fig_pos = (0, 480)

        # the marker set object
        self.markers = MarkerSet(m=4, diameter=0.04, dim=2)

        self.cam_url = ['http://172.16.156.' + str(n) + ':8080/?action=stream' for n in [139, 141]]
        #self.cam_url = ['http://213.221.142.195:8888/cgi-bin/faststream.jpg?stream=full&fps=0', 'http://213.221.150.11/cgi-bin/faststream.jpg?stream=full&fps=0']

        # Create all the text field to input distances
        d_field_w = 50 # width of the text field

        # markers on camera display
        self.cam_mark = [None,None]
        for i in xrange(2):
            self.cam_mark[i] = []

        # add the background image
        bg_file = './gui/assets/marker_pos.png'
        bg = QtGui.QPixmap(bg_file)
        label = QtGui.QLabel(self)
        label.setPixmap(bg)

        # Overlay the text edit fields
        dist_labels = ['W4','W1','N4','d14','S1','d24','d13',
                'd34','d12','N3','d23','S2','E3','E2',]
        dist_fields_pos = [(55,90), (55,370),
                (170, 50), (100, 230), (170, 410),
                (240, 155), (240, 305),
                (295, 90), (295, 370),
                (420, 50), (490, 230), (420, 410),
                (535, 90), (535, 370)
                ]
        all_fields = dict(zip(dist_labels, dist_fields_pos))

        self.dist_labels = ['d12','d13','d14','d23','d24','d34']
        self.dist_fields = [QtGui.QLineEdit(self) for i in xrange(len(self.dist_labels))]
        i = 0
        for d,t in zip(self.dist_fields, self.dist_labels):
            d.setFixedWidth(d_field_w)
            t = all_fields[t]
            d.move(t[0],t[1])
            i += 1

        # Add the button to process everything
        self.button = QtGui.QPushButton('Calibrate', self)
        self.button.clicked.connect(self.calibrate)
        self.button.move(320-self.button.geometry().width()/2, 440)

        # Embbed a matplotlib figure in the lower left quadrant
        self.fig_dpi = 70
        self.figure = MyDynamicMplCanvas(self, width=self.fig_w/self.fig_dpi, height=self.fig_h/self.fig_dpi, dpi=self.fig_dpi)
        self.figure.move(self.fig_pos[0], self.fig_pos[1])

        # Add the camera panels
        self.cam_num = 2
        self.cam_img = [None, None]
        self.camera = [QtGui.QLabel(self) for i in xrange(self.cam_num)]
        for i in xrange(2):
            self.camera[i].move(self.cam_pos[i][0], self.cam_pos[i][1])
            self.camera[i].resize(self.cam_w, self.cam_h)
            self.camera[i].setAlignment(QtCore.Qt.AlignCenter | QtCore.Qt.AlignVCenter)

        self.camera[0].mousePressEvent = self.cam0_mouseEvent
        self.camera[1].mousePressEvent = self.cam1_mouseEvent

        # Setup a timer to update cameras regularly
        self.cam_timer = QtCore.QTimer()
        self.cam_timer.start(self.cam_refresh)
        self.connect(self.cam_timer, QtCore.SIGNAL('timeout()'), self.run_cam_update) 

        # This is the tick timer to check on the cam update thread
        self.cam_updating = False
        self.tick_timer = QtCore.QTimer()
        self.connect(self.tick_timer, QtCore.SIGNAL('timeout()'), self.tick)
            
        # Set geometry and title and stuff
        self.setGeometry(0, 0, w, h)
        self.setWindowTitle('Calibration')    
        self.show()
        self.raise_()

    def cam0_mouseEvent(self, QMouseEvent):

        p = QMouseEvent.pos()
        self.cam_mark[0].append(p)
        self.updateCameras()

    def cam1_mouseEvent(self, QMouseEvent):

        p = QMouseEvent.pos()
        self.cam_mark[1].append(p)
        self.updateCameras()

    def paintCircles(self):

        for i in xrange(self.cam_num):
            pixmap = self.camera[i].pixmap()

            # need to test if pixmap exists or risk a seg fault!!!
            if pixmap is not None:

                # obtain the painter
                paint = QtGui.QPainter(pixmap)

                radx = 5
                rady = 5
                # draw red circles
                paint.setPen(QtCore.Qt.darkRed)
                paint.setBrush(QtCore.Qt.darkRed)
                for k,center in enumerate(self.cam_mark[i]):
                    paint.drawEllipse(center, radx, rady)
                    paint.drawText(center+QtCore.QPoint(1.5*radx,-1.5*rady), str(k+1))

    def run_cam_update(self):
        if not self.cam_updating:
            self.cam_updating = True
            threading.Thread(target=self.getCameras, name="_proc").start()
            self.tick_timer.start()

    def updateCameras(self):
        """ Update the camera labels with new image """

        for i in xrange(self.cam_num):
            if self.cam_img[i] is not None:
                pixmap = ndarray2pixmap(self.cam_img[i], size=(self.cam_w, self.cam_h))
                self.camera[i].setPixmap(pixmap)
                self.camera[i].setText('')
                self.paintCircles()
            else:
                self.camera[i].setText('Couldn''t load camera ' + str(i))

    def getCameras(self):
        """ Download current image from the webcam """

        for i in xrange(self.cam_num):
            try:
                self.cam_img[i] = get_image(self.cam_url[i])
            except:
                self.cam_img[i] = None

        # flag update as finished
        self.cam_updating = False


    def tick(self):
        if not self.cam_updating:
            self.tick_timer.stop()
            self.updateCameras()

    def calibrate(self):

        # read all the text fields
        distances = {}
        all_ok = True
        for label, field in zip(self.dist_labels, self.dist_fields):
            if field.text() != '':
                try:
                    distances[label] = float(field.text())
                except:
                    distances[label] = -1
                    all_ok = False
                    break

        interdistances = ['d12', 'd13', 'd14', 'd23', 'd24', 'd34']
        wall_distances = ['W4','W1','N4','N3','S1','S2','E2','E3']

        if np.array([distances[dp] != -1 for dp in interdistances]).all():

            D = np.zeros((self.markers.m, self.markers.m))

            # create EDM from distances
            diam = self.markers.diameter
            D[0,1] = D[1,0] = distances['d12'] + diam
            D[0,2] = D[2,0] = distances['d13'] + diam
            D[0,3] = D[3,0] = distances['d14'] + diam
            D[1,2] = D[2,1] = distances['d23'] + diam
            D[1,3] = D[3,1] = distances['d24'] + diam
            D[3,2] = D[2,3] = distances['d34'] + diam
            D **= 2

            self.markers.fromEDM(D)
            self.markers.normalize()

            self.figure.update_figure(self.markers)

        else:
            QtGui.QMessageBox.about(self, 'Error','Please, fill distances properly')



if __name__ == '__main__':

    app = QtGui.QApplication(sys.argv)
    win = CalibrationWidget()
    sys.exit(app.exec_())


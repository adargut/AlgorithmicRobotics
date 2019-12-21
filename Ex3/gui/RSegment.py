from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtOpenGL import QGLWidget, QGLFormat, QGL
from PyQt5.QtWidgets import (QApplication, QGraphicsView,
                             QGraphicsPixmapItem, QGraphicsScene, QGraphicsPolygonItem,
                             QGraphicsEllipseItem, QGraphicsLineItem, QOpenGLWidget)
from PyQt5.QtGui import QPainter, QPixmap, QPolygonF, QPen, QVector3D
from PyQt5.QtCore import (QObject, QPointF, QPoint, QRectF,
                          QPropertyAnimation, pyqtProperty, QSequentialAnimationGroup,
                          QParallelAnimationGroup, QPauseAnimation, Qt)
import math

# def angle(x1, y1, x2, y2):
#   y = y2-y1
#   x = x2-x1
#   return math.atan2(y, x)
#
# def length(x1, y1, x2, y2):
#   math.sqrt((x2-x1)**2 + (y2-y1)**2)

class RSegment(QObject):
  def __init__(self, x1, y1, l, a, color, line_width):
    self._point_radius = 3
    self._angle = a
    self._length = l
    self._x1 = x1
    self._y1 = y1
    self._x2 = x1 + self._length*math.cos(self._angle)
    self._y2 = y1 + self._length*math.sin(self._angle)
    self._pos = QVector3D(x1, y1, self._angle)
    super().__init__()
    self.line = QGraphicsLineItem()
    self.rect = QRectF(x1 - self._point_radius, y1 - self._point_radius, 2 * self._point_radius, 2 * self._point_radius)
    self.point = QGraphicsEllipseItem()
    self.point.setRect(self.rect)
    self.line.setLine(self._x1, self._y1, self._x2, self._y2)
    pen = QPen()
    pen.setWidthF(line_width)
    pen.setColor(color)
    self.line.setPen(pen)
    self.point.setPen(pen)

  def x(self):
    return self._x1

  def y(self):
    return self._y1

  def angle(self):
    return self._angle

  @pyqtProperty(QVector3D)
  def pos(self):
    return self._pos

  @pos.setter
  def pos(self, value):
    self._pos = value
    self._x1 = self._pos.x()
    self._y1 = self._pos.y()
    self._angle = value.z()
    self._x2 = self._x1 + self._length*math.cos(self._angle)
    self._y2 = self._y1 + self._length*math.sin(self._angle)
    self.line.setLine(self._x1, self._y1, self._x2, self._y2)
    self.rect = QRectF(self._x1-self._point_radius,self._y1-self._point_radius, 2 * self._point_radius, 2 * self._point_radius)
    self.point.setRect(self.rect)
    #print(self._angle)

  # @pyqtProperty(float)
  # def angle(self):
  #   return self._angle
  #
  # @angle.setter
  # def angle(self, value):
  #   self._angle = value
  #   self._x2 = self.x_1 + self._length*math.cos(self._angle)
  #   self._y2 = self.x_2 + self._length*math.sin(self._angle)
  #   self.line.setLine(self._x1, self._y1, self._x2, self._y2)

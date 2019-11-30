from arr2_epec_seg_ex import *
import conversions
from gui.gui import GUI, QtCore, QtGui, QtWidgets, Qt

def plus(x, y):
  return x+y

def set_up_arrangement():
  #creating an arrangement
  arr0 = Arrangement_2()
  p0 = Point_2(0, 100)
  p1 = Point_2(100, 200)
  p2 = Point_2(200, 100)
  p3 = Point_2(100, 0)
  c0 = Curve_2(Segment_2(p0, p1))
  c1 = Curve_2(Segment_2(p1, p2))
  c2 = Curve_2(Segment_2(p2, p3))
  c3 = Curve_2(Segment_2(p3, p0))

  insert(arr0, [c0, c1, c2, c3])

  lst = []
  for face in arr0.faces():
    face.set_data(0)
    if not face.is_unbounded():
      face.set_data(1)

  #creating another arrangement
  arr1 = Arrangement_2()
  p0 = Point_2(50, 100)
  p1 = Point_2(150, 200)
  p2 = Point_2(250, 100)
  p3 = Point_2(150, 0)
  c0 = Curve_2(Segment_2(p0, p1))
  c1 = Curve_2(Segment_2(p1, p2))
  c2 = Curve_2(Segment_2(p2, p3))
  c3 = Curve_2(Segment_2(p3, p0))

  insert(arr1, [c0, c1, c2, c3])

  lst = []
  for face in arr1.faces():
    face.set_data(0)
    if not face.is_unbounded():
      face.set_data(2)

  #computing the arrangement resulting from the overlay of arr0 and arr1
  #the Arr_face_overlay_traits define how to handle the face data in the resulting arrangement
  traits = Arr_face_overlay_traits(plus)
  arr_overlay = Arrangement_2()
  overlay(arr0, arr1, arr_overlay, traits)

  #splitting a preexisting face in arr_overlay
  #need to take care of the face data
  p1 = Point_2(200, 100)
  p2 = Point_2(250, 100)
  c = Curve_2(Segment_2(p1, p2))
  he = insert_non_intersecting_curve(arr_overlay, c)
  face_data = he.face().data() if he.face().data() != None else he.twin().face().data()
  he.face().set_data(face_data)
  he.twin().face().set_data(face_data)

  for face in arr_overlay.faces():
    print(face.data())
    lst = []
    if not face.is_unbounded():
      for edge in face.outer_ccb():
        lst.append(edge.source().point())
      lst = [conversions.point_2_to_xy(p) for p in lst]
      if face.data() == 3:
        gui.add_polygon(lst, Qt.darkMagenta)
      if face.data() == 2:
        gui.add_polygon(lst, Qt.blue)
      if face.data() == 1:
        gui.add_polygon(lst, Qt.red)

if __name__ == "__main__":
  import sys
  app = QtWidgets.QApplication(sys.argv)
  gui = GUI()
  gui.set_logic(0, set_up_arrangement)
  gui.set_button_text(0, "Overlay")
  gui.MainWindow.show()
  sys.exit(app.exec_())

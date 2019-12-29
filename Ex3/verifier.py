from arr2_epec_seg_ex import *
import math
import importlib
from gui.gui import GUI, QtCore, QtGui, QtWidgets, Qt, QPointF
import read_input
from conversions import point_2_to_xy, tuples_list_to_polygon_2, polygon_2_to_tuples_list, path_point_to_xyzd
import sample_polygons

class Polygons_scene():
  def __init__(self):
    self.origin = None
    self.length = None
    self.obstacles = []
    self.gui_robot = None
    self.gui_obstacles = []
    self.destination = None
    self.gui_destination = None
    self.path = []

  def draw_scene(self):
    gui.clear_scene()
    if(self.origin != None):
      x1 = FT(Gmpq(self.origin[0])).to_double()
      y1 = FT(Gmpq(self.origin[1])).to_double()
      a = FT(Gmpq(self.origin[2])).to_double()
      l = self.length.to_double()
      self.gui_robot = gui.add_segment(x1, y1, l, a, Qt.red)
    for obstacle in self.obstacles:
      self.gui_obstacles = []
      self.gui_obstacles.append(gui.add_polygon(obstacle, Qt.darkGray))
    if(self.destination != None):
      #self.gui_destination = gui.add_disc(4, int(self.destination[0]), int(self.destination[1]), Qt.green)
      da = FT(Gmpq(self.destination[2])).to_double()
      self.gui_destination = gui.add_segment(int(self.destination[0]), int(self.destination[1]), l, da, Qt.green)

  def load_scene(self, filename):
    scene = read_input.read_scene(filename)
    self.length = FT(Gmpq(scene[0]))
    self.origin = scene[1]
    destination = " ".join(scene[2])
    gui.set_field(2, destination)
    self.set_destination(destination.split(" "))
    self.obstacles = []
    for i in range(3, len(scene)):
      self.obstacles.append(scene[i])
    gui.clear_queue()
    self.draw_scene()


  def save_scene(self, filename):
    def polygon_to_string(polygon):
      s = " "
      temp = [str(i) for i in polygon]
      line = str(len(polygon)) + " " + s.join(temp)
      line = line.replace("(", "")
      line = line.replace(")", "")
      line = line.replace(",", "")
      return line

    file = open(filename, 'w')
    #print(self.destination)
    line = str(self.destination[0]) + " " + str(self.destination[1])
    file.write(line + '\n')
    line = polygon_to_string(self.origin)
    file.write(line)
    if len(self.obstacles) > 0: file.write('\n')
    for i in range(len(self.obstacles)):
      line = polygon_to_string(self.obstacles[i])
      if i != len(self.obstacles) - 1: line = line + '\n'
      file.write(line)
    file.close()

  def set_destination(self, destination):
    dx1 = destination[0]
    dy1 = destination[1]
    da = destination[2]
    self.destination = (dx1, dy1, da)
    if self.gui_destination != None:
      gui.remove_item(self.gui_destination.line)
      gui.remove_item(self.gui_destination.point)
      #gui.add_disc(4, int(dx1), int(dy1), Qt.green)
    l = self.length.to_double()
    da = FT(Gmpq(self.destination[2])).to_double()
    self.gui_destination = gui.add_segment(int(self.destination[0]), int(self.destination[1]), l, da, Qt.green)

  def set_up_animation(self):
    self.draw_scene()
    if len(self.path) == 0:
      return
    if len(self.path) == 1:
      start = path_point_to_xyzd(self.path[0])
      anim = gui.general_segment_animation(self.gui_robot, start[0], start[1], start[2], *start)
      gui.queue_animation(anim)
    else:
      for i in range(len(self.path) - 1):
        start = path_point_to_xyzd(self.path[i])
        end = path_point_to_xyzd(self.path[i+1])
        anim = gui.general_segment_animation(self.gui_robot, start[0], start[1], start[2], *end)
        gui.queue_animation(anim)

  def is_path_valid(self):
    if self.path == None: return False
    if len(self.path) == 0: return False

    obstacle_polygons = []
    for obs in self.obstacles:
      p = tuples_list_to_polygon_2(obs)
      obstacle_polygons.append(p)

    path_polygons = []
    for i in range(len(self.path) - 1):
      sample_polygons.sample_polygons(self.path[i], self.path[i+1], self.length, path_polygons)

    path_set = Polygon_set_2()
    path_set.join_polygons(path_polygons)
    obstacles_set = Polygon_set_2()
    obstacles_set.join_polygons(obstacle_polygons)

    lst = []
    path_set.polygons_with_holes(lst)
    for pwh in lst:
      p = pwh.outer_boundary()
      lst = polygon_2_to_tuples_list(p)
      gui.add_polygon(lst, Qt.lightGray).polygon.setZValue(-3)
      for p in pwh.holes():
        lst = polygon_2_to_tuples_list(p)
        gui.add_polygon(lst, Qt.white).polygon.setZValue(-2)


    # check that the origin matches the first point in the path
    x1 = FT(Gmpq(self.origin[0]))
    y1 = FT(Gmpq(self.origin[1]))
    a = FT(Gmpq(self.origin[2]))

    dx1 = FT(Gmpq(self.destination[0]))
    dy1 = FT(Gmpq(self.destination[1]))
    da = FT(Gmpq(self.destination[2]))
    check0 = True if (x1 == self.path[0][0] and y1 == self.path[0][1] and a == self.path[0][2]) else False
    # check that the destination matches the last point in the path
    check1 = True if (dx1 == self.path[-1][0] and dy1 == self.path[-1][1] and da == self.path[-1][2]) else False
    #check that there are no collisions
    check2 = True if not path_set.do_intersect(obstacles_set) else False
    #res = (check0 and check1 and check2)
    res = (check0 and check1 and check2)
    print("Valid path: ", res)
    if check0 == False:
      print("Origin mismatch")
      print(x1, y1, a)
      print(self.path[0][0], self.path[0][1], self.path[0][2])
    if check1 == False:
      print("Destination mismatch")
      print(dx1, dy1, da, "with type:", type(dx1), type(dy2), type(da))
      print(self.path[-1][0], self.path[-1][1], self.path[-1][2], "with type:", type(self.path[-1][0]),
            type(self.path[-1][0]), type(self.path[-1][2]))
    if check2 == False:
      print("Movement along path intersects with obstacles")
    return res

def set_up_scene():
  gui.clear_scene()
  ps.destination = None
  ps.gui_destination = None
  scene_file = gui.get_field(0)
  ps.load_scene(scene_file)
  print("loaded scene from", scene_file)


def save_scene():
  # scene_file = gui.get_field(1)
  # ps.save_scene(scene_file)
  # print("saved scene to", scene_file)
  pass

def generate_path():
  ps.path = []
  gui.clear_queue()
  path_name = gui.get_field(3)
  gp = importlib.import_module(path_name)
  gp.generate_path(ps.path, ps.length, ps.obstacles, ps.origin, ps.destination)
  # for p in points:
  #   gui.add_segment(p.x().to_double(), p.y().to_double(), ps.length.to_double(), p.z().to_double(), Qt.blue)

  print("Generated path via", path_name + ".generate_path")
  ps.set_up_animation()

def load_path():
  ps.path = []
  gui.clear_queue()
  path_name = gui.get_field(4)
  read_input.load_path(ps.path, path_name)
  print("Loaded path from", path_name)
  ps.set_up_animation()

def save_path():
  path_name = gui.get_field(5)
  read_input.save_path(ps.path, path_name)
  print("Path saved to", path_name)
def is_path_valid():
  ps.is_path_valid()

def set_destination():
  destination = gui.get_field(2).split(" ")
  ps.set_destination(destination)

def animate_path():
  gui.play_queue()

if __name__ == "__main__":
  import sys
  app = QtWidgets.QApplication(sys.argv)
  gui = GUI()
  ps = Polygons_scene()
  gui.set_program_name("Polygon Scene Motion Planning")
  gui.set_field(0, "scene1.txt")
  # gui.set_field(1, "output_scene0.txt")
  gui.set_field(3, "ex32")
  gui.set_field(4, "path0.txt")
  gui.set_field(5, "path_out.txt")
  gui.set_logic(0, set_up_scene)
  gui.set_button_text(0, "Load scene")
  # gui.set_logic(1, save_scene)
  gui.set_button_text(1, "Unused")
  gui.set_logic(2 , set_destination)
  gui.set_button_text(2, "Set destination")
  gui.set_logic(3, generate_path)
  gui.set_button_text(3, "Generate path")
  gui.set_logic(4, load_path)
  gui.set_button_text(4, "Load path")
  gui.set_logic(5, save_path)
  gui.set_button_text(5, "Save path")
  gui.set_logic(6, animate_path)
  gui.set_button_text(6, "Animate movement along path")
  gui.set_logic(7, is_path_valid)
  gui.set_button_text(7, "Check path validity")
  gui.MainWindow.show()
  sys.exit(app.exec_())
from arr2_epec_seg_ex import *
import math
import importlib
from gui.gui import GUI, QtCore, QtGui, QtWidgets, Qt, QPointF
import read_input
from conversions import point_2_to_xy, tuples_list_to_polygon_2, polygon_2_to_tuples_list
import ms_polygon_segment

class Polygons_scene():
  def __init__(self):
    self.robot = None
    self.obstacles = []
    self.gui_robot = None
    self.gui_obstacles = []
    self.destination = None
    self.gui_destination = None
    self.path = []

  def draw_scene(self):
    gui.clear_scene()
    if(self.robot != None):
      self.gui_robot = gui.add_polygon(self.robot, Qt.yellow)
    for obstacle in self.obstacles:
      self.gui_obstacles = []
      self.gui_obstacles.append(gui.add_polygon(obstacle, Qt.darkGray))
    if(self.destination != None):
      self.gui_destination = gui.add_disc(4, *self.destination, Qt.green)

  def load_scene(self, filename):
    bounding_rect = [(0,0), (0, 2000), (2000, 2000), (2000, 0)]
    gui.add_polygon(bounding_rect, Qt.transparent)
    scene = read_input.read_polygon_scene(filename)
    gui.set_field(2, " ".join(scene[0]))
    self.robot = scene[1]
    self.obstacles = []
    for i in range(2, len(scene)):
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
    print(self.destination)
    line = str(self.destination[0]) + " " + str(self.destination[1])
    file.write(line + '\n')
    line = polygon_to_string(self.robot)
    file.write(line)
    if len(self.obstacles) > 0: file.write('\n')
    for i in range(len(self.obstacles)):
      line = polygon_to_string(self.obstacles[i])
      if i != len(self.obstacles) - 1: line = line + '\n'
      file.write(line)
    file.close()

  def set_destination(self, destination):
    self.destination = destination
    if self.gui_destination == None:
      self.gui_destination = gui.add_disc(4, *destination, Qt.green)
    else:
      self.gui_destination.pos = QPointF(*destination)

  def set_up_animation(self):
    self.draw_scene()
    if len(self.path) == 0:
      return
    if len(self.path) == 1:
      start = point_2_to_xy(self.path[0])
      anim = gui.linear_translation_animation(self.gui_robot, *start, *start)
      gui.queue_animation(anim)
    else:
      for i in range(len(self.path) - 1):
        start = point_2_to_xy(self.path[i])
        end = point_2_to_xy(self.path[i+1])
        s = gui.add_segment(*start, *end, Qt.red)
        s.line.setZValue(-1)
        anim = gui.linear_translation_animation(self.gui_robot, *start, *end)
        gui.queue_animation(anim)

  def is_path_valid(self):
    if self.path == None: return False
    if len(self.path) == 0: return False
    robot_polygon = tuples_list_to_polygon_2(self.robot)
    path_polygons = []
    if len(self.path) > 1:
      for i in range(len(self.path) - 1):
        source = self.path[i]
        target = self.path[i+1]
        s = Segment_2(source, target)
        pwh = ms_polygon_segment.minkowski_sum_polygon_segment(robot_polygon, s)
        path_polygons.append(pwh)

    obstacle_polygons = []
    for obs in self.obstacles:
      p = tuples_list_to_polygon_2(obs)
      obstacle_polygons.append(p)

    path_set = Polygon_set_2()
    path_set.join_polygons_with_holes(path_polygons)
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
    check0 = True if self.robot[0] == point_2_to_xy(self.path[0]) else False
    # check that the destination matches the last point in the path
    check1 = True if self.destination == point_2_to_xy(self.path[-1]) else False
    #check that there are no collisions
    check2 = True if not path_set.do_intersect(obstacles_set) else False
    res = (check0 and check1 and check2)
    print("Valid path: ", res)
    if check0 == False:
      print("Origin mismatch")
    if check1 == False:
      print("Destination mismatch")
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
  scene_file = gui.get_field(1)
  ps.save_scene(scene_file)
  print("saved scene to", scene_file)

def generate_path():
  ps.path = []
  gui.clear_queue()
  path_name = gui.get_field(3)
  gp = importlib.import_module(path_name)
  gp.generate_path(ps.path, ps.robot, ps.obstacles, ps.destination)
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
  with open(path_name, 'w') as f:
    for i in range(len(ps.path)):
      p = ps.path[i]
      x = p.x().exact()
      y = p.y().exact()
      f.write(str(x.numerator()) + "/" + str(x.denominator()) + " " + str(y.numerator()) + "/" + str(y.denominator()))
      if i != len(ps.path)-1: f.write('\n')
  print("Path saved to", path_name)
def is_path_valid():
  ps.is_path_valid()

def set_destination():
  s = gui.get_field(2).split(" ")
  destination = (int(s[0]), int(s[1]))
  ps.set_destination(destination)

def animate_path():
  gui.play_queue()

if __name__ == "__main__":
  import sys
  app = QtWidgets.QApplication(sys.argv)
  gui = GUI()
  ps = Polygons_scene()
  gui.set_program_name("Polygon Scene Motion Planning")
  gui.set_field(0, "polygon_scene0.txt")
  gui.set_field(1, "output_scene0.txt")
  gui.set_field(3, "ex23")
  gui.set_field(4, "path0.txt")
  gui.set_field(5, "path_out.txt")
  gui.set_logic(0, set_up_scene)
  gui.set_button_text(0, "Load scene")
  gui.set_logic(1, save_scene)
  gui.set_button_text(1, "Save scene")
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
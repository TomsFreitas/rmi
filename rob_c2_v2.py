import math
import sys
import time

from croblink import *
from math import *
import xml.etree.ElementTree as ET
import collections
from scipy.signal.signaltools import wiener
import pprint
from enum import Enum

pp = pprint.PrettyPrinter(indent=10)

CELLROWS = 7
CELLCOLS = 14


class orientation(Enum):
    Right = 0
    Up = 1
    Left = 2
    Down = 3


class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.current_measures = []
        self.state = "map"
        self.possible_headings = [0, 90, 180, 270]
        self.supposed_x = 0
        self.supposed_y = 0
        self.orientation = orientation.Right
        self.first_boot = True
        self.moving = False
        self.rotating = False
        self.mymap = [[' '] * (CELLCOLS * 4 - 1) for i in range(CELLROWS * 4 - 1)]
        self.map_location_x = 27
        self.map_location_y = 13
        self.mymap[13][27] = "I"
        self.visited = set()
        self.not_visited = set()
        self.target_locked = None
        self.target_location = None
        self.graph = {}
        self.path = None
        self.flag = False
        self.need_to_rotate = False

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

        while True:
            self.readSensors()
            if self.first_boot:
                self.supposed_x = self.measures.x
                self.supposed_y = self.measures.y
                self.first_boot = False

            if self.measures.compass < 0:
                self.measures.compass = self.measures.compass + 360

            self.current_measures = [self.measures.irSensor[0], self.measures.irSensor[1],
                                     self.measures.irSensor[2], self.measures.irSensor[3]]

            if self.measures.endLed:
                print(self.rob_name + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed == True:
                    state = 'wait'

                if self.measures.ground == 0:
                    self.setVisitingLed(True);
                self.wander()
            elif state == 'wait':
                self.setReturningLed(True)
                if self.measures.visitingLed == True:
                    self.setVisitingLed(False)
                if self.measures.returningLed == True:
                    state = 'return'
                self.driveMotors(0.0, 0.0)
            elif state == 'return':
                if self.measures.visitingLed == True:
                    self.setVisitingLed(False)
                if self.measures.returningLed == True:
                    self.setReturningLed(False)
                self.wander()

    def wander(self):
        if self.state == "map":
            print("I am mapping")
            self.map()
            self.state = "go"

        elif self.state == "go_with_purpose":
            if self.target_location is None:
                print("where_to_advanced")
                self.where_to_advanced()
            if not self.need_to_rotate:
                temp_orientation = self.where_to_basic(self.path[0])
                if temp_orientation != self.orientation:
                    self.target_locked = temp_orientation
                    self.need_to_rotate = True
            if self.need_to_rotate:
                self.rotate()
                return
            self.go()





        elif self.state == "go":
            print("I am going")
            if self.next():
                self.go()
            else:
                print("No path forward")
                self.where_to_basic()

        elif self.state == "rotate":
            print("rotate")
            self.rotate()

        elif self.state == "stop":
            print("I am stopping")
            self.stop()
            if self.target_location == (self.map_location_x, self.map_location_y):
                self.path = None
                self.target_location = None
                self.target_locked = None
            if self.path is not None:
                self.state = "go_with_purpose"
            else:
                self.state = "map"

    def where_to_basic(self, target=None):
        # Are there any free, not visited, spaces adjacent to me
        level = 2

        possible_places = [((self.map_location_x - level, self.map_location_y), orientation.Left),
                           ((self.map_location_x + level, self.map_location_y), orientation.Right),
                           ((self.map_location_x, self.map_location_y - level), orientation.Up),
                           ((self.map_location_x, self.map_location_y + level), orientation.Down)]
        for adjacent in possible_places:
            try:
                if target is not None:
                    if adjacent[0] == target:
                        return adjacent[1]
                if adjacent[0] not in self.visited and self.mymap[adjacent[0][1]][adjacent[0][0]] == 'F':
                    self.target_locked = adjacent[1]
                    self.state = "rotate"
                    return
            except IndexError as e:
                continue
        if self.target_locked is None:
            self.state = "go_with_purpose"

    def where_to_advanced(self):
        min = 99

        for key in self.graph.keys():
            self.graph[key] = list(set(self.graph[key]))

        self.not_visited = self.not_visited - self.visited
        #print(self.not_visited)
        #print(self.graph)
        for node in self.not_visited:
            self.graph.setdefault(node, [])

        for node in self.not_visited:
            path = self.calculate_path(self.graph, (self.map_location_x, self.map_location_y), node)
            if len(path) < min:
                self.target_location = node
                self.path = path[1:]
                min = len(path)
        print("I am travelling to {} using path {}".format(self.target_location, self.path))

    def calculate_path(self, graph, start, end, path=[]):
        path = path + [start]
        if start == end:
            return path
        shortest = None
        for node in graph[start]:
            if node not in path:
                newpath = self.calculate_path(graph, node, end, path)
                if newpath:
                    if not shortest or len(newpath) < len(shortest):
                        shortest = newpath

        return shortest

    def rotate(self):
        target_heading = self.possible_headings[self.target_locked.value]
        #print(target_heading, self.measures.compass, self.target_locked)
        factor = self.get_rotation_factor(soft_rotation=False, target=target_heading)
        #print(factor)
        if not (target_heading - 1 <= self.measures.compass <= target_heading + 1):
            self.move(0, 0.5, 0, factor)
        else:
            if self.target_location is None:
                self.state = "go"
            else:
                self.state = "go_with_purpose"
                self.need_to_rotate = False
            self.orientation = self.target_locked
            self.target_locked = None

    def go(self):
        factor = self.get_rotation_factor()
        if self.orientation == orientation.Right:
            if self.measures.x < self.supposed_x + 1.7:
                self.move(0.13, 0.1, 0, factor)
            else:
                self.moving = False
                self.supposed_x += 2
                self.map_location_x += 2
                self.state = "stop"
                self.visited.add((self.map_location_x, self.map_location_y))
                self.not_visited.discard((self.map_location_x, self.map_location_y))
                if self.path is not None:
                    self.path = self.path[1:]
                    print(self.path)

        elif self.orientation == orientation.Left:
            if self.measures.x > self.supposed_x - 1.7:
                self.move(0.13, 0.1, 0, factor)
            else:
                self.moving = False
                self.supposed_x -= 2
                self.map_location_x -= 2
                self.state = "stop"
                self.visited.add((self.map_location_x, self.map_location_y))
                self.not_visited.discard((self.map_location_x, self.map_location_y))
                if self.path is not None:
                    self.path = self.path[1:]
                    print(self.path)


        elif self.orientation == orientation.Up:
            if self.measures.y < self.supposed_y + 1.7:
                self.move(0.13, 0.1, 0, factor)
            else:
                self.moving = False
                self.supposed_y += 2
                self.map_location_y -= 2
                self.state = "stop"
                self.visited.add((self.map_location_x, self.map_location_y))
                self.not_visited.discard((self.map_location_x, self.map_location_y))
                if self.path is not None:
                    self.path = self.path[1:]
                    print(self.path)


        elif self.orientation == orientation.Down:
            if self.measures.y > self.supposed_y - 1.7:
                self.move(0.13, 0.1, 0, factor)
            else:
                self.moving = False
                self.supposed_y -= 2
                self.map_location_y += 2
                self.state = "stop"
                self.visited.add((self.map_location_x, self.map_location_y))
                self.not_visited.discard((self.map_location_x, self.map_location_y))
                if self.path is not None:
                    self.path = self.path[1:]
                    print(self.path)


    def get_rotation_factor(self, soft_rotation=True, target=None):
        if soft_rotation:
            supposed_heading = self.possible_headings[self.orientation.value]
        else:
            supposed_heading = target
        real_heading = self.measures.compass
        if real_heading > 180 and supposed_heading == 0:
            diff = 360 - real_heading
        else:
            diff = abs(supposed_heading - real_heading)
        #print(diff)
        if (360 + supposed_heading - real_heading) % 360 > 180:
            #print("clockwise")
            if target is not None:
                return 0.075 * diff
            else:
                return 0.5 * diff
        else:
            #print("counter clockwise")
            if target is not None:
                return -0.075 * diff
            else:
                return -0.5 * diff

    def move(self, linear, k, m, r):
        right_power = linear + (k * (m - r)) / 2
        left_power = linear - (k * (m - r)) / 2
        self.driveMotors(left_power, right_power)

    def stop(self):
        for i in range(0, 5):
            self.driveMotors(0, 0)

    def map(self):
        if self.orientation == orientation.Right:
            if self.current_measures[0] > 1:
                self.mymap[self.map_location_y][self.map_location_x + 1] = "|"
            else:
                self.mymap[self.map_location_y][self.map_location_x + 2] = "F"
                self.graph.setdefault((self.map_location_x, self.map_location_y), []) \
                    .append((self.map_location_x + 2, self.map_location_y))
                self.not_visited.add((self.map_location_x + 2, self.map_location_y))

            if self.current_measures[1] > 1:
                self.mymap[self.map_location_y - 1][self.map_location_x] = "-"
            else:
                self.mymap[self.map_location_y - 2][self.map_location_x] = "F"
                self.graph.setdefault((self.map_location_x, self.map_location_y), []) \
                    .append((self.map_location_x, self.map_location_y - 2))
                self.not_visited.add((self.map_location_x, self.map_location_y - 2))

            if self.current_measures[2] > 1:
                self.mymap[self.map_location_y + 1][self.map_location_x] = "-"
            else:
                self.mymap[self.map_location_y + 2][self.map_location_x] = "F"
                self.graph.setdefault((self.map_location_x, self.map_location_y), []) \
                    .append((self.map_location_x, self.map_location_y + 2))
                self.not_visited.add((self.map_location_x, self.map_location_y + 2))

            if self.current_measures[3] > 1:
                self.mymap[self.map_location_y][self.map_location_x - 1] = "|"
            else:
                self.mymap[self.map_location_y][self.map_location_x - 2] = "F"
                self.graph.setdefault((self.map_location_x, self.map_location_y), []) \
                    .append((self.map_location_x - 2, self.map_location_y))
                self.not_visited.add((self.map_location_x - 2, self.map_location_y))



        elif self.orientation == orientation.Up:
            if self.current_measures[0] > 1:
                self.mymap[self.map_location_y - 1][self.map_location_x] = "-"
            else:
                self.mymap[self.map_location_y - 2][self.map_location_x] = "F"
                self.graph.setdefault((self.map_location_x, self.map_location_y), []) \
                    .append((self.map_location_x, self.map_location_y - 2))
                self.not_visited.add((self.map_location_x, self.map_location_y - 2))

            if self.current_measures[1] > 1:
                self.mymap[self.map_location_y][self.map_location_x - 1] = "|"
            else:
                self.mymap[self.map_location_y][self.map_location_x - 2] = "F"
                self.graph.setdefault((self.map_location_x, self.map_location_y), []) \
                    .append((self.map_location_x - 2, self.map_location_y))
                self.not_visited.add((self.map_location_x - 2, self.map_location_y))

            if self.current_measures[2] > 1:
                self.mymap[self.map_location_y][self.map_location_x + 1] = "|"
            else:
                self.mymap[self.map_location_y][self.map_location_x + 2] = "F"
                self.graph.setdefault((self.map_location_x, self.map_location_y), []) \
                    .append((self.map_location_x + 2, self.map_location_y))
                self.not_visited.add((self.map_location_x + 2, self.map_location_y))

            if self.current_measures[3] > 1:
                self.mymap[self.map_location_y + 1][self.map_location_x] = "-"
            else:
                self.mymap[self.map_location_y + 2][self.map_location_x] = "F"
                self.graph.setdefault((self.map_location_x, self.map_location_y), []) \
                    .append((self.map_location_x, self.map_location_y + 2))
                self.not_visited.add((self.map_location_x, self.map_location_y + 2))


        elif self.orientation == orientation.Down:
            if self.current_measures[0] > 1:
                self.mymap[self.map_location_y + 1][self.map_location_x] = "-"
            else:
                self.mymap[self.map_location_y + 2][self.map_location_x] = "F"
                self.graph.setdefault((self.map_location_x, self.map_location_y), []) \
                    .append((self.map_location_x, self.map_location_y + 2))
                self.not_visited.add((self.map_location_x, self.map_location_y + 2))

            if self.current_measures[1] > 1:
                self.mymap[self.map_location_y][self.map_location_x + 1] = "|"
            else:
                self.mymap[self.map_location_y][self.map_location_x + 2] = "F"
                self.graph.setdefault((self.map_location_x, self.map_location_y), []) \
                    .append((self.map_location_x + 2, self.map_location_y))
                self.not_visited.add((self.map_location_x + 2, self.map_location_y))
                self.not_visited.add((self.map_location_x + 2, self.map_location_y))

            if self.current_measures[2] > 1:
                self.mymap[self.map_location_y][self.map_location_x - 1] = "|"
            else:
                self.mymap[self.map_location_y][self.map_location_x - 2] = "F"
                self.graph.setdefault((self.map_location_x, self.map_location_y), []) \
                    .append((self.map_location_x - 2, self.map_location_y))
                self.not_visited.add((self.map_location_x - 2, self.map_location_y))

            if self.current_measures[3] > 1:
                self.mymap[self.map_location_y - 1][self.map_location_x] = "-"
            else:
                self.mymap[self.map_location_y - 2][self.map_location_x] = "F"
                self.graph.setdefault((self.map_location_x, self.map_location_y), []) \
                    .append((self.map_location_x, self.map_location_y - 2))
                self.not_visited.add((self.map_location_x, self.map_location_y - 2))

        else:
            if self.current_measures[0] > 1:
                self.mymap[self.map_location_y][self.map_location_x - 1] = "|"
            else:
                self.mymap[self.map_location_y][self.map_location_x - 2] = "F"
                self.graph.setdefault((self.map_location_x, self.map_location_y), []) \
                    .append((self.map_location_x - 2, self.map_location_y))
                self.not_visited.add((self.map_location_x - 2, self.map_location_y))

            if self.current_measures[1] > 1:
                self.mymap[self.map_location_y + 1][self.map_location_x] = "-"
            else:
                self.mymap[self.map_location_y + 2][self.map_location_x] = "F"
                self.graph.setdefault((self.map_location_x, self.map_location_y), []) \
                    .append((self.map_location_x, self.map_location_y + 2))
                self.not_visited.add((self.map_location_x, self.map_location_y + 2))

            if self.current_measures[2] > 1:
                self.mymap[self.map_location_y - 1][self.map_location_x] = "-"
            else:
                self.mymap[self.map_location_y - 2][self.map_location_x] = "F"
                self.graph.setdefault((self.map_location_x, self.map_location_y), []) \
                    .append((self.map_location_x, self.map_location_y - 2))
                self.not_visited.add((self.map_location_x, self.map_location_y - 2))

            if self.current_measures[3] > 1:
                self.mymap[self.map_location_y][self.map_location_x + 1] = "|"
            else:
                self.mymap[self.map_location_y][self.map_location_x + 2] = "F"
                self.graph.setdefault((self.map_location_x, self.map_location_y), []) \
                    .append((self.map_location_x + 2, self.map_location_y))
                self.not_visited.add((self.map_location_x + 2, self.map_location_y))

        return

    def next(self):
        if self.orientation == orientation.Right:
            return self.mymap[self.map_location_y][self.map_location_x + 2] == "F" and \
                   self.mymap[self.map_location_y][self.map_location_x + 1] != "|"
        elif self.orientation == orientation.Up:
            return self.mymap[self.map_location_y - 2][self.map_location_x] == "F" and \
                   self.mymap[self.map_location_y - 1][self.map_location_x] != "-"
        elif self.orientation == orientation.Left:
            return self.mymap[self.map_location_y][self.map_location_x - 2] == "F" and \
                   self.mymap[self.map_location_y][self.map_location_x - 1] != "|"
        else:
            return self.mymap[self.map_location_y + 2][self.map_location_x] == "F" and \
                   self.mymap[self.map_location_y + 1][self.map_location_x] != "-"


class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()

        self.labMap = [[' '] * (CELLCOLS * 2 - 1) for i in range(CELLROWS * 2 - 1)]
        i = 1
        for child in root.iter('Row'):
            line = child.attrib['Pattern']
            row = int(child.attrib['Pos'])
            if row % 2 == 0:  # this line defines vertical lines
                for c in range(len(line)):
                    if (c + 1) % 3 == 0:
                        if line[c] == '|':
                            self.labMap[row][(c + 1) // 3 * 2 - 1] = '|'
                        else:
                            None
            else:  # this line defines horizontal lines
                for c in range(len(line)):
                    if c % 3 == 0:
                        if line[c] == '-':
                            self.labMap[row][c // 3 * 2] = '-'
                        else:
                            None

            i = i + 1


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None

for i in range(1, len(sys.argv), 2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob = MyRob(rob_name, pos, [0.0, 90.0, -90.0, 180.0], host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()

    rob.run()

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
        self.visited = []
        self.target_locked = None

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
        print(self.visited)
        if self.state == "map":
            print("I am mapping")
            self.map()
            self.state = "go"

        elif self.state == "go":
            print("I am going")
            if self.next():
                print("There is next available")
                self.go()
            else:
                print("I am rotating")
                if self.target_locked is None:
                    self.where_to()
                self.rotate()

        elif self.state == "stop":
            print("I am stopping")
            self.stop()
            self.state = "map"

    def where_to(self):
        # Are there any free, not visited, spaces adjacent to me
        next = None
        possible_places = [((self.map_location_x-2, self.map_location_y), orientation.Left),((self.map_location_x+2, self.map_location_y),orientation.Right),
                           ((self.map_location_x, self.map_location_y-2),orientation.Up),((self.map_location_x, self.map_location_y+2),orientation.Down)]
        for adjacent in possible_places:
            if adjacent[0] not in self.visited and self.mymap[adjacent[0][1]][adjacent[0][0]] == 'F':
                self.target_locked = adjacent[1]

    def rotate(self):
        print("I am inside rotate")
        target_heading = self.possible_headings[self.target_locked.value]
        print(target_heading,self.measures.compass, self.target_locked)
        factor = self.get_rotation_factor(soft_rotation=False, target=target_heading)
        print(factor)
        if not(target_heading - 1 <= self.measures.compass <= target_heading + 1):
            print("MOVE")
            self.move(0,0.9,0,factor)
        else:
            self.state = "go"
            self.orientation = self.target_locked
            self.target_locked = None





    def go(self):
        factor = self.get_rotation_factor()
        if self.orientation == orientation.Right:
            if self.measures.x < self.supposed_x + 2:
                self.move(0.09, 0.015, 0, factor)
            else:
                self.moving = False
                self.supposed_x += 2
                self.map_location_x += 2
                self.state = "stop"
                self.visited.append((self.map_location_x, self.map_location_y))

        elif self.orientation == orientation.Left:
            if self.measures.x > self.supposed_x - 2:
                self.move(0.09, 0.015, 0, factor)
            else:
                self.moving = False
                self.supposed_x -= 2
                self.map_location_x -= 2
                self.state = "stop"
                self.visited.append((self.map_location_x, self.map_location_y))


        elif self.orientation == orientation.Up:
            if self.measures.y < self.supposed_y + 2:
                self.move(0.09, 0.015, 0, factor)
            else:
                self.moving = False
                self.supposed_y += 2
                self.map_location_y -= 2
                self.state = "stop"
                self.visited.append((self.map_location_x, self.map_location_y))


        elif self.orientation == orientation.Down:
            if self.measures.y > self.supposed_y - 2:
                self.move(0.09, 0.015, 0, factor)
            else:
                self.moving = False
                self.supposed_y -= 2
                self.map_location_y += 2
                self.state = "stop"
                self.visited.append((self.map_location_x, self.map_location_y))

    def get_rotation_factor(self, soft_rotation=True, target=None):
        if soft_rotation:
            supposed_heading = self.possible_headings[self.orientation.value]
        else:
            supposed_heading = target
        real_heading = self.measures.compass
        diff = abs(real_heading - supposed_heading)
        print(diff)
        if (360 + supposed_heading - real_heading) % 360 > 180:
            return +0.001 * diff
        else:
            return -0.001 * diff


    def move(self, linear, k, m, r):
        right_power = linear + (k * (m - r)) / 2
        left_power = linear - (k * (m - r)) / 2
        self.driveMotors(left_power, right_power)

    def stop(self):
        for i in range(0, 40):
            self.driveMotors(0, 0)
        self.driveMotors(0, 0)

    def map(self):
        if self.orientation == orientation.Right:
            if self.current_measures[0] > 1:
                self.mymap[self.map_location_y][self.map_location_x + 1] = "|"
            else:
                self.mymap[self.map_location_y][self.map_location_x + 2] = "F"

            if self.current_measures[1] > 1:
                self.mymap[self.map_location_y + 1][self.map_location_x] = "-"
            else:
                self.mymap[self.map_location_y + 2][self.map_location_x] = "F"

            if self.current_measures[2] > 1:
                self.mymap[self.map_location_y - 1][self.map_location_x] = "-"
            else:
                self.mymap[self.map_location_y - 2][self.map_location_x] = "F"

            if self.current_measures[3] > 1:
                self.mymap[self.map_location_y][self.map_location_x - 1] = "|"
            else:
                self.mymap[self.map_location_y][self.map_location_x - 2] = "F"

        elif self.orientation == orientation.Up:
            if self.current_measures[0] > 1:
                self.mymap[self.map_location_y - 1][self.map_location_x] = "-"
            else:
                self.mymap[self.map_location_y - 2][self.map_location_x] = "F"

            if self.current_measures[1] > 1:
                self.mymap[self.map_location_y][self.map_location_x - 1] = "|"
            else:
                self.mymap[self.map_location_y][self.map_location_x - 2] = "F"

            if self.current_measures[2] > 1:
                self.mymap[self.map_location_y][self.map_location_x + 1] = "|"
            else:
                self.mymap[self.map_location_y][self.map_location_x + 2] = "F"

            if self.current_measures[3] > 1:
                self.mymap[self.map_location_y + 1][self.map_location_x] = "-"
            else:
                self.mymap[self.map_location_y + 2][self.map_location_x] = "F"

        elif self.orientation == orientation.Down:
            if self.current_measures[0] > 1:
                self.mymap[self.map_location_y + 1][self.map_location_x] = "-"
            else:
                self.mymap[self.map_location_y + 2][self.map_location_x] = "F"

            if self.current_measures[1] > 1:
                self.mymap[self.map_location_y][self.map_location_x + 1] = "|"
            else:
                self.mymap[self.map_location_y][self.map_location_x + 2] = "F"

            if self.current_measures[2] > 1:
                self.mymap[self.map_location_y][self.map_location_x - 1] = "|"
            else:
                self.mymap[self.map_location_y][self.map_location_x - 2] = "F"

            if self.current_measures[3] > 1:
                self.mymap[self.map_location_y - 1][self.map_location_x] = "-"
            else:
                self.mymap[self.map_location_y - 2][self.map_location_x] = "F"

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

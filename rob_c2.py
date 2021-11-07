import math
import sys
import time

from croblink import *
from math import *
import xml.etree.ElementTree as ET
import collections
from scipy.signal.signaltools import wiener
import pprint

pp = pprint.PrettyPrinter(indent=10)

CELLROWS = 7
CELLCOLS = 14


class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.current_measures = []
        self.previous_x = 0
        self.previous_y = 0
        self.first_x = 0
        self.first_y = 0
        self.first_stage = True
        self.state = "map"
        self.mymap = [[' '] * (CELLCOLS * 4 - 1) for i in range(CELLROWS * 4 - 1)]
        self.first_move = True
        self.current_location_x = 27
        self.current_location_y = 13
        self.orientation = "right"
        self.free_turns = []
        self.target_heading = 0
        self.supposed_heading = 0
        self.rotating = False
        self.moving = False
        self.supposed_x = 0
        self.supposed_y = 0

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

            if self.measures.compass < 0:
                self.measures.compass = self.measures.compass + 360

            if self.first_move:
                self.first_x = self.measures.x
                self.first_y = self.measures.y
                self.supposed_x = self.measures.x
                self.supposed_y = self.measures.y
                self.mymap[13][27] = "I"
                self.first_move = False

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

        center = self.current_measures[0]
        left = self.current_measures[1]
        right = self.current_measures[2]
        back = self.current_measures[3]

        if self.state == "forward":
            if self.next():
                self.forward()
            else:
                print("we need to rotate")
                self.state = "rotate"

        if self.state == "map":
            print("Mapping")
            print(left, center, right, back)
            print(self.orientation)
            self.map()
            self.state = "forward"
            # forward has priority
        if self.state == "rotate":
            self.rotate_degrees(90)
        if self.state == "stop":
            self.stop()

    def forward(self):
        deviation = self.calculate_deviation_heading()
        print(deviation)
        if self.first_stage:
            self.previous_x = self.supposed_x
            self.previous_y = self.supposed_y
            self.first_stage = False
        if self.orientation == "right":

            if self.measures.x < self.previous_x + 2:
                self.moving = True
                self.move(0.1, 0.1, 0, deviation)
            else:
                self.moving = False
                self.current_location_x+=2
                self.stop()
                self.supposed_x += 2

        elif self.orientation == "left":
            if self.measures.x > self.previous_x - 2:
                self.moving = True
                self.move(0.1, 0.1, 0, deviation)
            else:
                self.stop()
                self.moving = False
                self.current_location_x -= 2
                self.supposed_x += 2

        elif self.orientation == "up":
            if self.measures.y <= self.previous_y + 2:
                self.moving = True
                self.move(0.1, 0.1, 0, deviation)
            else:
                self.stop()
                self.moving = False
                self.current_location_y -= 2
                self.supposed_y -= 2
        elif self.orientation == "down":
            if self.measures.y > self.previous_y - 2:
                self.moving = True
                self.move(0.1, 0.1, 0, deviation)
            else:
                self.stop()
                self.moving = False
                self.current_location_y += 2
                self.supposed_y -= 2
        if not self.moving:
            self.state = "map"
            self.first_stage = True
            self.stop()

    def move(self, linear, k, m, r):
        right_power = linear + (k * (m - r)) / 2
        left_power = linear - (k * (m - r)) / 2
        self.driveMotors(left_power, right_power)

    def stop(self):
        for i in range(0,15):
            self.driveMotors(-0.15,-0.15)
        self.driveMotors(0,0)

    def rotate_degrees(self, degrees):
        if not self.rotating:
            self.target_heading = (self.supposed_heading+degrees) % 360
        deviation = self.calculate_deviation_heading()
        print(self.target_heading, deviation, self.measures.compass)
        if not (self.target_heading-3 <= self.measures.compass <= self.target_heading + 3) :
            if degrees > 0:
                self.move(0, 0.1, 0, -0.8)
            else:
                self.move(0, 0.1, 0, 0.8)
            self.rotating = True
        else:
            print("rotation complete")
            self.update_orientation(degrees)
            print(self.orientation)
            self.rotating = False
            self.supposed_heading = self.target_heading
            self.state = "forward"

    def update_orientation(self, degrees):
        if self.orientation == "right":
            if degrees == 90:
                self.orientation = "up"
            elif degrees == 180:
                self.orientation = "left"
            elif degrees == 270:
                self.orientation = "down"
        elif self.orientation == "up":
            if degrees == 90:
                self.orientation = "left"
            elif degrees == 180:
                self.orientation = "down"
            elif degrees == 270:
                self.orientation = "right"
        elif self.orientation == "left":
            if degrees == 90:
                self.orientation = "down"
            elif degrees == 180:
                self.orientation = "right"
            elif degrees == 270:
                self.orientation = "up"
        elif self.orientation == "down":
            if degrees == 90:
                self.orientation = "right"
            elif degrees == 180:
                self.orientation = "up"
            elif degrees == 270:
                self.orientation = "left"


    def calculate_deviation_heading(self):
        deviation = self.supposed_heading - self.measures.compass
        print(self.supposed_heading, self.measures.compass)
        if deviation < 0:
            deviation = deviation % 360
            return -0.0001*deviation
        else:
            return 0.0001*deviation


    def map(self):

        if self.orientation == "right":
            if self.current_measures[0] > 1:
                self.mymap[self.current_location_y][self.current_location_x + 1] = "|"
            else:
                self.mymap[self.current_location_y][self.current_location_x + 2] = "F"

            if self.current_measures[1] > 1:
                self.mymap[self.current_location_y + 1][self.current_location_x] = "-"
            else:
                self.mymap[self.current_location_y + 2][self.current_location_x] = "F"

            if self.current_measures[2] > 1:
                self.mymap[self.current_location_y - 1][self.current_location_x] = "-"
            else:
                self.mymap[self.current_location_y - 2][self.current_location_x] = "F"

            if self.current_measures[3] > 1:
                self.mymap[self.current_location_y][self.current_location_x - 1] = "|"
            else:
                self.mymap[self.current_location_y][self.current_location_x - 2] = "F"

        elif self.orientation == "up":
            if self.current_measures[0] > 1:
                self.mymap[self.current_location_y - 1][self.current_location_x] = "-"
            else:
                self.mymap[self.current_location_y - 2][self.current_location_x] = "F"

            if self.current_measures[1] > 1:
                self.mymap[self.current_location_y][self.current_location_x - 1] = "|"
            else:
                self.mymap[self.current_location_y][self.current_location_x - 2] = "F"

            if self.current_measures[2] > 1:
                self.mymap[self.current_location_y][self.current_location_x + 1] = "|"
            else:
                self.mymap[self.current_location_y][self.current_location_x + 2] = "F"

            if self.current_measures[3] > 1:
                self.mymap[self.current_location_y + 1][self.current_location_x] = "-"
            else:
                self.mymap[self.current_location_y + 2][self.current_location_x] = "F"

        elif self.orientation == "down":
            if self.current_measures[0] > 1:
                self.mymap[self.current_location_y + 1][self.current_location_x] = "-"
            else:
                self.mymap[self.current_location_y + 2][self.current_location_x] = "F"

            if self.current_measures[1] > 1:
                self.mymap[self.current_location_y][self.current_location_x + 1] = "|"
            else:
                self.mymap[self.current_location_y][self.current_location_x + 2] = "F"

            if self.current_measures[2] > 1:
                self.mymap[self.current_location_y][self.current_location_x - 1] = "|"
            else:
                self.mymap[self.current_location_y][self.current_location_x - 2] = "F"

            if self.current_measures[3] > 1:
                self.mymap[self.current_location_y - 1][self.current_location_x] = "-"
            else:
                self.mymap[self.current_location_y - 2][self.current_location_x] = "F"


        return

    def next(self):
        if self.orientation == "right":
            return self.mymap[self.current_location_y][self.current_location_x + 2] == "F" and self.mymap[self.current_location_y][self.current_location_x + 1] != "|"
        elif self.orientation == "up":
            return self.mymap[self.current_location_y - 2][self.current_location_x] == "F" and self.mymap[self.current_location_y - 1][self.current_location_x] != "-"
        elif self.orientation == "left":
            return self.mymap[self.current_location_y][self.current_location_x - 2] == "F" and self.mymap[self.current_location_y][self.current_location_x - 1] != "|"
        else:
            return self.mymap[self.current_location_y + 2][self.current_location_x] == "F" and self.mymap[self.current_location_y + 1][self.current_location_x] != "-"



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

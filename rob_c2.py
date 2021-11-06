import math
import sys
import time

from croblink import *
from math import *
import xml.etree.ElementTree as ET
import collections
from scipy.signal.signaltools import wiener

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
        self.mymap = [[' '] * (CELLCOLS) for i in range(CELLROWS)]
        self.first_move = True
        self.current_location_x = 1
        self.current_location_y = 1
        self.orientation = "right"
        self.free_turns = []
        self.target_heading = 0
        self.rotating = False

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
            if self.first_move:
                self.first_x = self.measures.x
                self.first_y = self.measures.y
                self.mymap[1][1] = "I"
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
                print("RUN WANDER")
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
                print("WAIT WANDER")
                self.wander()

    def wander(self):

        center = self.current_measures[0]
        left = self.current_measures[1]
        right = self.current_measures[2]
        back = self.current_measures[3]
        gps_x = self.measures.x
        gps_y = self.measures.y
        compass = self.measures.compass
        print(compass)
        if self.state == "forward":
            if self.first_stage:
                self.previous_x = gps_x
                self.previous_y = gps_y
                self.first_stage = False
            if gps_x < self.previous_x + 2:
                print(gps_x, self.previous_x)
                self.forward(0.075 * (self.previous_x + 2 - gps_x)+0.05, 0.1, 0, 0)
            else:
                self.state = "map"
                self.first_stage = True
                self.stop()
        if self.state == "map":
            print(left, center, right, back)
            self.map()

            # forward has priority
            if self.next():
                self.state = "forward"
            else:
                self.state = "rotate"
        if self.state == "rotate":
            self.rotate(90)

        if self.state == "stop":
            self.stop()

    def forward(self, linear, k, m, r):
        right_power = linear + (k * (m - r)) / 2
        left_power = linear - (k * (m - r)) / 2
        self.driveMotors(left_power, right_power)

    def stop(self):
        self.driveMotors(0, 0)

    def rotate(self, degrees):
        if n := self.measures.compass < 0:
            gps = n + 360
        else:
            gps = self.measures.compass
        if not self.rotating:
            self.target_heading = (gps+degrees) % 360
        print(gps, self.target_heading)
        if degrees > 0:
            if not (self.target_heading-5 <= gps <= self.target_heading + 5) :
                self.forward(0,0.1,0,-1)
                self.rotating = True
            else:
                self.rotating = False
                self.state = "stop"


    def map(self):
        if self.orientation == "right":

            if self.current_measures[0] > 2:
                self.mymap[self.current_location_x + 1][self.current_location_y] = "|"
            else:
                self.mymap[self.current_location_x + 1][self.current_location_y] = "F"

            if self.current_measures[1] > 2:
                self.mymap[self.current_location_x][self.current_location_y + 1] = "-"
            else:
                self.mymap[self.current_location_x][self.current_location_y - 1] = "F"

            if self.current_measures[2] > 2:
                self.mymap[self.current_location_x][self.current_location_y - 1] = "-"
            else:
                self.mymap[self.current_location_x][self.current_location_y - 1] = "F"

            if self.current_measures[3] > 2:
                self.mymap[self.current_location_x - 1][self.current_location_y] = "|"
            else:
                self.mymap[self.current_location_x - 1][self.current_location_y] = "F"


        elif self.orientation == "up":
            if self.current_measures[0] > 2:
                self.mymap[self.current_location_x][self.current_location_y + 1] = "-"
            else:
                self.mymap[self.current_location_x][self.current_location_y - 1] = "F"

            if self.current_measures[1] > 2:
                self.mymap[self.current_location_x - 1][self.current_location_y] = "|"
            else:
                self.mymap[self.current_location_x - 1][self.current_location_y] = "F"

            if self.current_measures[2] > 2:
                self.mymap[self.current_location_x + 1][self.current_location_y] = "|"
            else:
                self.mymap[self.current_location_x + 1][self.current_location_y] = "F"

            if self.current_measures[3] > 2:
                self.mymap[self.current_location_x][self.current_location_y - 1] = "-"
            else:
                self.mymap[self.current_location_x][self.current_location_y - 1] = "F"

        return

    def next(self):
        if self.orientation == "right":
            return self.mymap[self.current_location_x + 1][self.current_location_y] == "F"
        elif self.orientation == "up":
            return self.mymap[self.current_location_x][self.current_location_y + 1] == "F"
        elif self.orientation == "left":
            return self.mymap[self.current_location_x - 1][self.current_location_y] == "F"
        else:
            return self.mymap[self.current_location_x][self.current_location_y - 1] == "F"


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
    rob = MyRob(rob_name, pos, [0.0, 90.0, -90.0, 270.0], host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()

    rob.run()

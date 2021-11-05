import math
import sys
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
        self.rotation_speed = 0.1
        self.linear_speed = 0.0
        self.rotation_speed_base = 0.0
        self.linear_speed_base = 0.1
        self.rotation = 0

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
        print("AVERAGE CENTER {}\nAVERAGE LEFT {}\nAVERAGE RIGHT {}\nAVERAGE BACK {}".format(center, left, right, back))


        CENTER_CLOSE_WALL_HARD_TURN = -0.02
        CENTER_CLOSE_WALL_NORMAL_TURN = 3.5
        KEEPING_CENTER_COEFFICIENT = 10
        LINEAR_CENTER_SPEED = 3
        ROTATIONAL_SPEED = 0.1

        if center > 1:
            print("Center wall close")
            if left < right:
                # print("Right wall closer to me")
                # print("AVERAGE CENTER {}\nAVERAGE LEFT {}\nAVERAGE RIGHT {}\nAVERAGE BACK {}".format(center, left, right, back))
                self.driveMotors(CENTER_CLOSE_WALL_HARD_TURN * center, CENTER_CLOSE_WALL_NORMAL_TURN * left + center)
            else:
                # print("Left wall closer to me")
                # print("AVERAGE CENTER {}\nAVERAGE LEFT {}\nAVERAGE RIGHT {}\nAVERAGE BACK {}".format(center, left, right, back))
                self.driveMotors(CENTER_CLOSE_WALL_NORMAL_TURN * right + center, CENTER_CLOSE_WALL_HARD_TURN * center)
            return
        elif left > 2.9:
            print("Left wall close")
            desvio = left - 2.15
            self.driveMotors(KEEPING_CENTER_COEFFICIENT * desvio + LINEAR_CENTER_SPEED * center, ROTATIONAL_SPEED * right)
            return
        elif right > 2.9:
            print("Right wall close")
            desvio = right - 2.15
            self.driveMotors(ROTATIONAL_SPEED * left, KEEPING_CENTER_COEFFICIENT * desvio + LINEAR_CENTER_SPEED * center)
            return
        else:
            self.driveMotors(1, 1)
            return


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
    rob = MyRob(rob_name, pos, [0.0, 60.0, -60.0, 180.0], host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()

    rob.run()

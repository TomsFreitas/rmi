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

        # First iteration
        # if center > 4 or left > 4 or right > 4:
        #     self.driveMotors(-0.1, +0.1)
        #     print("First IF")
        # elif left > 0.7:
        #     print("Second IF")
        #     self.driveMotors(0.2, 0.0)
        # elif right > 0.7:
        #     print("third IF")
        #     self.driveMotors(0.0, 0.2)
        # else:
        #     self.driveMotors(0.2,0.2)
        # SECOND ITERATION
        # if center > 2 and right > 2:
        #     self.driveMotors(-0.1, 0.2)
        #     print("first if")
        # elif center > 2 and left > 2:
        #     print("second if")
        #     self.driveMotors(0.2, -0.1)
        # elif right > 4 or left > 4:
        #     print("third if")
        #     if left > right:
        #         self.driveMotors(left, 0)
        #     else:
        #         self.driveMotors(0, right)
        # else:
        #     print("else")
        #     self.driveMotors(0.5, 0.5)

        #if center < 4 and right < 2.1 and left < 2.1:
            #self.driveMotors(0.5,0.5)
            #print("Driving forward")
        #else:

        if center > 1.2:
            print("Center wall close")
            if left < right:
                #print("Right wall closer to me")
                #print("AVERAGE CENTER {}\nAVERAGE LEFT {}\nAVERAGE RIGHT {}\nAVERAGE BACK {}".format(center, left, right, back))
                self.driveMotors(-0.3*center, 1.2*right)
            else:
                #print("Left wall closer to me")
                #print("AVERAGE CENTER {}\nAVERAGE LEFT {}\nAVERAGE RIGHT {}\nAVERAGE BACK {}".format(center, left, right, back))
                self.driveMotors(1.2*left, -0.3*center)
        elif left > 3:
            print("Left wall close")
            desvio = left-2.15
            self.driveMotors(3*desvio+0.6*center, 0.1*right)
        elif right > 3:
            print("Right wall close")
            desvio = right-2.15
            self.driveMotors(0.1*left,3*desvio+0.6*center)
        else:
            self.driveMotors(0.5,0.5)






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

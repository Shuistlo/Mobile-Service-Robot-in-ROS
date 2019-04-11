#!/usr/bin/env python

import importlib
# NECESSARY IMPORTS
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import rospy
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
# LOCATION DICTIONARY
#EXTRAS
import os
import smtplib
from email.mime.text import MIMEText

from controller_framework import *

class Controller:
    def __init__(self):
        #node
        rospy.init_node('the_only_node', anonymous=True)

        self.nav = simple_navigation_node.GoToPose()
        self.repose = repose_node.poseSetter()
        self.cleaner = clearMap.mapCleaner()
        self.teleporter = teleport_node.Teleporter()
        self.landmarks = landmarks

        #information UPDATE ACCORDINGLY
        self.floor = 7
        self.floorPortion = "L"

        self.inUse = False
        self.elevatorTime = False
        self.inEvel = False
        self.currentGoalPath = []
        self.currentGoal = ""

    #should call to handle moving in and out of the elevator
    def evelEvent(self):
        os.system("espeak 'please press the elevator button for me'")
        os.system("espeak 'please tell me when to enter the elevator by pressing E'")
        xText = raw_input("press E to Enter/Exit an elevator: ")
        index = self.currentGoalPath.index("ELEVATORTIME")

        rightQuat = [0, 0, 0, 1]
        leftQuat = [0, 0, 1, 0]

        interest = self.currentGoalPath[index + 1]
        success = self.attempt3(interest, rightQuat, 30)
        if not success:
            rospy.loginfo("cant seem to get in the elevator, asking for help....")
            os.system("espeak 'can't seem to get in the elevator'")
            os.system("espeak 'seeking help'")
            self.askForHelp(False)
            return success

        os.system("espeak 'please press the floor button'")
        self.teleporter.set_map_filename_client(float(self.currentGoal[0])) #HOPEFULLY THIS WORKS
        goalDict = "dict" + self.currentGoal[0] + "L"
        pos = self.landmarks.dictdict[goalDict]["ELEVATOR"]
        self.repose.newPose(pos[0], pos[1])

        os.system("espeak 'please tell me when to exit the elevator by pressing E'")
        xText = raw_input("press E to Enter/Exit an elevator: ")

        self.cleaner.reset_costmap_layers()
        index = index + 2
        interest = self.currentGoalPath[index]
        success = self.attempt3(interest, leftQuat, 30)
        if not success:
            rospy.loginfo("cant seem to get in the elevator, asking for help....")
            os.system("espeak 'can't seem to get out of the elevator'")
            os.system("espeak 'seeking help'")
            self.askForHelp(True)
            return success

            index = index + 1
            self.go(index, len(self.currentGoalPath))

        self.go(index + 1, len(self.currentGoalPath))
        return success

    #should get a generated route and perform the whole route if its on the same floor
    #else the elevator logic should be completed by evelEvent
    def goFirst(self, room):
        os.system("espeak 'going to goal'")
        self.inUse = True
        route = self.generateRoute(room)
        print(route)

        if False in route:
            rospy.loginfo("Invalid Room Entered!")
            os.system("espeak 'invalid room entered'")
            os.system("espeak 'please enter room in format floor, room number, wing'")
            self.inUse = False
            return False

        self.currentGoalPath = route
        self.currentGoal = room

        #chop at the elevator
        if "ELEVATORTIME" in self.currentGoalPath:
            completion = route.index("ELEVATORTIME")
        else:
            completion = len(route)
        self.go(0, completion)

    def go(self, start, completion):
        #complete route generated
        rightQuat = [0, 0, 0, 1]
        leftQuat = [0, 0, 1, 0]
        success = True
        index = 0
        while index < completion and success:
            interest = self.currentGoalPath[index]
            index = index + 1

            if not isinstance(interest, str) and len(interest) != 1:
                quat = leftQuat
                if index < (len(self.currentGoalPath) - 1):
                    if (self.currentGoalPath[index + 1] == "SLEEP") and (self.floorPortion == "N" or "L"):
                        quat = rightQuat
                success = self.attempt3(interest, quat, 60)
                continue
            elif len(interest) == 1:
                self.floorPortion = interest
                print("portion is " + self.floorPortion)
            elif interest == "SLEEP":
                os.system("espeak 'please open the door for me'")
                rospy.sleep(5)
                self.cleaner.clearMapClient() #potentially needs a full cleaner

        if not success:
            failure = self.currentGoalPath[index-1]
            rospy.loginfo("had trouble going to " + str(failure) + " seeking help...")
            os.system("espeak 'had trouble reaching the goal'")
            os.system("espeak 'seeking help'")
            os.system("espeak 'returning to help point'")
            self.askForHelp(False)
            currentDict = "dict" + str(self.floor) + str(self.floorPortion)
            loc = self.landmarks.dictdict[currentDict]["RETREAT"]
            failure = self.attempt3(loc, leftQuat, 60)

        #clean up at end
        if completion == len(self.currentGoalPath) and success: #so no elevator stuff happened
            self.currentGoalPath == []
            self.currentGoal = ""
            os.system("espeak 'We have arrived at our destination'")
            rospy.sleep(2)
            os.system("espeak 'returning to help point'")
            currentDict = "dict" + str(self.floor) + str(self.floorPortion)
            loc = self.landmarks.dictdict[currentDict]["RETREAT"]
            failure = self.attempt3(loc, leftQuat, 60)
            rospy.loginfo("localizing and clearing costmaps")
            os.system("espeak 'localising and clearing costmaps'")
            self.nav.makesmallspin()
            self.cleaner.reset_costmap_layers()
        return success

    #given a goal generate a path
    def generateRoute(self, goal):
        goalRoute = []  # give it location
        robLobbyDict = "dict" + str(self.floor) + "L"
        robcurrentDict = "dict" + str(self.floor) + self.floorPortion
        goalDict = "dict" + goal[0] + goal[3]  # which dictionary the goal should be located

        #lobby requests
        if len(goal) != 4 and goal not in self.landmarks.dict5L:
            goalRoute.append(False)
            return goalRoute
        if len(goal) == 4 and goal not in self.landmarks.dictdict[goalDict]:
            goalRoute.append(False)
            return goalRoute
        if len(goal) != 4:
            if self.floorPortion == "L":
                goalRoute.append(self.landmarks.dictdict[robLobbyDict][goal])
                return goalRoute
            else:
                goalRoute.append(self.landmarks.dictdict[robcurrentDict]["AWAITEXIT"])
                goalRoute.append("SLEEP")
                goalRoute.append(self.landmarks.dictdict[robLobbyDict]["AWAIT" + self.floorPortion])
                goalRoute.append("L")
                goalRoute.append(self.landmarks.dictdict[robLobbyDict][goal])
                return goalRoute

        #non lobby goals
        #on the same wing but not lobby goals
        elif goal[0] == str(self.floor) and goal[3] == self.floorPortion:
            goalRoute.append(self.landmarks.dictdict[goalDict][goal])
            return goalRoute

        #on the same floor
        elif goal[3] != self.floorPortion and goal[0] == str(self.floor):
            if self.floorPortion != "L":
                goalRoute.append(self.landmarks.dictdict[robcurrentDict]["AWAITEXIT"])
                goalRoute.append("SLEEP")
                goalRoute.append(self.landmarks.dictdict[robLobbyDict]["AWAIT" + self.floorPortion])
                goalRoute.append("L")
            goalRoute.append(self.landmarks.dictdict[robLobbyDict]["AWAIT" + goal[3]])
            goalRoute.append("SLEEP")
            goalRoute.append(self.landmarks.dictdict[goalDict]["AWAITEXIT"])
            goalRoute.append(goal[3])
            goalRoute.append(self.landmarks.dictdict[goalDict][goal])
            return goalRoute

        #not on the same floor
        elif goal[0] != str(self.floor):
            if self.floorPortion != "L":
                goalRoute.append(self.landmarks.dictdict[robcurrentDict]["AWAITEXIT"])
                goalRoute.append("SLEEP")
                f = "AWAIT" + self.floorPortion
                goalRoute.append(self.landmarks.dictdict[robLobbyDict]["AWAIT" + self.floorPortion])
                goalRoute.append("L")
            #now get to the elevator
            goalRoute.append(self.landmarks.dictdict[robLobbyDict]["AWAITTELE"])
            goalRoute.append("ELEVATORTIME")
            goalRoute.append(self.landmarks.dictdict[robLobbyDict]["ELEVATOR"])
            goalDictLobby = "dict" + goal[0] + "L"
            goalRoute.append(self.landmarks.dictdict[goalDictLobby]["AWAITTELE"])
            goalRoute.append(self.landmarks.dictdict[goalDictLobby]["AWAIT" + goal[3]])
            goalRoute.append("SLEEP")
            goalRoute.append(self.landmarks.dictdict[goalDict]["AWAITEXIT"])
            goalRoute.append(goal[3])
            goalRoute.append(self.landmarks.dictdict[goalDict][goal])
            return goalRoute

    #attempt to go to a goal 3 times (so this would be one set of coordinates, not the whole goal path)
    def attempt3(self, goal, quat, actionTime):
        errorCounter = 0
        success = False
        position = {'x': goal[0], 'y': goal[1]}
        #quaternion = {'r1': 0.000, 'r2': 0.000, 'r3': 0.000, 'r4': 1.000}
        quaternion = {'r1': quat[0], 'r2': quat[1], 'r3': quat[2], 'r4': quat[3]}
        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])

        while not success and errorCounter < 3:
            success = self.nav.goto(position, quaternion, float(actionTime))
            if success:
                rospy.loginfo("GOAL REACHED")
            else:
                rospy.loginfo("GOAL FAILED")
                self.cleaner.reset_costmap_layers()

            # Sleep to give the last log messages time to be sent
            rospy.sleep(1)
            errorCounter = errorCounter + 1
        return success

    def askForHelp(self, evel):
        statement = "help, I'm on floor " + str(self.floor) + " in " + self.floorPortion + " please rescue me :C"
        if evel:
            statement = "help, I'm stuck in the elevator please rescue me :C"
        smtp_ssl_host = 'smtp.mail.yahoo.com'
        smtp_ssl_port = 465
        username = 'definitelynotabot95@yahoo.com'
        password = 'iaintnobot'
        sender = 'definitelynotabot95@yahoo.com'
        target = 'k1630593@kcl.ac.uk'

        msg = MIMEText(statement)
        msg['Subject'] = 'HELP ' + statement
        msg['From'] = sender
        msg['To'] = target

        server = smtplib.SMTP_SSL(smtp_ssl_host, smtp_ssl_port)
        server.login(username, password)
        server.sendmail(sender, target, msg.as_string())
        server.quit()

    def debugger(self):
        print(f.generateRoute("703N"))
        print(f.generateRoute("MTOILET"))
        print(f.generateRoute("603N"))

if __name__ == "__main__":
    f = Controller()
    f.nav.makesmallspin()
    f.debugger()
    while True:
        room = raw_input("enter a room: ")
        if room[0] != str(f.floor) and len(room) == 4:
            f.goFirst(room)
            f.evelEvent()
        elif len(room) != 4 or room[0] == str(f.floor):
            f.goFirst(room)



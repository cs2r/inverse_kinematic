#!/usr/bin/env python
################################################################################
# Copyright (C) 2012-2013 Leap Motion, Inc. All rights reserved.               #
# Leap Motion proprietary and confidential. Not for distribution.              #
# Use subject to the terms of the Leap Motion SDK Agreement available at       #
# https://developer.leapmotion.com/sdk_agreement, or another agreement         #
# between Leap Motion and you, your company or other organization.             #
################################################################################

import sys, time, os.path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from src.lib import Leap
import rospy
from geometry_msgs.msg import Vector3



class SampleListener(Leap.Listener):
    finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
    bone_names = ['Metacarpal', 'Proximal', 'Intermediate', 'Distal']
    state_names = ['STATE_INVALID', 'STATE_START', 'STATE_UPDATE', 'STATE_END']

    def on_init(self, controller):
        print "Initialized"
        rospy.init_node("leap_motion_publisher")
        self.pos_pub=rospy.Publisher("position", Vector3, queue_size=10)
        self.ort_pub=rospy.Publisher('orientation', Vector3 , queue_size=10)
        self.pos=Vector3()
        self.ort=Vector3()

    def on_connect(self, controller):
        print "Connected"

        # Enable gestures
        controller.enable_gesture(Leap.Gesture.TYPE_CIRCLE);
        controller.enable_gesture(Leap.Gesture.TYPE_KEY_TAP);
        controller.enable_gesture(Leap.Gesture.TYPE_SCREEN_TAP);
        controller.enable_gesture(Leap.Gesture.TYPE_SWIPE);

    def on_disconnect(self, controller):
        # Note: not dispatched when running in a debugger.
        print "Disconnected"

    def on_exit(self, controller):
        print "Exited"

    def on_frame(self, controller):
        # Get the most recent frame and report some basic information
        frame = controller.frame()



        # Get hands
        for hand in frame.hands:

            handType = "Left hand" if hand.is_left else "Right hand"



            # Get the hand's normal vector and direction
            normal = hand.palm_normal
            direction = hand.direction

            self.ort.x=direction.pitch
            self.ort.y=normal.roll


            # Get arm bone
            arm = hand.arm

            self.pos.z=arm.wrist_position[1]
            self.pos.y=-arm.wrist_position[2]
            self.pos_pub.publish(self.pos)
            # Get fingers
            for finger in hand.fingers:
                # Get bones
                if finger.type == 1:
                    index_dir = finger.direction[0]
                elif finger.type == 2:
                    medlle_dir = finger.direction[0]
            gripper = abs(index_dir - medlle_dir)
            self.ort.z = gripper
            self.ort_pub.publish(self.ort)
            time.sleep(0.01)


        if not (frame.hands.is_empty and frame.gestures().is_empty):
            print ""

    def state_string(self, state):
        if state == Leap.Gesture.STATE_START:
            return "STATE_START"

        if state == Leap.Gesture.STATE_UPDATE:
            return "STATE_UPDATE"

        if state == Leap.Gesture.STATE_STOP:
            return "STATE_STOP"

        if state == Leap.Gesture.STATE_INVALID:
            return "STATE_INVALID"

def main():
    # Create a sample listener and controller
    listener = SampleListener()
    controller = Leap.Controller()

    # Have the sample listener receive events from the controller
    controller.add_listener(listener)

    # Keep this process running until Enter is pressed
    print "Press Enter to quit..."
    try:
        sys.stdin.readline()
    except KeyboardInterrupt:
        pass
    finally:
        # Remove the sample listener when done
        controller.remove_listener(listener)


if __name__ == "__main__":
    main()

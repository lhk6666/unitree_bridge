#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import time
from std_msgs.msg import Empty
from dynamixel_sdk import *

class Dog_motions:
    def __init__(self):
        rospy.init_node('go1_attach_detach', anonymous=True)
        self.go1_attach_sub = rospy.Subscriber('/go1/attach', Empty, self.catch_objects)
        self.go1_detach_sub = rospy.Subscriber('/go1/detach', Empty, self.detach_objects)
        
        # Servo
        self.DEVICENAME = '/dev/ttyS4'
        self.BAUDRATE = 1000000
        self.PROTOCOL_VERSION = 2.0
        self.DXL_ID = 1
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_PRESENT_POSITION = 132
        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0
        self.DXL_MINIMUM_POSITION_VALUE = 180
        self.DXL_MAXIMUM_POSITION_VALUE = 2245
        self.DXL_MOVING_STATUS_THRESHOLD = 10

    def catch_objects(self, msg):
        portHandler = PortHandler(self.DEVICENAME)
        packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        if portHandler.openPort():
            print("Success open port")
        else:
            print("Failure open port")
            exit()

        if portHandler.setBaudRate(self.BAUDRATE):
            print("Set baudrate")
        else:
            print("Can not set baudrate")
            exit()

        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Failure communication: {packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"Error of servo: {packetHandler.getRxPacketError(dxl_error)}")
        else:
            print("Servo is running")

        goal_positions = [self.DXL_MINIMUM_POSITION_VALUE, self.DXL_MAXIMUM_POSITION_VALUE]
        index = 0

        try:
            while index < 2:
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, self.DXL_ID, self.ADDR_GOAL_POSITION, goal_positions[index])
                if dxl_comm_result != COMM_SUCCESS:
                    print(f"Failure communication: {packetHandler.getTxRxResult(dxl_comm_result)}")
                elif dxl_error != 0:
                    print(f"Error of servo: {packetHandler.getRxPacketError(dxl_error)}")

                while True:
                    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, self.DXL_ID, self.ADDR_PRESENT_POSITION)
                    if dxl_comm_result != COMM_SUCCESS:
                        print(f"Failure communication: {packetHandler.getTxRxResult(dxl_comm_result)}")
                    elif dxl_error != 0:
                        print(f"Error of servo: {packetHandler.getRxPacketError(dxl_error)}")

                    print(f"Current position: {dxl_present_position}")

                    if abs(goal_positions[index] - dxl_present_position) < self.DXL_MOVING_STATUS_THRESHOLD:
                        break

                index+=1
                time.sleep(1)

        except KeyboardInterrupt:
            print("Interrupt")

        packetHandler.write1ByteTxRx(portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)

        portHandler.closePort()

    def detach_objects(self, msg):
        portHandler = PortHandler(self.DEVICENAME)
        packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        if portHandler.openPort():
            print("Success open port")
        else:
            print("Failure open port")
            exit()

        if portHandler.setBaudRate(self.BAUDRATE):
            print("Set baudrate")
        else:
            print("Can not set baudrate")
            exit()

        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Failure communication: {packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"Error of servo: {packetHandler.getRxPacketError(dxl_error)}")
        else:
            print("Servo is running")

        goal_positions = [self.DXL_MAXIMUM_POSITION_VALUE, self.DXL_MINIMUM_POSITION_VALUE]
        index = 0

        try:
            while index < 2:
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, self.DXL_ID, self.ADDR_GOAL_POSITION, goal_positions[index])
                if dxl_comm_result != COMM_SUCCESS:
                    print(f"Failure communication: {packetHandler.getTxRxResult(dxl_comm_result)}")
                elif dxl_error != 0:
                    print(f"Error of servo: {packetHandler.getRxPacketError(dxl_error)}")

                while True:
                    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, self.DXL_ID, self.ADDR_PRESENT_POSITION)
                    if dxl_comm_result != COMM_SUCCESS:
                        print(f"Failure communication: {packetHandler.getTxRxResult(dxl_comm_result)}")
                    elif dxl_error != 0:
                        print(f"Error of servo: {packetHandler.getRxPacketError(dxl_error)}")

                    print(f"Current position: {dxl_present_position}")

                    if abs(goal_positions[index] - dxl_present_position) < self.DXL_MOVING_STATUS_THRESHOLD:
                        break

                index+=1
                time.sleep(1)

        except KeyboardInterrupt:
            print("Interrupt")

        packetHandler.write1ByteTxRx(portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)

        portHandler.closePort()

if __name__ == '__main__':
    try:
        Dog_motions()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
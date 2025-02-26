#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import socket
import json
import sys
import importlib
import yaml
import os
from std_srvs.srv import Trigger

def json_to_rosmsg(data_dict, msg_class):
    msg = msg_class()

    def assign_fields(msg_obj, d):
        for slot, stype in zip(msg_obj.__slots__, msg_obj._slot_types):
            val = d.get(slot, None)
            if val is None:
                continue
            if hasattr(msg_obj, slot) and hasattr(getattr(msg_obj, slot), '__slots__'):
                assign_fields(getattr(msg_obj, slot), val)
            else:
                setattr(msg_obj, slot, val)

    assign_fields(msg, data_dict)
    return msg

def service_client_stand():
    try:
        service_client_stand = rospy.ServiceProxy('/go1/stand', Trigger)
        response = service_client_stand()
        return response
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def service_client_sit():
    try:
        service_client_sit = rospy.ServiceProxy('/go1/sit', Trigger)
        response = service_client_sit()
        return response
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    rospy.init_node("udp_receiver_node")

    config_path = rospy.get_param("~config_path", "")
    if not config_path or not os.path.exists(config_path):
        rospy.logerr("Config file not found or not specified.")
        sys.exit(1)
        
    receive_ip = rospy.get_param("~ip", "0.0.0.0")
    receive_port = rospy.get_param("~port", 5000)

    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
    except Exception as e:
        rospy.logerr("Failed to load config file: %s", e)
        sys.exit(1)

    topic_map = {}
    for t in config['topics']:
        topic_name = t['name']
        msg_type_str = t['type']
        package_name, message_name = msg_type_str.split('/')
        module = importlib.import_module(package_name + '.msg')
        msg_class = getattr(module, message_name)

        pub = rospy.Publisher(topic_name, msg_class, queue_size=10)
        topic_map[topic_name] = (msg_class, pub)
        rospy.loginfo("Setup receiver for topic: %s (%s)", topic_name, msg_type_str)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((receive_ip, receive_port))
    except Exception as e:
        rospy.logerr("Failed to bind UDP socket: %s", e)
        sys.exit(1)

    rospy.loginfo("UDP receiver node started. Listening on %s:%d", receive_ip, receive_port)
    
    while not rospy.is_shutdown():
        try:
            data, addr = sock.recvfrom(4096)
            if not data:
                continue
            msg_str = data.decode('utf-8')
            try:
                packet = json.loads(msg_str)
                topic_name = packet.get("topic_name", None)
                data_dict = packet.get("data", None)

                if topic_name is None or data_dict is None:
                    rospy.logwarn("Invalid packet format: %s", msg_str)
                    continue

                if topic_name == "/go1/sit":
                    service_client_sit()
                    rospy.loginfo("Received from %s: topic: %s -> called service /go1/sit", addr, topic_name)
                    continue

                if topic_name == "/go1/stand":
                    service_client_stand()
                    rospy.loginfo("Received from %s: topic: %s -> called service /go1/stand", addr, topic_name)
                    continue

                if topic_name not in topic_map:
                    rospy.logwarn("Received topic %s not in config.", topic_name)
                    continue

                msg_class, pub = topic_map[topic_name]
                ros_msg = json_to_rosmsg(data_dict, msg_class)
                pub.publish(ros_msg)
                rospy.loginfo("Received from %s: topic: %s -> published on %s", addr, topic_name, topic_name)
            except json.JSONDecodeError as je:
                rospy.logwarn("Failed to decode JSON: %s", je)
        except socket.error as se:
            rospy.logwarn("Socket error: %s", se)

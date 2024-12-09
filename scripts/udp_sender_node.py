#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import yaml
import json
import socket
import sys
import importlib

def message_to_dict(msg):
    if hasattr(msg, '__slots__'):
        data = {}
        for slot, stype in zip(msg.__slots__, msg._slot_types):
            val = getattr(msg, slot)
            if hasattr(val, '__slots__'):
                data[slot] = message_to_dict(val)
            else:
                data[slot] = val
        return data
    else:
        if hasattr(msg, 'data'):
            return {"data": msg.data}
        return {"data": str(msg)}


def callback_factory(topic_name, udp_socket, target_ip, target_port):
    def callback(msg):
        data_dict = message_to_dict(msg)
        send_dict = {
            "topic_name": topic_name,
            "data": data_dict
        }
        json_data = json.dumps(send_dict)
        udp_socket.sendto(json_data.encode('utf-8'), (target_ip, target_port))
    return callback


def import_message_type(msg_type_str):
    package_name, message_name = msg_type_str.split('/')
    module = importlib.import_module(package_name + '.msg')
    msg_class = getattr(module, message_name)
    return msg_class


if __name__ == '__main__':
    rospy.init_node('udp_sender_node')

    config_path = rospy.get_param("~config_path", "/home/dragon_llm/ros/unitree_bridge_ws/src/unitree_bridge/config/config.yaml")

    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
    except Exception as e:
        rospy.logerr("Failed to load config file: %s", e)
        sys.exit(1)

    target_ip = config['udp']['ip']
    target_port = config['udp']['port']

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    for t in config['topics']:
        topic_name = t['name']
        topic_type_str = t['type']

        msg_class = import_message_type(topic_type_str)

        rospy.Subscriber(topic_name, msg_class, callback_factory(topic_name, udp_socket, target_ip, target_port))
        rospy.loginfo("Subscribed to topic: %s (%s)", topic_name, topic_type_str)

    rospy.spin()

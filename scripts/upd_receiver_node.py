#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import socket
import json
import sys
import importlib
import yaml
import os

def json_to_rosmsg(data_dict, msg_class):
    # 根据msg_class和data_dict构建ROS消息
    # 这里与之前的message_to_dict相反，需要将字典映射回msg对象
    msg = msg_class()

    def assign_fields(msg_obj, d):
        # 递归赋值
        for slot, stype in zip(msg_obj.__slots__, msg_obj._slot_types):
            val = d.get(slot, None)
            if val is None:
                continue
            # 判断是否是子消息
            if hasattr(msg_obj, slot) and hasattr(getattr(msg_obj, slot), '__slots__'):
                # 子消息，递归
                assign_fields(getattr(msg_obj, slot), val)
            else:
                setattr(msg_obj, slot, val)

    assign_fields(msg, data_dict)
    return msg

if __name__ == "__main__":
    rospy.init_node("udp_flexible_receiver_node")

    # 从参数中获取config文件路径（假设与发送端共用同一份yaml）
    config_path = rospy.get_param("~config_path", "")
    if not config_path or not os.path.exists(config_path):
        rospy.logerr("Config file not found or not specified.")
        sys.exit(1)
        
    # UDP配置
    receive_ip = rospy.get_param("~ip", "0.0.0.0")
    receive_port = rospy.get_param("~port", 5000)

    # 读取YAML配置文件
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
    except Exception as e:
        rospy.logerr("Failed to load config file: %s", e)
        sys.exit(1)

    # 准备主题对应的Publisher与消息类型映射
    topic_map = {}
    for t in config['topics']:
        topic_name = t['name']
        msg_type_str = t['type']
        package_name, message_name = msg_type_str.split('/')
        module = importlib.import_module(package_name + '.msg')
        msg_class = getattr(module, message_name)

        # 为每个topic创建一个Publisher
        pub = rospy.Publisher(topic_name + "_received", msg_class, queue_size=10)
        topic_map[topic_name] = (msg_class, pub)
        rospy.loginfo("Setup receiver for topic: %s -> %s_received (%s)", topic_name, topic_name, msg_type_str)

    # 创建UDP socket并绑定
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((receive_ip, receive_port))
    except Exception as e:
        rospy.logerr("Failed to bind UDP socket: %s", e)
        sys.exit(1)

    rospy.loginfo("Flexible UDP receiver node started. Listening on %s:%d", receive_ip, receive_port)

    # 接收循环
    while not rospy.is_shutdown():
        try:
            data, addr = sock.recvfrom(4096)
            if not data:
                continue
            msg_str = data.decode('utf-8')
            # JSON解析
            try:
                packet = json.loads(msg_str)
                topic_name = packet.get("topic_name", None)
                data_dict = packet.get("data", None)

                if topic_name is None or data_dict is None:
                    rospy.logwarn("Invalid packet format: %s", msg_str)
                    continue

                if topic_name not in topic_map:
                    rospy.logwarn("Received topic %s not in config.", topic_name)
                    continue

                msg_class, pub = topic_map[topic_name]
                # 将data_dict映射回ROS消息
                ros_msg = json_to_rosmsg(data_dict, msg_class)
                # 发布到对应的话题（带后缀_received）
                pub.publish(ros_msg)
                rospy.loginfo("Received from %s: topic: %s -> published on %s_received", addr, topic_name, topic_name)
            except json.JSONDecodeError as je:
                rospy.logwarn("Failed to decode JSON: %s", je)
        except socket.error as se:
            rospy.logwarn("Socket error: %s", se)

# Unitree Bridge Package (For Dragon Lab unitree go1 configuration)

This package provides functionality to:

- Subscribe to specified ROS topics,
- Serialize the received messages into JSON format,
- Send them via UDP to another machine within the same network,
- Receive the JSON messages on the other machine and re-publish them as ROS topics.

The package is configured via a shared YAML file that defines which topics to handle and what message types they use, ensuring that both sender and receiver nodes operate with the same configuration.

## Features

- **Configurable via YAML:** No need to hardcode topic names or message types. Both sender and receiver use the same `config.yaml`.
- **Flexible Message Handling:** The nodes dynamically load message types and can handle multiple topics as defined in the configuration.
- **Common Network Transport:** Uses UDP for lightweight, low-latency communication between machines on the same network.
- **JSON Serialization:** Topic messages are converted to JSON for easy parsing and debugging.

## Prerequisites

- A ROS workspace set up with `catkin_make` or `catkin build`.
- Basic understanding of ROS, including how to create and run packages, publishers, and subscribers.
- Python 2 or 3 (depending on your ROS distribution) and `pyyaml` installed:  
  ```bash
  sudo apt-get install python-pip
  pip install pyyaml
  ```
- Ensure both the sending and receiving machines are on the same network and can reach each other’s IP addresses.

## Package Setup

1. **Create a ROS package** (already done by `catkin_create_pkg`):
   ```bash
   cd ~/catkin_ws/src
   catkin_create_pkg unitree_bridge roscpp rospy std_msgs geometry_msgs
   ```

2. **Clone or Copy Code:**
   Place the provided code files into the `unitree_bridge` package.  
   The recommended structure:
   ```
   unitree_bridge/
     CMakeLists.txt
     package.xml
     config/
       config.yaml
     scripts/
       udp_sender_node.py
       udp_receiver_node.py
     launch/
       udp_sender.launch
       udp_receiver.launch
   ```
   
3. **Install Dependencies:**
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

## Configuration File (config.yaml)

Define the topics and UDP settings in `config/config.yaml`. For example:

```yaml
udp:
  ip: "192.168.1.100"  # Target IP for sending or listening
  port: 5000           # Target UDP port

topics:
  - name: "/your_topic_name"
    type: "std_msgs/String"
  - name: "/another_topic"
    type: "geometry_msgs/Twist"
```

- `udp.ip` and `udp.port`: Set these to the receiver’s IP and desired port for sending node; for the receiver, this determines the listening port.
- `topics` array: Each entry specifies a ROS topic name and the corresponding message type.

## Sender Node

- **Purpose:** Subscribes to the listed topics, converts messages to JSON, includes the `topic_name`, and sends them via UDP.
- **Launch:**  
  ```bash
  roslaunch unitree_bridge forward.launch
  ```
  
  The `udp_sender.launch` file loads `config.yaml` and runs `udp_sender_node.py`.
  
- **What It Does:**  
  For each configured topic in `config.yaml`, the node:
  - Dynamically subscribes to that topic.
  - On receiving a message, serializes it to JSON.
  - Adds a `topic_name` field and sends it as a UDP packet to the specified IP and port.

## Receiver Node

- **Purpose:** Receives UDP JSON packets, uses the `topic_name` field to determine the correct message type from the same `config.yaml`, reconstructs the ROS message, and then publishes it to a corresponding topic (`<original_topic>_received` by default).

- **Launch:**
  ```bash
  roslaunch unitree_bridge udp_receiver.launch
  ```
  
  This will:
  - Load the same `config.yaml`.
  - Set up a UDP socket to listen on `udp.ip:udp.port`.
  - For each incoming packet, parse the JSON, find the corresponding message type, reconstruct the ROS message, and publish it on `<topic_name>_received`.

## End-to-End Example

1. **On the Receiver Machine (with IP 192.168.1.100):**
   - Edit `config.yaml` so that `udp.ip: "192.168.1.100"` and `udp.port: 5000`.
   - Run the receiver:
     ```bash
     roslaunch unitree_bridge udp_receiver.launch
     ```
   - This machine now listens for UDP packets on `0.0.0.0:5000` (or the IP from config) and publishes messages to `<topic_name>_received`.

2. **On the Sender Machine:**
   - Ensure `config.yaml` matches the receiver’s configuration and that the `udp.ip` is set to `192.168.1.100` (the receiver’s IP).
   - Run the sender:
     ```bash
     roslaunch unitree_bridge forward.launch
     ```
   - The sender node will subscribe to the configured ROS topics. When those topics have messages published (e.g., from `rostopic pub` or another node), it will send them via UDP to the receiver.

3. **Verify:**
   - On the receiver machine, check the output of:
     ```bash
     rostopic list
     rostopic echo /your_topic_name_received
     ```
   - Messages published on `/your_topic_name` on the sender side should appear on `/your_topic_name_received` on the receiver side.

## Customization

- **Changing Topics or Message Types:** Update `config.yaml` accordingly on both machines.
- **Adding More Topics:** Just append more entries to the `topics` section in `config.yaml`.
- **Different UDP Ports or IPs:** Change `udp.ip` and `udp.port` in `config.yaml`.
- **Message Conversion Logic:**  
  For complex messages, customize the `message_to_dict` and `json_to_rosmsg` functions in `udp_sender_node.py` and `udp_flexible_receiver_node.py` to handle nested fields.
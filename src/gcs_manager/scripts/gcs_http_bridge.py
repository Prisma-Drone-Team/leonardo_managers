#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from std_msgs.msg import String
import threading
import requests
import json
import os
from datetime import datetime

import numpy as np

import re

class HttpBridgeNode(Node):
    def __init__(self):
        super().__init__('http_bridge_node')

        # Publishers and Subscribers
        self.request_sub = self.create_subscription(
            String, '/gcs_result', self.handle_result, 10)
        
        self.request_sub = self.create_subscription(
            String, '/ptz_task', self.handle_ptz_task, 10)
        
        self.drone_pub = self.create_publisher(
            String, '/seed_pdt_drone/stream', 10)
        self.rover_pub = self.create_publisher(
            String, '/seed_pdt_rover/stream', 10)
        self.ptz_pub = self.create_publisher(
            String, '/seed_pdt_camera/command', 10)
        
        self.request_db = dict()
        self.sequence_db = dict()

        # MAP OF LEONARDO SECTORS (STATIC)
        self.id_matrix = np.array([
            [ 7, 7, 7, 7, 7, 5, 5, 5, 5, 4, 4, 4, 4, 4, 2, 2, 2, 2, 2, 2 ],
            [ 7, 7, 6, 7, 7, 5, 5, 5, 5, 4, 4, 4, 4, 4, 2, 2, 2, 2, 2, 2 ],
            [ 7, 7, 6, 7, 7, 5, 5, 5, 5, 4, 4, 4, 4, 4, 2, 2, 2, 2, 2, 2 ],
            [ 7, 7, 6, 6, 6, 6, 6, 5, 5, 4, 4, 4, 4, 4, 1, 1, 1, 1, 1, 1 ],
            [ 7, 7, 6, 6, 6, 5, 5, 5, 5, 4, 3, 3, 3, 4, 1, 1, 1, 1, 1, 1 ],
            [ 7, 7, 6, 6, 6, 6, 6, 5, 5, 4, 3, 3, 3, 4, 1, 1, 1, 1, 1, 1 ],
            [ 8, 8, 8, 8, 8, 8, 5, 5, 5, 3, 3, 3, 3, 3, 1, 1, 1, 1, 1, 1 ],
            [ 8, 8, 8, 8, 8, 8, 5, 5, 5, 3, 3, 3, 3, 3, 1, 1, 1, 1, 1, 1 ],
            [ 8, 8, 8, 8, 8, 8, 5, 5, 5, 3, 3, 3, 3, 3, 1, 1, 1, 1, 1, 1 ],
            [ 8, 8, 8, 8, 8, 8, 5, 5, 5, 3, 3, 3, 3, 3, 1, 1, 1, 1, 1, 1 ] ])
        
        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        # Params
        #self.declare_parameter('server_url', 'http://127.0.0.1:5000')
        self.declare_parameter('server_url', 'http://192.168.3.231:5000')
        #self.declare_parameter('server_url', 'http://192.168.3.104:5000')
        self.declare_parameter('client_id', 'unina_client')

        self.server_url = self.get_parameter('server_url').value
        self.client_id = self.get_parameter('client_id').value

        self.headers = {"Client-ID": self.client_id}

        # Start background thread for GET stream
        self.stream_thread = threading.Thread(target=self.get_stream, daemon=True)
        self.stream_thread.start()

        self.get_logger().info("HTTP Bridge Node started")

    def lookup_target_sector(self, target_frame: str) -> str:
        try:
            # Lookup transform from map -> target
            trans = self.tf_buffer.lookup_transform(
                'map',
                target_frame,
                rclpy.time.Time()
            )

            # Extract x, y coordinates
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            self.get_logger().info(f"Target pose in map: x={x:.2f}, y={y:.2f}")

            # Convert to matrix indices
            i = int(y)  # row index
            j = 19 - int(x)  # column index
            if 0 <= i < self.id_matrix.shape[0] and 0 <= j < self.id_matrix.shape[1]:
                cell_id = self.id_matrix[i, j]
                self.get_logger().info(f"Matrix indices: ({i}, {j}), ID={cell_id}")
                return cell_id
            else:
                self.get_logger().warn("Target out of matrix bounds!")
                return "None"

        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn("Transform not available")


    def handle_ptz_task(self, msg: String):
        # parse command
        task = self.parse_task_json(msg)

        # add task to the db (by id)
        self.request_db[task["task_id"]] = task

        # translate into seed commands
        self.leo_to_seed(task)



    def parse_task_json(self, json_str: str) -> dict:
        """
        Parse and validate a task JSON string according to the updated schema.
        Returns a Python dict with validated fields.
        Raises ValueError if parsing or validation fails.
        """

        if json_str.startswith("data:"):
            json_str = json_str[len("data:"):].strip()
        # Replace Python-style with JSON-style
        json_str = json_str.replace("'", '"')
        json_str = json_str.replace("None", "null")

        # replace lists (TO BE CHANGED BY LEONARDO)
        json_str = json_str.replace("(", "[").replace(")", "]")

        print("NEW JSON RECEIVED:")
        print(json_str)

        try:
            data = json.loads(json_str)
        except json.JSONDecodeError as e:
            raise ValueError(f"Invalid JSON: {e}")

        # Expected schema with defaults
        schema = {
            "task_id": 0,
            "task_type": "",
            "request_time": "",
            "deadline": "",
            "max_value": 0,
            "agent": "",
            "target_type": "",
            "target": "",
            "zone": None,  # can be null or list of vertices
        }

        # Fill missing fields with defaults
        for key, default in schema.items():
            if key not in data:
                data[key] = default
        '''
        # ---- Validation ----
        if not isinstance(data["task_id"], int):
            raise ValueError("task_id must be an integer")

        if not isinstance(data["task_type"], str):
            raise ValueError("task_type must be a string")

        if not isinstance(data["agent"], str):
            raise ValueError("agent must be a string")

        if not isinstance(data["target_type"], str):
            raise ValueError("target_type must be a string")

        if not isinstance(data["target"], str):
            raise ValueError("target must be a string")

        # request_time / deadline must match YYYY-MM-DD HH:MM:SS
        for field in ["request_time", "deadline"]:
            if not isinstance(data[field], str):
                raise ValueError(f"{field} must be a string")
            if data[field]:
                try:
                    datetime.strptime(data[field], "%Y-%m-%d %H:%M:%S")
                except ValueError:
                    raise ValueError(f"{field} must be in format YYYY-MM-DD HH:MM:SS")

        if not isinstance(data["max_value"], int):
            raise ValueError("max_value must be an integer")

        # zone can be None or list of (x,y) tuples
        if data["zone"] is not None:
            if not isinstance(data["zone"], list):
                raise ValueError("zone must be null or a list of vertices")
            for v in data["zone"]:
                if not (isinstance(v, (list, tuple)) and len(v) == 2):
                    raise ValueError("zone vertices must be 2-element lists/tuples")
                if not all(isinstance(coord, (int, float)) for coord in v):
                    raise ValueError("zone coordinates must be numbers")
        '''
        return data
    
    def leo_to_seed(self, leo_task: dict):
        msg = "null"
        if leo_task["task_type"] == "find_object":
            t1 = datetime.strptime(leo_task["request_time"], "%Y-%m-%d %H:%M:%S")
            t2 = datetime.strptime(leo_task["deadline"], "%Y-%m-%d %H:%M:%S")
            delta = t2-t1

            target = leo_task["target"]
            id = leo_task["task_id"]
            msg = f"find_object({target},{delta.total_seconds()}, {id})"
            
            ros_msg = String()
            ros_msg.data = f"{msg}"

            # to rover
            self.rover_pub.publish(ros_msg)

            # to drone
            self.drone_pub.publish(ros_msg)

        elif leo_task["task_type"] == "find_target":
            t1 = datetime.strptime(leo_task["request_time"], "%Y-%m-%d %H:%M:%S")
            t2 = datetime.strptime(leo_task["deadline"], "%Y-%m-%d %H:%M:%S")
            delta = t2-t1

            target = leo_task["target"]
            zone = self.zone_to_string(leo_task["zone"])
            id = leo_task["task_id"]
            msg = f"find_target(marker_id{target},{zone},{delta.total_seconds()},{id})"

            ros_msg = String()
            ros_msg.data = f"{msg}"

            if leo_task["agent"] == "uav":
                self.drone_pub.publish(ros_msg)
            elif leo_task["agent"] == "ugv":
                self.rover_pub.publish(ros_msg)
            #elif leo_task["agent"] == "ptz":
            #    self.ptz_pub.publish(ros_msg)

        elif leo_task["task_type"] == "ptz_collab":

            msg_zone = self.zone_to_string(leo_task["zone"])
            if self.zone_to_string(leo_task["zone"]) == "null":
                msg_zone = "none"

            target = leo_task["target"]
            id = leo_task["task_id"]

            t1 = datetime.strptime(leo_task["request_time"], "%Y-%m-%d %H:%M:%S")
            t2 = datetime.strptime(leo_task["deadline"], "%Y-%m-%d %H:%M:%S")
            delta = t2-t1

            msg = f"cover({msg_zone},{target},{id},{delta})"

            ros_msg = String()
            ros_msg.data = f"{msg}"

            self.ptz_pub.publish(ros_msg)

        elif leo_task["task_type"] == "follow_sequence":
            t1 = datetime.strptime(leo_task["request_time"], "%Y-%m-%d %H:%M:%S")
            t2 = datetime.strptime(leo_task["deadline"], "%Y-%m-%d %H:%M:%S")
            delta = t2-t1

            # add a counter for that sequence
            self.sequence_db[leo_task["task_id"]] = 1

            id = leo_task["task_id"]

            if leo_task["agent"] == "uav":

                seq = self.task_to_string(leo_task["target"],"fly_by",leo_task["task_id"])
                msg = f"follow_sequence({seq},{delta.total_seconds()},{id})"

                ros_msg = String()
                ros_msg.data = f"{msg}"

                self.drone_pub.publish(ros_msg)
            elif leo_task["agent"] == "ugv":

                seq = self.task_to_string(leo_task["target"],"move_by","fly_by",leo_task["task_id"])
                msg = f"follow_sequence({seq},{delta.total_seconds()},{id})"

                ros_msg = String()
                ros_msg.data = f"{msg}"

                self.rover_pub.publish(ros_msg)

        elif leo_task["task_type"] == "land_highest_spot":
            t1 = datetime.strptime(leo_task["request_time"], "%Y-%m-%d %H:%M:%S")
            t2 = datetime.strptime(leo_task["deadline"], "%Y-%m-%d %H:%M:%S")
            delta = t2-t1

            id = leo_task["task_id"]
            msg = f"land_highest_spot({delta.total_seconds()},{id})"

            ros_msg = String()
            ros_msg.data = f"{msg}"

            self.drone_pub.publish(ros_msg)

        elif leo_task["task_type"] == "emergency_rtb":
            t1 = datetime.strptime(leo_task["request_time"], "%Y-%m-%d %H:%M:%S")
            t2 = datetime.strptime(leo_task["deadline"], "%Y-%m-%d %H:%M:%S")
            delta = t2-t1

            id = leo_task["task_id"]
            msg = f"emergency_rtb({delta.total_seconds()},{id})"

            ros_msg = String()
            ros_msg.data = f"{msg}"

            self.rover_pub.publish(ros_msg)

        elif leo_task["task_type"] == "emergency_landing":
            t1 = datetime.strptime(leo_task["request_time"], "%Y-%m-%d %H:%M:%S")
            t2 = datetime.strptime(leo_task["deadline"], "%Y-%m-%d %H:%M:%S")
            delta = t2-t1

            id = leo_task["task_id"]
            msg = f"emergency_landing({delta.total_seconds()},{id})"

            ros_msg = String()
            ros_msg.data = f"{msg}"

            self.drone_pub.publish(ros_msg)
        else:
            task_type = leo_task["task_type"]
            self.get_logger().error(f"leo_to_seed, unknown command: {task_type}")

    def zone_to_string(self, zone) -> str:
        if not zone:
            return ""
        try:
            return ",".join(f"({x},{y})" for x, y in zone)
        except Exception:
            raise ValueError("Zone must be a list of 2-element lists/tuples with numbers")
    
    def task_to_string(self, seq, task_string, id) -> str:
        if not seq:
            return "null"
        try:
            return "[" + ",".join(f"{task_string}(marker_id{x},{id})" for x in seq) + "]"
        except Exception:
            raise ValueError("Sequence must be a list of numbers")


    # this is a thread always running
    def get_stream(self):
        """ Continuously listen to server stream and forward messages to ROS """
        url = f"{self.server_url}/mission_updates"
        try:
            response = requests.get(url, headers=self.headers, stream=True)
            for line in response.iter_lines():
                if line:
                    #print(line.decode('utf-8'))
                    msg = line.decode('utf-8')

                    # parse command
                    task = self.parse_task_json(msg)

                    # add task to the db (by id)
                    self.request_db[task["task_id"]] = task

                    # translate into seed commands
                    self.leo_to_seed(task)

                    print("CURRENT REQUEST DB:")
                    for item in self.request_db:
                        print(self.request_db[item])
                    print("---")

        except requests.exceptions.ConnectionError:
            self.get_logger().error("Connection error: could not connect to stream endpoint")
        except Exception as e:
            self.get_logger().error(f"Stream error: {e}")

    # def get_stream(self):
    #     """ Continuously listen to server stream and forward messages to ROS """
    #     url = f"{self.server_url}/mission_updates"
    #     while rclpy.ok():  # run until ROS shuts down
    #         try:
    #             with requests.get(url, headers=self.headers, stream=True, timeout=60) as response:
    #                 if response.status_code != 200:
    #                     self.get_logger().error(f"Stream error: HTTP {response.status_code}")
    #                     break

    #                 for line in response.iter_lines():
    #                     if not rclpy.ok():  # exit cleanly if shutting down
    #                         return
    #                     if line:
    #                         msg = line.decode('utf-8')

    #                         # parse command
    #                         try:
    #                             task = self.parse_task_json(msg)
    #                             self.leo_to_seed(task)
    #                         except Exception as e:
    #                             self.get_logger().error(f"Parse error: {e}")

    #                         # also publish raw stream msg
    #                         ros_msg = String()
    #                         ros_msg.data = f"[STREAM] {msg}"
    #                         self.response_pub.publish(ros_msg)

    #         except requests.exceptions.ConnectionError:
    #             self.get_logger().warn("Connection lost, retrying in 3s...")
    #             self.create_timer(3.0, lambda: None)  # small wait before retry
    #         except Exception as e:
    #             self.get_logger().error(f"Stream error: {e}")
    #             self.create_timer(3.0, lambda: None)  # wait before retry


    # this is a callback, invoked on string publishing
    def handle_result(self, msg: String):
        """
        Handle messages from ROS /gcs_result topic.
        Expect strings formatted as (id, timestamp, path_to_image).
        Example:
            {"type": "post_metadata", "data": {...}}
            {"type": "post_file", "file": "/path/to/file.png", "metadata": {...}}
        """
        
        s = msg.data
        s = s.strip("()")

        # Split by comma, then strip spaces
        parts = [p.strip() for p in s.split(",")]

        str_id = parts[0]
        id = int(str_id)
        timestamp = parts[1]
        file_path = parts[2]

        #print("CURRENT REQUEST DB (RESULT):")
        #for item in self.request_db:
        #    print(self.request_db[item])
        #    print(item)
        #    if(item == int(std_id)):
        #        print("id found:",std_id)
        #        id = item
        #print("---")

        if(id == 0):
            return

        json_reply = {}
        have_file = False
        print(id)
        print(timestamp)
        print(file_path)
        res_task = self.request_db[id]
        print(res_task["task_type"])
        if self.request_db[id]["task_type"] == "find_object":
            # OK
            json_reply = {
                "task_id": id,
                "task_type": self.request_db[id]["task_type"],
                "target": self.request_db[id]["target"],
                "request_time": self.request_db[id]["request_time"],
                "completion_time": timestamp,
                "result": "None"
            }
            have_file = True
        elif self.request_db[id]["task_type"] == "find_target":
            # OK
            sect = self.lookup_target_sector(str(self.request_db[id]["target"]))
            json_reply = {
                "task_id": id,
                "task_type": self.request_db[id]["task_type"],
                "target": self.request_db[id]["target"],
                "request_time": self.request_db[id]["request_time"],
                "completion_time": timestamp,
                "result": sect
            }
            have_file = True
        elif self.request_db[id]["task_type"] == "ptz_collab":
            # OK
            json_reply = {
                "task_id": id,
                "task_type": self.request_db[id]["task_type"],
                "target": "",
                "request_time": self.request_db[id]["request_time"],
                "completion_time": timestamp,
                "result": "None"
            }
        elif self.request_db[id]["task_type"] == "follow_sequence":
            # OK (BUT SYNTAX OF TARGET SHOULD BE VERIFIED)
            json_reply = {
                "task_id": id,
                "task_type": self.request_db[id]["task_type"],
                "target": self.request_db[id]["target"][self.sequence_db[id]],
                "request_time": self.request_db[id]["request_time"],
                "completion_time": timestamp,
                "result": "None"
            }
            have_file = True
            self.sequence_db[id] = self.sequence_db[id] + 1
        elif self.request_db[id]["task_type"] == "land_highest_spot":
            # OK
            json_reply = {
                "task_id": id,
                "task_type": self.request_db[id]["task_type"],
                "target": "None",
                "request_time": self.request_db[id]["request_time"],
                "completion_time": timestamp,
                "result": "None"
            }
        elif self.request_db[id]["task_type"] == "emergency_rtb":
            # OK
            json_reply = {
                "task_id": id,
                "task_type": self.request_db[id]["task_type"],
                "target": "None",
                "request_time": self.request_db[id]["request_time"],
                "completion_time": timestamp,
                "result": "None"
            }
        elif self.request_db[id]["task_type"] == "emergency_landing":
            # OK
            json_reply = {
                "task_id": id,
                "task_type": self.request_db[id]["task_type"],
                "target": "None",
                "request_time": self.request_db[id]["request_time"],
                "completion_time": timestamp,
                "result": "None"
            }

        try:
            headers = {"Client-ID": "unina_gcs"} 
            url = f"{self.server_url}/upload_file"

            if not have_file:
                response = requests.post(url, headers=headers, json=json_reply)  # Send metadata as JSON message

            else:
                if not file_path or not os.path.exists(file_path):
                    self.get_logger().error("File missing or path invalid in request")
                    return

                files = {
                    'file': open(file_path, 'rb'),
                    'metadata': (None, json.dumps(json_reply), 'application/json')
                }
                response = requests.post(url, headers=self.headers, files=files)

            #ros_msg = String()
            #ros_msg.data = f"[RESPONSE] {response.text}"
            #self.response_pub.publish(ros_msg)
            print("result sent, server replied:",response.json())

        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"HTTP request failed: {e}")



def main(args=None):
    rclpy.init(args=args)
    node = HttpBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down HTTP Bridge Node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


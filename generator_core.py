#! /usr/bin/python3

import jinja2
import re
from typing import Tuple, Dict, List

### UTILS ###

def _camel_to_snake(name: str) -> str:
    """
    Converts CamelCase to snake_case.
    
    Examples:
        PointCloud2 -> point_cloud2
        TwistStamped -> twist_stamped
        IMU -> imu
    """
    if not name:
        return name
    
    # Proccess abbreviations in the beginning
    # Example: IMUData -> imu_data
    
    s1 = re.sub(r'([a-z])([A-Z])', r'\1_\2', name)
    s2 = re.sub(r'([A-Z])([A-Z][a-z])', r'\1_\2', s1)   
    return s2.lower()

def convert_ros_format_generic(ros_type: str) -> Tuple[str, str]:
    """
    Convert ROS type to path and extract package.
    Works for msg, srv, and action types.
    
    Returns:
        tuple: (path, package_name)
    """
    # Normalize input
    normalized = ros_type.replace('/', '::')
    
    # Check for known ROS types
    for ros_type in ['msg', 'srv', 'action']:
        pattern = f'::{ros_type}::'
        if pattern in normalized:
            package, _, message = normalized.split('::')
            message_snake = _camel_to_snake(message)
            return f"{package}/{ros_type}/{message_snake}", package
    
    # If no type specified, try to guess
    if '::' in normalized:
        parts = normalized.split('::')
        if len(parts) == 3:
            # Assume middle part is type (even if not standard)
            package, ros_type, message = parts
            message_snake = _camel_to_snake(message)
            return f"{package}/{ros_type}/{message_snake}", package
        elif len(parts) == 2:
            # No type specified, assume 'msg'
            package, message = parts
            message_snake = _camel_to_snake(message)
            return f"{package}/msg/{message_snake}", package
    
    # Fallback
    return ros_type, "unknown"

def has_keys(dictionary, keys):
    return all(key in dictionary for key in keys)

#############

class Ros2PkgGenerator:
    def __init__(self, config = {}):
        
        if config != {}:
            self.config = config
            # TODO: add items correctly (with add methods)
        else:
            self.config = {
                "node_filename": "node",
                'node_classname': 'MyNode',
                'node_name': 'my_node',
                'is_component': True,
                'include_pkgs': set(),
                'includes': set(),
                'params': [],
                'publishers': [],
                'subscribers': [],
                'timers': [],
                'services': [],
                'clients': [],
                'action_servers': [],
                'action_clients': [],
                'sync_subscribers': [],
                "package_name": "my_package",
                "cmake_target_name": "my_library"
            }
        
        self.env = jinja2.Environment(
            loader=jinja2.FileSystemLoader('templates'),
            trim_blocks=True,
            lstrip_blocks=True
        )
        
        self.templates = {
            'hpp': self.env.get_template('node.hpp.j2'),
            'cpp': self.env.get_template('node.cpp.j2'),
            'xml': self.env.get_template('package.xml.j2'),
            'cmake': self.env.get_template('CMakeLists.txt.j2'),
        }
        
        self.cb_types = {"Object": "obj", "UniquePtr": "uptr", "SharedPtr": "sptr", "ConstSharedPtr": "csptr"}
    
    def __getitem__(self, key):
        return self.config[key]
    
    def __setitem__(self, key, value):
        self.config[key] = value
    
    def __delitem__(self, key):
        del self.config[key]
    
    def is_name_busy(self, var_name: str):
        name_busy = False
        name_busy = name_busy or any(p["var_name"] == var_name for p in self.config.get("publishers", []))
        if not name_busy:
            name_busy = name_busy or any(s["var_name"] == var_name for s in self.config.get("subscribers", []))
        if not name_busy:
            name_busy = name_busy or any(t["var_name"] == var_name for t in self.config.get("timers", []))
        if not name_busy:
            name_busy = name_busy or any(s["var_name"] == var_name for s in self.config.get("services", []))
        if not name_busy:
            name_busy = name_busy or any(c["var_name"] == var_name for c in self.config.get("clients", []))
        if not name_busy:
            name_busy = name_busy or any(action_srv["var_name"] == var_name for action_srv in self.config.get("action_servers", []))
        if not name_busy:
            name_busy = name_busy or any(action_client["var_name"] == var_name for action_client in self.config.get("action_clients", []))
        # TODO: add checks for parameters
        return name_busy
    
    def __update_includes(self):
        deps = set()
        includes = set()
        
        for s in self.config.get("subscribers", []):
            deps.update(s["depends"])
            includes.update(s["includes"])
        
        for p in self.config.get("publishers", []):
            deps.update(p["depends"])
            includes.update(p["includes"])
        
        for s in self.config.get("services", []):
            deps.update(s["depends"])
            includes.update(s["includes"])
        
        for c in self.config.get("clients", []):
            deps.update(c["depends"])
            includes.update(c["includes"])
        
        action_servers = self.config.get("action_servers", [])
        for action_srv in action_servers:
            deps.update(action_srv["depends"])
            includes.update(action_srv["includes"])
        
        action_clients = self.config.get("action_clients", [])
        for action_client in action_clients:
            deps.update(action_client["depends"])
            includes.update(action_client["includes"])
        
        if len(action_servers) + len(action_clients) > 0:
            deps.add("rclcpp_action")
            includes.add("rclcpp_action/rclcpp_action.hpp")
        
        sync_subscribers = self.config.get("sync_subscribers", [])
        for sync_sub in sync_subscribers:
            for s in sync_sub["subs"]:
                deps.update(s["depends"])
                includes.update(s["includes"])
            if sync_sub["sync_policy"] == "ExactTime":
                includes.add("message_filters/sync_policies/exact_time.h")
            elif sync_sub["sync_policy"] == "ApproximateTime":
                includes.add("message_filters/sync_policies/approximate_time.h")
            elif sync_sub["sync_policy"] == "ApproximateEpsilonTime":
                includes.add("message_filters/sync_policies/approximate_epsilon_time.h")
        
        if len(sync_subscribers) > 0:
            deps.add("message_filters")
            includes.add("message_filters/subscriber.h")
            includes.add("message_filters/synchronizer.h")

        self.config['include_pkgs'] = deps
        self.config['includes'] = includes
    
    def remove_item(self, var_name: str, dict_name: str):
        index = next([i for i, item in enumerate(self.config[dict_name]) if item["var_name"] == var_name], -1)
        if index >= 0:
            del self.config[dict_name][index]
    
    def remove_items(self, var_name: List[str], dict_name: str):
        self.config[dict_name] = [item for item in self.config[dict_name] if item["var_name"] not in var_name]
    
    def get_item(self, var_name: str, dict_name: str):
        index = next(iter([i for i, item in enumerate(self.config[dict_name]) if item["var_name"] == var_name]), -1)
        return (None if index < 0 else self.config[dict_name][index], index)
    
    def update_item(self, item_info: Dict, index: int, dict_name: str, add_method = None):
        if index >= 0 and index < len(self.config[dict_name]):
            del self.config[dict_name][index]
            add_method(item_info)
    
    # Subscriptions
    
    def add_subscription(self, sub_info: Dict):
        if not has_keys(sub_info, ["msg_type", "var_name", "topic", "callback", "callback_arg_type", "qos"]):
            return
        if self.is_name_busy(sub_info["var_name"]):
            return
        
        msg_type_snake, msg_pkg = convert_ros_format_generic(sub_info["msg_type"])
        msg_include = f"{msg_type_snake}.hpp"
        sub_info["depends"] = [msg_pkg]
        sub_info["includes"] = [msg_include]
        
        self.config["subscribers"].append(sub_info)
    
    def remove_subscription(self, sub_var_name: str):
        self.remove_item(sub_var_name, "subscribers")
    
    def remove_subscriptions(self, sub_var_names: List[str]):
        self.remove_items(sub_var_names, "subscribers")
    
    def get_subscription(self, sub_var_name: str) -> Tuple[dict, int]:
        return self.get_item(sub_var_name, "subscribers")
    
    def update_subscription(self, sub_info: Dict, index: int):
        self.update_item(sub_info, index, "subscribers", self.add_subscription)
    
    # Publishers
    
    def add_publisher(self, pub_info: Dict):
        if not has_keys(pub_info, ["msg_type", "var_name", "topic", "qos"]):
            return
        if self.is_name_busy(pub_info["var_name"]):
            return
        
        msg_type_snake, msg_pkg = convert_ros_format_generic(pub_info["msg_type"])
        msg_include = f"{msg_type_snake}.hpp"
        pub_info["depends"] = [msg_pkg]
        pub_info["includes"] = [msg_include]
        
        self.config["publishers"].append(pub_info)
    
    def remove_publisher(self, pub_var_name: str):
        self.remove_item(pub_var_name, "publishers")
    
    def remove_publishers(self, pub_var_names: List[str]):
        self.remove_items(pub_var_names, "publishers")
    
    def get_publisher(self, pub_var_name: str) -> Tuple[dict, int]:
        return self.get_item(pub_var_name, "publishers")
    
    def update_publisher(self, pub_info: Dict, index: int):
        self.update_item(pub_info, index, "publishers", self.add_publisher)
    
    # Timers
    
    def add_timer(self, timer_info: Dict):
        if not has_keys(timer_info, ["var_name", "period", "callback"]):
            return
        if self.is_name_busy(timer_info["var_name"]):
            return
        self.config["timers"].append(timer_info)
    
    def remove_timer(self, timer_var_name: str):
        self.remove_item(timer_var_name, "timers")
    
    def remove_timers(self, timer_var_names: List[str]):
        self.remove_items(timer_var_names, "timers")
    
    def get_timer(self, timer_var_name: str) -> Tuple[dict, int]:
        return self.get_item(timer_var_name, "timers")
    
    def update_timer(self, timer_info: Dict, index: int):
        self.update_item(timer_info, index, "timers", self.add_timer)
    
    # Params
    
    def add_param(self, param_info: Dict):
        if not has_keys(param_info, ["name", "type", "default"]):
            return
        if any(p["name"] == param_info["name"] for p in self.config["params"]):
            return
        self.config["params"].append(param_info)
    
    def remove_param(self, param_name: str):
        index = next([i for i, p in enumerate(self.config["params"]) if p["name"] == param_name], -1)
        if index >= 0:
            del self.config["params"][index]
    
    def remove_params(self, param_names: List[str]):
        self.config["params"] = [p for p in self.config["params"] if p["name"] not in param_names]
    
    def get_param(self, param_name: str) -> Tuple[dict, int]:
        index = next(iter([i for i, p in enumerate(self.config["params"]) if p["name"] == param_name]), -1)
        return (None if index < 0 else self.config["params"][index], index)
    
    def update_param(self, param_info: Dict, index: int):
        self.update_item(param_info, index, "params", self.add_param)
    
    # Service servers
    
    def add_service(self, srv_info: Dict):
        if not has_keys(srv_info, ["type", "var_name", "name", "callback"]):
            return
        if self.is_name_busy(srv_info["var_name"]):
            return
        
        srv_type_snake, srv_pkg = convert_ros_format_generic(srv_info["type"])
        srv_include = f"{srv_type_snake}.hpp"
        srv_info["depends"] = [srv_pkg]
        srv_info["includes"] = [srv_include]
        
        self.config["services"].append(srv_info)
    
    def remove_service(self, srv_var_name: str):
        self.remove_item(srv_var_name, "services")

    def remove_services(self, srv_var_names: List[str]):
        self.remove_items(srv_var_names, "services")

    def get_service(self, srv_var_name: str) -> Tuple[dict, int]:
        return self.get_item(srv_var_name, "services")

    def update_service(self, srv_info: Dict, index: int):
        self.update_item(srv_info, index, "services", self.add_service)
    
    # Service clients
    
    def add_client(self, client_info: Dict):
        if not has_keys(client_info, ["type", "var_name", "srv_name"]):
            return
        if self.is_name_busy(client_info["var_name"]):
            return
        
        client_type_snake, client_pkg = convert_ros_format_generic(client_info["type"])
        client_include = f"{client_type_snake}.hpp"
        client_info["depends"] = [client_pkg]
        client_info["includes"] = [client_include]
        
        self.config["clients"].append(client_info)
    
    def remove_client(self, client_var_name: str):
        self.remove_item(client_var_name, "clients")

    def remove_clients(self, client_var_names: List[str]):
        self.remove_items(client_var_names, "clients")

    def get_client(self, client_var_name: str) -> Tuple[dict, int]:
        return self.get_item(client_var_name, "clients")

    def update_client(self, client_info: Dict, index: int):
        self.update_item(client_info, index, "clients", self.add_client)
    
    # Action servers
    
    def add_action_server(self, action_srv_info: Dict):
        if not has_keys(action_srv_info, ["name", "var_name", "type", "handle_goal", "handle_cancel", "handle_accepted", "execute"]):
            return
        if self.is_name_busy(action_srv_info["var_name"]):
            return
        action_srv_type_snake, action_srv_pkg = convert_ros_format_generic(action_srv_info["type"])
        action_srv_include = f"{action_srv_type_snake}.hpp"
        action_srv_info["depends"] = [action_srv_pkg]
        action_srv_info["includes"] = [action_srv_include]
        
        self.config["action_servers"].append(action_srv_info)

    def remove_action_server(self, action_srv_var_name: str):
        self.remove_item(action_srv_var_name, "action_servers")

    def remove_action_servers(self, action_srv_var_names: List[str]):
        self.remove_items(action_srv_var_names, "action_servers")
        
    def get_action_server(self, action_srv_var_name: str) -> Tuple[dict, int]:
        return self.get_item(action_srv_var_name, "action_servers")
    
    def update_action_server(self, action_srv_info: Dict, index: int):
        self.update_item(action_srv_info, index, "action_servers", self.add_action_server)
    
    # Action clients
    
    def add_action_client(self, action_client_info: Dict):
        if not has_keys(action_client_info, ["srv_name", "var_name", "type", "goal_response_callback", "feedback_callback", "result_callback"]):
            return
        if self.is_name_busy(action_client_info["var_name"]):
            return
        action_client_type_snake, action_client_pkg = convert_ros_format_generic(action_client_info["type"])
        action_client_include = f"{action_client_type_snake}.hpp"
        action_client_info["depends"] = [action_client_pkg]
        action_client_info["includes"] = [action_client_include]
        
        self.config["action_clients"].append(action_client_info)

    def remove_action_client(self, action_client_var_name: str):
        self.remove_item(action_client_var_name, "action_clients")

    def remove_action_clients(self, action_client_var_names: List[str]):
        self.remove_items(action_client_var_names, "action_clients")
        
    def get_action_client(self, action_client_var_name: str) -> Tuple[dict, int]:
        return self.get_item(action_client_var_name, "action_clients")
    
    def update_action_client(self, action_client_info: Dict, index: int):
        self.update_item(action_client_info, index, "action_clients", self.add_action_client)
    
    # Synchronized subscriptions (message filters)
    
    def add_sync_subscription(self, sync_sub_info: Dict):
        if not has_keys(sync_sub_info, ["var_name", "callback", "sync_policy", "queue_size", "subs"]):
            return
        if self.is_name_busy(sync_sub_info["var_name"]):
            return
        
        for s in sync_sub_info["subs"]:
            msg_type_snake, msg_pkg = convert_ros_format_generic(s["msg_type"])
            msg_include = f"{msg_type_snake}.hpp"
            s["depends"] = [msg_pkg]
            s["includes"] = [msg_include]

        if "sync_subscribers" not in self.config:
            self.config["sync_subscribers"] = []
        self.config["sync_subscribers"].append(sync_sub_info)
    
    def remove_sync_subscription(self, sync_sub_var_name: str):
        self.remove_item(sync_sub_var_name, "sync_subscribers")
    
    def remove_sync_subscriptions(self, sync_sub_var_names: List[str]):
        self.remove_items(sync_sub_var_names, "sync_subscribers")
    
    def get_sync_subscription(self, sync_sub_var_name: str) -> Tuple[dict, int]:
        return self.get_item(sync_sub_var_name, "sync_subscribers")
    
    def update_sync_subscription(self, sync_sub_info: Dict, index: int):
        self.update_item(sync_sub_info, index, "sync_subscribers", self.add_sync_subscription)
    
    def generate_files(self):
        self.__update_includes()
        self.config["advertisement"] = "The package was created using ros2-package-generator: https://github.com/tamerlan-b/ros2-package-generator.git"
        return {
            f'{self.config["node_filename"]}.hpp': self.templates['hpp'].render(**self.config), 
            f'{self.config["node_filename"]}.cpp': self.templates['cpp'].render(**self.config), 
            "CMakeLists.txt": self.templates['cmake'].render(**self.config), 
            'package.xml': self.templates['xml'].render(**self.config),
        }

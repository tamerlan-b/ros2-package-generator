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
        name_busy = name_busy or any(p["var_name"] == var_name for p in self.config["publishers"])
        if not name_busy:
            name_busy = name_busy or any(s["var_name"] == var_name for s in self.config["subscribers"])
        if not name_busy:
            name_busy = name_busy or any(t["var_name"] == var_name for t in self.config["timers"])
        if not name_busy:
            name_busy = name_busy or any(s["var_name"] == var_name for s in self.config["services"])
        # TODO: add checks for parameters
        return name_busy
    
    # Subscriptions
    
    def add_subscription(self, sub_info):
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
        index = next([i for i, s in enumerate(self.config["subscribers"]) if s["var_name"] == sub_var_name], -1)
        if index >= 0:
            del self.config["subscribers"][index]
    
    def remove_subscriptions(self, sub_var_names: List[str]):
        self.config["subscribers"] = [s for s in self.config["subscribers"] if s["var_name"] not in sub_var_names]
    
    def get_subscription(self, sub_var_name) -> Tuple[dict, int]:
        index = next(iter([i for i, s in enumerate(self.config["subscribers"]) if s["var_name"] == sub_var_name]), -1)
        return (None if index < 0 else self.config["subscribers"][index], index)
    
    def update_subscription(self, sub_info: Dict, index: int):
        if index >= 0 and index < len(self.config["subscribers"]):
            del self.config["subscribers"][index]
            self.add_subscription(sub_info)
    
    # Publishers
    
    def add_publisher(self, pub_info):
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
        index = next([i for i, p in enumerate(self.config["publishers"]) if p["var_name"] == pub_var_name], -1)
        if index >= 0:
            del self.config["publishers"][index]
    
    def remove_publishers(self, pub_var_names: List[str]):
        self.config["publishers"] = [p for p in self.config["publishers"] if p["var_name"] not in pub_var_names]
    
    def get_publisher(self, pub_var_name) -> Tuple[dict, int]:
        index = next(iter([i for i, p in enumerate(self.config["publishers"]) if p["var_name"] == pub_var_name]), -1)
        return (None if index < 0 else self.config["publishers"][index], index)
    
    def update_publisher(self, pub_info: Dict, index: int):
        if index >= 0 and index < len(self.config["publishers"]):
            del self.config["publishers"][index]
            self.add_publisher(pub_info)
    
    # Timers
    
    def add_timer(self, timer_info):
        if not has_keys(timer_info, ["var_name", "period", "callback"]):
            return
        if self.is_name_busy(timer_info["var_name"]):
            return
        self.config["timers"].append(timer_info)
    
    def remove_timer(self, timer_var_name: str):
        index = next([i for i, t in enumerate(self.config["timers"]) if t["var_name"] == timer_var_name], -1)
        if index >= 0:
            del self.config["timers"][index]
    
    def remove_timers(self, timer_var_names: List[str]):
        self.config["timers"] = [t for t in self.config["timers"] if t["var_name"] not in timer_var_names]
    
    def get_timer(self, timer_var_name) -> Tuple[dict, int]:
        index = next(iter([i for i, t in enumerate(self.config["timers"]) if t["var_name"] == timer_var_name]), -1)
        return (None if index < 0 else self.config["timers"][index], index)
    
    def update_timer(self, timer_info: Dict, index: int):
        if index >= 0 and index < len(self.config["timers"]):
            del self.config["timers"][index]
            self.add_timer(timer_info)
    
    # Params
    
    def add_param(self, param_info):
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
    
    def get_param(self, param_var_name) -> Tuple[dict, int]:
        index = next(iter([i for i, p in enumerate(self.config["params"]) if p["var_name"] == param_var_name]), -1)
        return (None if index < 0 else self.config["params"][index], index)
    
    def update_param(self, param_info: Dict, index: int):
        if index >= 0 and index < len(self.config["params"]):
            del self.config["params"][index]
            self.add_param(param_info)
    
    # Services
    
    def add_service(self, srv_info):
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
        index = next([i for i, s in enumerate(self.config["services"]) if s["var_name"] == srv_var_name], -1)
        if index >= 0:
            del self.config["services"][index]

    def remove_services(self, srv_var_names: List[str]):
        self.config["services"] = [s for s in self.config["services"] if s["var_name"] not in srv_var_names]

    def get_service(self, srv_var_name) -> Tuple[dict, int]:
        index = next(iter([i for i, s in enumerate(self.config["services"]) if s["var_name"] == srv_var_name]), -1)
        return (None if index < 0 else self.config["services"][index], index)

    def update_service(self, srv_info: Dict, index: int):
        if index >= 0 and index < len(self.config["services"]):
            del self.config["services"][index]
            self.add_service(srv_info)
    
    def __update_includes(self):
        deps = set()
        includes = set()
        
        for s in self.config["subscribers"]:
            deps.update(s["depends"])
            includes.update(s["includes"])
        
        for p in self.config["publishers"]:
            deps.update(p["depends"])
            includes.update(p["includes"])
        
        for s in self.config["services"]:
            deps.update(s["depends"])
            includes.update(s["includes"])

        self.config['include_pkgs'] = deps
        self.config['includes'] = includes
    
    def generate_files(self):
        self.__update_includes()
        self.config["advertisement"] = "The package was created using ros2-package-generator: https://github.com/tamerlan-b/ros2-package-generator.git"
        return {
            f'{self.config["node_filename"]}.hpp': self.templates['hpp'].render(**self.config), 
            f'{self.config["node_filename"]}.cpp': self.templates['cpp'].render(**self.config), 
            "CMakeLists.txt": self.templates['cmake'].render(**self.config), 
            'package.xml': self.templates['xml'].render(**self.config),
        }

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

def split_ros2_type(ros_msg_type: str) -> Tuple[str, str, str]:
    parts = ros_msg_type.split('/')
    pkg_name = parts[0]
    interface_type = parts[1]   # 'msg', 'srv' or 'action'
    type = parts[2]
    return pkg_name, interface_type, type

def to_cpp_include(pkg_name: str, interface_type: str, type: str) -> str:
    return f"{pkg_name}/{interface_type}/{_camel_to_snake(type)}"

def convert_ros_format_generic(ros_type: str) -> Tuple[str, str]:
    """
    Convert ROS type to path and extract package.
    Works for msg, srv, and action types.

    Args:
        ros_type (str): ROS2 interface. For example: "sensor_msgs/msg/Image", "sensor_msgs/srv/SetCameraInfo"

    Returns:
        Tuple[str, str]: (c++ include header, package_name). Example: ("sensor_msgs/msg/image.hpp", "sensor_msgs")
    """

    pkg_name, interface_type, type = split_ros2_type(ros_type)
    return to_cpp_include(pkg_name, interface_type, type), pkg_name

def to_py_import(ros_type: str) -> Tuple[str, str]:
    """
    Convert ROS type to a python module and class name.
    Works for msg, srv, and action types.

    Args:
        ros_type (str): ROS2 interface. For example: "sensor_msgs/msg/Image"

    Returns:
        Tuple[str, str]: (python module, class name). Example: ("sensor_msgs.msg", "Image")
    """
    pkg_name, interface_type, type = split_ros2_type(ros_type)
    return f"{pkg_name}.{interface_type}", type

# Maps the C++-flavored parameter type strings used by the UI/templates to the
# rclpy ParameterValue accessor that must be used to read them back.
PARAM_PY_ACCESSORS = {
    "bool": "bool_value",
    "int": "integer_value",
    "double": "double_value",
    "std::string": "string_value",
    "std::vector<int>": "integer_array_value",
    "std::vector<double>": "double_array_value",
    "std::vector<std::string>": "string_array_value",
}

def cpp_param_default_to_py(cpp_type: str, default: str) -> str:
    """
    Convert a C++ parameter default literal (as stored by the shared ParamManager)
    into an equivalent Python literal for rclpy's declare_parameter().

    bool: true/false -> True/False
    vector<...>: {1, 2, 3} -> [1, 2, 3]
    int/double/string: unchanged (C++ and Python share that literal syntax)
    """
    default = str(default).strip()
    if cpp_type == "bool":
        return "True" if default.lower() in ("true", "1") else "False"
    if cpp_type.startswith("std::vector<"):
        return "[" + default.strip().strip("{}").strip() + "]"
    return default

def has_keys(dictionary, keys):
    return all(key in dictionary for key in keys)

#############


class NodeItemManagerBase:
    
    def __init__(self, sub_dict: Dict, unique_key: str = 'var_name'):
        self.dict = sub_dict
        self.unique_key = unique_key
    
    def __remove_item(self, unique_value: str):
        index = next([i for i, item in enumerate(self.dict) if item[self.unique_key] == unique_value], -1)
        if index >= 0:
            del self.dict[index]
    
    def __remove_items(self, unique_value: List[str]):
        indices = [i for i, item in enumerate(self.dict) if item[self.unique_key] in unique_value]
        for i in reversed(indices):
            if i >= 0:
                del self.dict[i]
    
    def __get_item(self, unique_value: str):
        index = next(iter([i for i, item in enumerate(self.dict) if item[self.unique_key] == unique_value]), -1)
        return (None if index < 0 else self.dict[index], index)
    
    def __update_item(self, item_info: Dict, index: int, add_method = None):
        if index >= 0 and index < len(self.dict):
            del self.dict[index]
            add_method(item_info)
    
    def add(self, info: Dict):
        self.dict.append(info)
    
    def remove(self, var_name: str):
        self.__remove_item(var_name)
    
    def remove_many(self, var_names: List[str]):
        self.__remove_items(var_names)
    
    def get(self, var_name: str) -> Tuple[dict, int]:
        return self.__get_item(var_name)
    
    def update(self, info: Dict, index: int):
        self.__update_item(info, index, self.add)
    
    def clear(self):
        self.dict.clear()
    
    # TODO: make abstract
    def validate(self, info: Dict, skip_name: bool = False) -> Tuple[bool, str]:
        return True, ""

class SubManager(NodeItemManagerBase):
    def __init__(self, sub_dict: Dict):
        super().__init__(sub_dict)

    def add(self, info: Dict):
        if not has_keys(info, ["msg_type", "var_name", "topic", "callback", "qos"]):
            return
        if not ("callback_arg_type" in info.keys() or "cpp_type" in info.keys()):
            return
        # TODO: add busy_method
        # if self.is_name_busy(info["var_name"]):
        #     return
        
        msg_type_snake, msg_pkg = convert_ros_format_generic(info["msg_type"])
        msg_include = f"{msg_type_snake}.hpp"
        info["depends"] = [msg_pkg]
        info["includes"] = [msg_include]
        info["py_module"], info["py_class"] = to_py_import(info["msg_type"])

        super().add(info)

    def validate(self, info: Dict, skip_name: bool = False) -> Tuple[bool, str]:
        if info.get("msg_type", None) == None:
            return False, "Message type is empty"
        if info.get("var_name", "") == "":
            return False, "Variable name type is empty"
        if info.get("topic", "") == "":
            return False, "Topic name is empty"
        if info.get("callback", "") == "":
            return False, "Callback method name is empty"
        if info.get("qos", {}) == {}:
            return False, "QoS is empty"
        if "cpp_type" in info.keys() and info["cpp_type"] == "":
            return False, "C++ is empty"
        # if not skip_name and self.is_name_busy(info["var_name"]):
        #     return False, "Variable's name is busy"
        return True, ""

class PubManager(NodeItemManagerBase):
    def __init__(self, pub_dict: Dict):
        super().__init__(pub_dict)
    
    def add(self, info: Dict):
        if not has_keys(info, ["msg_type", "var_name", "topic", "qos"]):
            return
        # TODO: return
        # if self.is_name_busy(info["var_name"]):
        #     return
        msg_type_snake, msg_pkg = convert_ros_format_generic(info["msg_type"])
        msg_include = f"{msg_type_snake}.hpp"
        info["depends"] = [msg_pkg]
        info["includes"] = [msg_include]
        info["py_module"], info["py_class"] = to_py_import(info["msg_type"])

        super().add(info)

    def validate(self, info: Dict, skip_name: bool = False) -> Tuple[bool, str]:
        if info.get("msg_type", None) == None:
            return False, "Message type is empty"
        if info.get("var_name", "") == "":
            return False, "Variable name type is empty"
        if info.get("topic", "") == "":
            return False, "Topic name is empty"
        if info.get("qos", {}) == {}:
            return False, "QoS is empty"
        # if not skip_name and self.is_name_busy(info["var_name"]):
        #     return False, "Variable's name is busy"
        return True, ""

class TimerManager(NodeItemManagerBase):
    def __init__(self, timer_dict: Dict):
        super().__init__(timer_dict)
    
    def add(self, info: Dict):
        if not has_keys(info, ["var_name", "period", "callback"]):
            return
        # TODO: return
        # if self.is_name_busy(info["var_name"]):
        #     return
        super().add(info)

    def validate(self, info: Dict, skip_name: bool = False) -> Tuple[bool, str]:
        if info.get("var_name", "") == "":
            return False, "Variable name type is empty"
        if info.get("callback", "") == "":
            return False, "Callback method name is empty"
        # if not skip_name and self.is_name_busy(info["var_name"]):
        #     return False, "Variable's name is busy"
        return True, ""

class ParamManager(NodeItemManagerBase):
    def __init__(self, timer_dict: Dict):
        super().__init__(timer_dict, unique_key="name")
    
    def add(self, info: Dict):
        if not has_keys(info, ["name", "type", "default"]):
            return
        # TODO: return
        # if any(p["name"] == info["name"] for p in self.config["params"]):
        #     return
        super().add(info)
    
    def validate(self, info: Dict, skip_name: bool = False) -> Tuple[bool, str]:
        if info.get("name", "") == "":
            return False, "Param name is empty"
        if info.get("type", None) == None:
            return False, "Param type is empty"
        if info.get("default", None) == None:
            return False, "Default value is empty"
        return True, ""

class ServiceManager(NodeItemManagerBase):
    def __init__(self, service_dict: Dict):
        super().__init__(service_dict)
    
    def add(self, info: Dict):
        if not has_keys(info, ["type", "var_name", "name", "callback"]):
            return
        # TODO: check
        # if self.is_name_busy(info["var_name"]):
        #     return
        
        srv_type_snake, srv_pkg = convert_ros_format_generic(info["type"])
        srv_include = f"{srv_type_snake}.hpp"
        info["depends"] = [srv_pkg]
        info["includes"] = [srv_include]
        info["py_module"], info["py_class"] = to_py_import(info["type"])
        super().add(info)
    
    def validate(self, info: Dict, skip_name: bool = False) -> Tuple[bool, str]:
        if info.get("var_name", "") == "":
            return False, "Variable's name is empty"
        if info.get("type", None) == None:
            return False, "Service type is empty"
        if info.get("name", "") == "":
            return False, "Service name is empty"
        if info.get("callback", "") == "":
            return False, "Service callback is empty"
        # if not skip_name and self.is_name_busy(info["var_name"]):
        #     return False, "Variable's name is busy"
        return True, ""

class ClientManager(NodeItemManagerBase):
    def __init__(self, client_dict: Dict):
        super().__init__(client_dict)
    
    def add(self, info: Dict):
        if not has_keys(info, ["type", "var_name", "srv_name"]):
            return
        # TODO: check
        # if self.is_name_busy(info["var_name"]):
        #     return
        
        client_type_snake, client_pkg = convert_ros_format_generic(info["type"])
        client_include = f"{client_type_snake}.hpp"
        info["depends"] = [client_pkg]
        info["includes"] = [client_include]
        info["py_module"], info["py_class"] = to_py_import(info["type"])
        super().add(info)
    
    def validate(self, info: Dict, skip_name: bool = False) -> Tuple[bool, str]:
        if info.get("var_name", "") == "":
            return False, "Variable's name is empty"
        if info.get("type", None) == None:
            return False, "Service type is empty"
        if info.get("srv_name", "") == "":
            return False, "Service name is empty"
        # if not skip_name and self.is_name_busy(info["var_name"]):
        #     return False, "Variable's name is busy"
        return True, ""

class ActionServerManager(NodeItemManagerBase):
    def __init__(self, action_server_dict: Dict):
        super().__init__(action_server_dict)
    
    def add(self, info: Dict):
        if not has_keys(info, ["name", "var_name", "type", "handle_goal", "handle_cancel", "handle_accepted", "execute"]):
            return
        # TODO: check
        # if self.is_name_busy(info["var_name"]):
        #     return
        action_srv_type_snake, action_srv_pkg = convert_ros_format_generic(info["type"])
        action_srv_include = f"{action_srv_type_snake}.hpp"
        info["depends"] = [action_srv_pkg]
        info["includes"] = [action_srv_include]
        super().add(info)
    
    def validate(self, info: Dict, skip_name: bool = False) -> Tuple[bool, str]:
        if info.get("name", "") == "":
            return False, "Action name is empty"
        if info.get("var_name", "") == "":
            return False, "Variable's name is empty"
        if info.get("type", None) == None:
            return False, "Action type is empty"
        if info.get("handle_goal", "") == "":
            return False, "Handle goal method name is empty"
        if info.get("handle_cancel", "") == "":
            return False, "Handle cancel method name is empty"
        if info.get("handle_accepted", "") == "":
            return False, "Handle accepted method name is empty"
        if info.get("execute", "") == "":
            return False, "Execute method name is empty"
        # if not skip_name and self.is_name_busy(info["var_name"]):
        #     return False, "Variable's name is busy"
        return True, ""

class ActionClientManager(NodeItemManagerBase):
    def __init__(self, action_client_dict: Dict):
        super().__init__(action_client_dict)
    
    def add(self, info: Dict):
        if not has_keys(info, ["srv_name", "var_name", "type", "goal_response_callback", "feedback_callback", "result_callback"]):
            return
        # TODO: check
        # if self.is_name_busy(info["var_name"]):
        #     return
        action_client_type_snake, action_client_pkg = convert_ros_format_generic(info["type"])
        action_client_include = f"{action_client_type_snake}.hpp"
        info["depends"] = [action_client_pkg]
        info["includes"] = [action_client_include]
        super().add(info)
    
    def validate(self, info: Dict, skip_name: bool = False) -> Tuple[bool, str]:
        if info.get("srv_name", "") == "":
            return False, "Action server name is empty"
        if info.get("var_name", "") == "":
            return False, "Variable's name is empty"
        if info.get("type", None) == None:
            return False, "Action type is empty"
        if info.get("goal_response_callback", "") == "":
            return False, "Goal response callback name is empty"
        if info.get("feedback_callback", "") == "":
            return False, "Feedback callback name is empty"
        if info.get("result_callback", "") == "":
            return False, "Result callback name is empty"
        # if not skip_name and self.is_name_busy(info["var_name"]):
        #     return False, "Variable's name is busy"
        return True, ""

class SyncSubManager(NodeItemManagerBase):
    def __init__(self, sync_sub_dict: Dict):
        super().__init__(sync_sub_dict, unique_key='callback')
        self.num_max_subs = 9
        
    def add(self, info: Dict):
        if not has_keys(info, ["callback", "sync_policy", "queue_size", "subs"]):
            return
        # TODO: check
        # if self.is_name_busy(info["callback"]):
        #     return
        for s in info["subs"]:
            msg_type_snake, msg_pkg = convert_ros_format_generic(s["msg_type"])
            msg_include = f"{msg_type_snake}.hpp"
            s["depends"] = [msg_pkg]
            s["includes"] = [msg_include]
        super().add(info)

    def validate(self, info: Dict, skip_name: bool = False) -> Tuple[bool, str]:
        if info.get("sync_policy", None) == None:
            return False, "Synchronization policy is empty"
        if info.get("callback", "") == "":
            return False, "Callback name is empty"
        if info.get("subs", []) == []:
            return False, "No subscriptions for synchronization"
        if len(info.get("subs", [])) > self.num_max_subs:
            return False, "Number of synchronized topics in C++ API cannot exceed 9"
        return True, ""
    
    def validate_sub(self, sub_info: Dict, skip_name: bool = False) -> Tuple[bool, str]:
        if sub_info.get("msg_type", None) == None:
            return False, "Message type is empty"
        if sub_info.get("var_name", "") == "":
            return False, "Variable name type is empty"
        if sub_info.get("topic", "") == "":
            return False, "Topic name is empty"
        return True, ""

class Ros2PkgGenerator:
    
    def __init__(self, config = {}):

        default_config = {
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
            "cmake_target_name": "my_library",
            "ros_distro": "Foxy",
            "language": "cpp"
        }
        # Merge so partially-specified configs (e.g. test fixtures that only set
        # the fields they care about) don't KeyError on the untouched ones.
        self.config = {**default_config, **config}


        self.subs = SubManager(self.config["subscribers"])
        self.pubs = PubManager(self.config["publishers"])
        self.timers = TimerManager(self.config["timers"])
        self.params = ParamManager(self.config["params"])
        self.srvs = ServiceManager(self.config["services"])
        self.clients = ClientManager(self.config["clients"])
        self.action_srvs = ActionServerManager(self.config["action_servers"])
        self.action_clients = ActionClientManager(self.config["action_clients"])
        self.sync_subs = SyncSubManager(self.config["sync_subscribers"])
        
        self.env = jinja2.Environment(
            loader=jinja2.FileSystemLoader('templates'),
            trim_blocks=True,
            lstrip_blocks=True
        )
        
        supported_ros_distros = ["foxy", "galactic", "humble", "iron", "jazzy"]
        self.templates =  {}
        for distro in supported_ros_distros:
            self.templates[distro] = {
                'hpp': self.env.get_template(f'{distro}/node.hpp.jinja2'),
                'cpp': self.env.get_template(f'{distro}/node.cpp.jinja2'),
                'xml': self.env.get_template(f'{distro}/package.xml.jinja2'),
                'cmake': self.env.get_template(f'{distro}/CMakeLists.txt.jinja2'),
            }

        # Python (rclpy) templates are distro-agnostic, unlike the C++ ones above
        self.py_templates = {
            'py': self.env.get_template('common/node.py.jinja2'),
            'setup': self.env.get_template('common/setup.py.jinja2'),
            'setup_cfg': self.env.get_template('common/setup.cfg.jinja2'),
            'xml': self.env.get_template('common/package.xml.jinja2'),
        }

    def get_callback_types(self, ros_distro: str) -> Dict:
        if ros_distro.lower() < 'humble':
            return {"Object": "", "UniquePtr": "::UniquePtr", "SharedPtr": "::SharedPtr", "ConstSharedPtr": "::ConstSharedPtr"}
        else:
            return {"Object": "", "Reference": "&", "UniquePtr": "::UniquePtr", "SharedPtr": "::SharedPtr", "ConstSharedPtr": "::ConstSharedPtr"}
    
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
        
        has_type_adapters = False
        for s in self.config.get("subscribers", []):
            deps.update(s["depends"])
            includes.update(s["includes"])
            has_type_adapters = has_type_adapters or "adapter_name" in s
        
        for p in self.config.get("publishers", []):
            deps.update(p["depends"])
            includes.update(p["includes"])
            has_type_adapters = has_type_adapters or "adapter_name" in p
        
        if has_type_adapters:
            includes.add("rclcpp/type_adapter.hpp")
        
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
            includes.add(f"message_filters/sync_policies/{_camel_to_snake(sync_sub['sync_policy'])}.h")
        
        if len(sync_subscribers) > 0:
            deps.add("message_filters")
            includes.add("message_filters/subscriber.h")
            includes.add("message_filters/synchronizer.h")

        self.config['include_pkgs'] = deps
        self.config['includes'] = includes

    def __update_py_imports(self):
        imports = set()
        for s in self.config.get("subscribers", []):
            imports.add((s["py_module"], s["py_class"]))
        for p in self.config.get("publishers", []):
            imports.add((p["py_module"], p["py_class"]))
        for s in self.config.get("services", []):
            imports.add((s["py_module"], s["py_class"]))
        for c in self.config.get("clients", []):
            imports.add((c["py_module"], c["py_class"]))
        self.config['py_imports'] = sorted(imports)

    def __update_py_param_info(self):
        for par in self.config.get("params", []):
            par["py_accessor"] = PARAM_PY_ACCESSORS.get(par["type"], "string_value")
            par["py_default"] = cpp_param_default_to_py(par["type"], par["default"])

    def generate_files(self):
        self.__update_includes()
        self.config["advertisement"] = "The package was created using ros2-package-generator: https://ros2-package-generator.onrender.com/"
        distro = self.config.get("ros_distro", "Foxy").lower()
        language = self.config.get("language", "cpp").lower()

        if language == "python":
            self.__update_py_imports()
            self.__update_py_param_info()
            return {
                f'{self.config["node_filename"]}.py': self.py_templates['py'].render(**self.config),
                '__init__.py': '',
                'setup.py': self.py_templates['setup'].render(**self.config),
                'setup.cfg': self.py_templates['setup_cfg'].render(**self.config),
                'resource': '',
                'package.xml': self.py_templates['xml'].render(**self.config),
            }

        return {
            f'{self.config["node_filename"]}.hpp': self.templates[distro]['hpp'].render(**self.config),
            f'{self.config["node_filename"]}.cpp': self.templates[distro]['cpp'].render(**self.config),
            "CMakeLists.txt": self.templates[distro]['cmake'].render(**self.config),
            'package.xml': self.templates[distro]['xml'].render(**self.config),
        }

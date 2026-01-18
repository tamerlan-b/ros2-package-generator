#! /usr/bin/python3

import streamlit as st
import jinja2
from streamlit_tags import st_tags
from graphviz import Digraph
import re

st.title("ROS2 Foxy Package Generator")

if "msg_autocomplete" not in st.session_state:
    st.session_state["msg_autocomplete"] = []
    with open("ros_messages.txt", 'r') as f:
        st.session_state["msg_autocomplete"] = f.readlines()

if "node_info" not in st.session_state:
    # TODO: upgrade dict structure
    st.session_state["node_info"] = {
        "node_filename": "node",
        'node_classname': 'Node',
        'node_name': 'node',
        'is_component': True,
        'include_pkgs': set(),
        'includes': set(),
        'params': [],
        'publishers': [],
        'subscribers': [],
        "package_name": "my_package",
        "cmake_target_name": "my_library"
    }

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

def convert_msg_format(msg_type: str) -> tuple[str, str]:
    """
    Convert message from C++ type to path-like format.
    
    Args:
        msg_type: Input message type in C++ format ("sensor_msgs::msg::Image")
    
    Examples:
        >>> convert_msg_format("sensor_msgs::msg::PointCloud2")
        "sensor_msgs/msg/point_cloud2", "sensor_msgs"
    """
    # Normalize input data
    normalized = msg_type.replace('/', '::')
    
    if '::msg::' not in normalized and '::' in normalized:
        # Add 'msg' if missing
        parts = normalized.split('::')
        if len(parts) == 2:
            normalized = f"{parts[0]}::msg::{parts[1]}"
    
    # Split
    if '::msg::' in normalized:
        package, _, message = normalized.split('::')
    elif '::' in normalized:
        package, message = normalized.split('::')
        # Suppose it is msg
        _ = "msg"
    else:
        # If can't parse
        package, message = "unknown", normalized
    
    # Convert message to snake_case
    message_snake = _camel_to_snake(message)
    
    return f"{package}/msg/{message_snake}", package

def generate_files():
    env = jinja2.Environment(
            loader=jinja2.FileSystemLoader('templates'),
            trim_blocks=True,
            lstrip_blocks=True
        )
    hpp_template = env.get_template('node.hpp.j2')
    cpp_template = env.get_template('node.cpp.j2')
    xml_template = env.get_template('package.xml.j2')
    cmake_template = env.get_template('CMakeLists.txt.j2')
    config = st.session_state["node_info"]
    return {
        f'{st.session_state["node_info"]["node_filename"]}.hpp': hpp_template.render(**config), 
        f'{st.session_state["node_info"]["node_filename"]}.cpp': cpp_template.render(**config), 
        "CMakeLists.txt": cmake_template.render(**config), 
        'package.xml': xml_template.render(**config), 
            }

def add_msg_include(msg_type: str):
    msg_type_snake, msg_pkg = convert_msg_format(msg_type)
    msg_include = f"{msg_type_snake}.hpp"
    st.session_state["node_info"]["includes"].add(msg_include)
    st.session_state["node_info"]["include_pkgs"].add(msg_pkg)

def refresh_includes():
    st.session_state["node_info"]['include_pkgs'] = set()
    st.session_state["node_info"]['includes'] = set()       
    for p in st.session_state["node_info"]["publishers"]:
        add_msg_include(p["msg_type"])
    for s in st.session_state["node_info"]["subscribers"]:
        add_msg_include(s["msg_type"])

def add_qos(prior_qos = {}):
    default_qos = {"history": "Keep last", "queue_size": None, "reliability": "Reliable", "durability": "Volatile"}
    qos = {}
    history_opts = ["Keep last", "Keep all"]
    qos["history"] = st.radio("History", history_opts, index= 0 if prior_qos == {} or "history" not in prior_qos.keys() else [i for i, h in enumerate(history_opts) if h == prior_qos["history"]][0], horizontal=True)
    qos["is_keep_all"] = qos["history"] == "Keep all"
    qos["queue_size"] = 1
    if not qos["is_keep_all"]:
        qos["queue_size"] = st.number_input("Queue size", min_value=1, value= 1 if prior_qos == {} or "queue_size" not in prior_qos.keys() else prior_qos["queue_size"], step=1)
    
    reliability_opts = ["Reliable", "Best effort"]
    qos["reliability"] = st.radio("Reliability", reliability_opts, index= 0 if prior_qos == {} or "reliability" not in prior_qos.keys() else [i for i, r in enumerate(reliability_opts) if r == prior_qos["reliability"]][0], horizontal=True)
    qos["is_best_effort"] = qos["reliability"] == "Best effort"
    
    durability_opts = ["Volatile", "Transient local"]
    qos["durability"] = st.radio("Durability", durability_opts, index= 0 if prior_qos == {} or "durability" not in prior_qos.keys() else [i for i, d in enumerate(durability_opts) if d == prior_qos["durability"]][0], horizontal=True)
    qos["is_transient_local"] = qos["durability"] == "Transient local"
    
    def is_default_qos(custom_qos: dict, qos_default: dict):
        for key in ["history", "reliability", "durability"]:
            if custom_qos[key] != qos_default[key]:
                return False
        return True
    
    qos["is_default"] = is_default_qos(qos, default_qos)
    return qos

def get_pub_sub_info(prior_info={}):
    info = {}
    tags = st_tags(
        label='Message type',
        text='Press Enter to add',
        value=[] if prior_info == {} or 'msg_type' not in prior_info.keys() else [prior_info["msg_type"]],
        suggestions=st.session_state["msg_autocomplete"],
        maxtags=1,
        key="msgs_tags"
    )
    info["msg_type"] = None if len(tags) == 0 else tags[0]
    # TODO: check if variable's name is free
    info["var_name"] = st.text_input("Variable name", placeholder="cloud_sub", value= "" if prior_info == {} or 'var_name' not in prior_info.keys() else prior_info["var_name"])
    name_free = True
    for p in st.session_state["node_info"]["publishers"]:
        if p["var_name"] == info["var_name"]:
            name_free = False
            break
    if name_free:
        for s in st.session_state["node_info"]["subscribers"]:
            if s["var_name"] == info["var_name"]:
                name_free = False
                break
    if not name_free and 'var_name' not in prior_info.keys():
        st.error(f'Name {info["var_name"]} is already used for another variable')
            
    info["topic"] = st.text_input("Topic name", placeholder="/points", value="" if prior_info == {} or 'topic' not in prior_info.keys() else prior_info["topic"])
    return info

@st.dialog("Add Subscriber")
def add_subscriber():
    sub_info = get_pub_sub_info()
    sub_info["callback"] = st.text_input("Callback function name", placeholder="cloud_callback")
    callback_arg_type = st.radio("Callback argument type", ["Object", "UniquePtr", "SharedPtr", "ConstSharedPtr"], index=2, horizontal=True)
    cb_types = {"Object": "obj", "UniquePtr": "uptr", "SharedPtr": "sptr", "ConstSharedPtr": "csptr"}
    sub_info["callback_arg_type"] = cb_types[callback_arg_type]
    type_suffix = "" if callback_arg_type == "Object" else f"::{callback_arg_type}"
    st.code(f'void {sub_info["callback"]}(const {sub_info["msg_type"]}{type_suffix} msg);', language="cpp")
    with st.expander('QoS settings'):
        sub_info["qos"] = add_qos()
    
    if st.button("Submit"):
        st.session_state["node_info"]["subscribers"].append(sub_info)
        # Add to include lists
        add_msg_include(sub_info["msg_type"])
        st.rerun()

@st.dialog("Add Publisher")
def add_publisher():
    pub_info = get_pub_sub_info()
    with st.expander('QoS settings'):
        pub_info["qos"] = add_qos()
    if st.button("Submit"):
        st.session_state["node_info"]["publishers"].append(pub_info)
        # Add to include lists
        add_msg_include(pub_info["msg_type"])
        st.rerun()

@st.dialog("Add Parameter")
def add_parameter():
    param_info = {}
    param_info["name"] = st.text_input("Name")
    name_busy = any(p["name"] == param_info["name"] for p in st.session_state["node_info"]["params"])
    if name_busy:
        st.error(f'Name {param_info["name"]} is already used for another parameter')
        
    param_types = ["bool", "int", "double", "std::string", 
                    "std::vector<uint8_t>", "std::vector<bool>", 
                    "std::vector<int>", "std::vector<double>", "std::vector<std::string>"]
    param_info["type"] = st.selectbox("Type", param_types)
    
    default_value = st.text_input("Default value", placeholder="Optional field")
    if default_value != "":
        param_info["default"] = default_value
    
    if st.button("Submit"):
        st.session_state["node_info"]["params"].append(param_info)
        st.rerun()

@st.dialog("Edit Publisher")
def edit_publisher(pub_var_name):
    index = [i for i, p in enumerate(st.session_state["node_info"]['publishers']) if p["var_name"] == pub_var_name][0]
    editing_pub = st.session_state["node_info"]['publishers'][index]
    pub_info = get_pub_sub_info(editing_pub)
    with st.expander('QoS settings'):
        pub_info["qos"] = add_qos(editing_pub["qos"])
    if st.button("Apply"):
        st.session_state["node_info"]['publishers'][index] = pub_info
        refresh_includes()
        add_msg_include(pub_info["msg_type"])
        st.rerun()

@st.dialog("Edit Subscriber")
def edit_subscriber(sub_var_name):
    index = [i for i, p in enumerate(st.session_state["node_info"]['subscribers']) if p["var_name"] == sub_var_name][0]
    editing_sub = st.session_state["node_info"]['subscribers'][index]
    sub_info = get_pub_sub_info(editing_sub)
    
    sub_info["callback"] = st.text_input("Callback function name", placeholder="cloud_callback", value= "" if editing_sub == {} or 'callback' not in editing_sub.keys() else editing_sub["callback"])
    cb_types = {"Object": "obj", "UniquePtr": "uptr", "SharedPtr": "sptr", "ConstSharedPtr": "csptr"}
    callback_arg_type = st.radio("Callback argument type", cb_types.keys(), index=2 if editing_sub == {} or 'callback_arg_type' not in editing_sub.keys() else [i for i, k in enumerate(cb_types.keys()) if cb_types[k] == editing_sub["callback_arg_type"]][0], horizontal=True)
    sub_info["callback_arg_type"] = cb_types[callback_arg_type]
    type_suffix = "" if callback_arg_type == "Object" else f"::{callback_arg_type}"
    st.code(f'void {sub_info["callback"]}(const {sub_info["msg_type"]}{type_suffix} msg);', language="cpp")
    with st.expander('QoS settings'):
        sub_info["qos"] = add_qos(editing_sub["qos"])
    
    if st.button("Apply"):
        st.session_state["node_info"]['subscribers'][index] = sub_info
        refresh_includes()
        add_msg_include(sub_info["msg_type"])
        st.rerun()

@st.dialog("Edit Parameter")
def edit_parameter(param_name):
    index = [i for i, p in enumerate(st.session_state["node_info"]['params']) if p["name"] == param_name][0]
    editing_param = st.session_state["node_info"]['params'][index]
    param_info = {}
    param_info["name"] = st.text_input("Name", value="" if editing_param == {} or 'name' not in editing_param.keys() else editing_param["name"])
    name_busy = any(p["name"] == param_info["name"] for p in st.session_state["node_info"]["params"])
    if name_busy:
        if editing_param and editing_param['name'] != param_info["name"]:
            st.error(f'Name {param_info["name"]} is already used for another parameter')
        
    param_types = ["bool", "int", "double", "std::string", 
                    "std::vector<uint8_t>", "std::vector<bool>", 
                    "std::vector<int>", "std::vector<double>", "std::vector<std::string>"]
    param_info["type"] = st.selectbox("Type", param_types, index=0 if editing_param == {} or 'type' not in editing_param.keys() else [i for i, pt in enumerate(param_types) if pt == editing_param["type"]][0])
    
    default_value = st.text_input("Default value", placeholder="Optional field", value="" if editing_param == {} or 'default' not in editing_param.keys() else editing_param["default"])
    if default_value != "":
        param_info["default"] = default_value
    
    if st.button("Submit"):
        st.session_state["node_info"]["params"][index] = param_info
        st.rerun()

with st.sidebar:
    # TODO: Remove button later
    if st.button("Fill by default"):
        st.session_state["node_info"] = {
            "node_filename": "node",
            'node_classname': 'Node',
            'node_name': 'node',
            'is_component': True,
            'include_pkgs': {
                'sensor_msgs'
            },
            'includes': {
                'sensor_msgs/msg/image.hpp',
                'sensor_msgs/msg/point_cloud2.hpp'
            },
            'params': [
                {"name": "buffer_size", "type": "int"},
                {"name": "path_to_onnx", "type": "std::string"},
                ],
            'publishers': [
                {"msg_type": "sensor_msgs::msg::Image", "var_name": "img_pub", "topic": "/image", "qos": {"is_default": True, "queue_size": 4}},
                ],
            'subscribers': [
                {"msg_type": "sensor_msgs::msg::PointCloud2", "var_name": "cloud_sub", "callback": "cloud_callback", "topic": "/points", "qos": {"is_default": True, "queue_size": 4}},
                {"msg_type": "geometry_msgs::msg::PoseStamped", "var_name": "pose_sub", "callback": "pose_callback", "topic": "/initial_pose", "qos": {"is_default": True, "queue_size": 4}},
            ],
            "package_name": "my_package",
            "cmake_target_name": "my_library"
        }

    st.session_state["node_info"]["package_name"] = st.text_input("Package name", "my_package")
    st.session_state["node_info"]["node_filename"] = st.text_input("Node filename", "my_node")
    st.session_state["node_info"]["cmake_target_name"] = st.text_input("CMake target name", "my_node_component")
    st.session_state["node_info"]["node_classname"] = st.text_input("Node C++ classname", "Node")
    st.session_state["node_info"]["node_name"] = st.text_input("Node name", "my_node")
    st.session_state["node_info"]["node_ns"] = st.text_input("Node namespace", "my_ns")
    node_type = st.radio("Node type", ["node", "component"], index=1, horizontal=True)
    st.session_state["node_info"]["is_component"] = node_type == "component"
    st.session_state["node_info"]["tf_listener"] = st.checkbox("Add tf listener", False)

def draw_node():
    dot = Digraph(comment='ROS2 Node', format='svg')
    dot.attr(rankdir='LR')  # Left to right

    # Set styles
    dot.attr('node', shape='box', style='filled', color='lightblue2')
    dot.attr('edge', arrowhead='normal')
    
    # Add node (in the middle)
    dot.node('NODE', st.session_state["node_info"]['node_name'], shape='box', style='filled', 
            fillcolor='lightblue', fontsize='16', fontname='Arial')

    # Add input topics (subscribers) - to the left
    for i, sub in enumerate(st.session_state["node_info"]['subscribers']):
        dot.node(f'IN_{i}', sub["topic"], shape='parallelogram', 
                style='filled', fillcolor='lightcoral')
        dot.edge(f'IN_{i}', 'NODE', label='')

    # Add output topics (publishers) - to the right
    for i, pub in enumerate(st.session_state["node_info"]['publishers']):
        dot.node(f'OUT_{i}', pub["topic"], shape='parallelogram', 
                style='filled', fillcolor='lightgreen')
        dot.edge('NODE', f'OUT_{i}', label='')

    return dot

with st.expander("Node structure", expanded=True):
    def text_with_button(text, button_icon="➕", help=None, on_click=None, btn_key=None):
        text_col, btn_col = st.columns([2, 1], vertical_alignment='center')
        text_col.write(text)
        return btn_col.button(button_icon, help=help, on_click=on_click, key=btn_key)
    
    def checkboxes_with_button(text, button_icon="➕", help=None, on_click=None, btn_key=None):
        cb_col, btn_col = st.columns([2, 1], vertical_alignment='center')
        btn_col.button(button_icon, help=help, on_click=on_click, key=btn_key)
        return cb_col.checkbox(text)
     
    checkboxes = {'sub': [], 'pub': [], 'params': []}
        
    text_with_button("Subscribers:", "➕", help="Add subscriber", on_click=lambda: add_subscriber())
    for sub in st.session_state["node_info"]["subscribers"]:
        var_name = sub["var_name"]
        checkboxes['sub'].append(checkboxes_with_button(f'`{sub["var_name"]}` (`{sub["msg_type"]}`)', "✏️", help="Edit", btn_key=sub["var_name"], on_click=lambda var=var_name: edit_subscriber(var)))
    
    text_with_button("Publishers:", "➕", help="Add publisher", on_click=lambda: add_publisher())
    for pub in st.session_state["node_info"]["publishers"]:
        var_name = pub["var_name"]
        checkboxes['pub'].append(checkboxes_with_button(f'`{pub["var_name"]}` (`{pub["msg_type"]}`)', "✏️", help="Edit", btn_key=pub["var_name"], on_click=lambda var=var_name: edit_publisher(var)))
    
    text_with_button("Parameters:", "➕", help="Add parameter", on_click=lambda: add_parameter())
    for par in st.session_state["node_info"]["params"]:
        var_name = par["name"]
        checkboxes['params'].append(checkboxes_with_button(f'`{par["name"]}` (`{par["type"]}`)', "✏️", help="Edit", btn_key=par["name"], on_click=lambda var=var_name: edit_parameter(var)))
    
    # Remove selected items
    if st.button("Remove selected items 🗑️", type="primary"):
        st.session_state["node_info"]["publishers"] = [p for i, p in enumerate(st.session_state["node_info"]["publishers"], 0) if checkboxes['pub'][i] == False]
        st.session_state["node_info"]["subscribers"] = [s for i, s in enumerate(st.session_state["node_info"]["subscribers"], 0) if checkboxes['sub'][i] == False]
        st.session_state["node_info"]["params"] = [p for i, p in enumerate(st.session_state["node_info"]["params"], 0) if checkboxes['params'][i] == False]
        # TODO: Remove includes more safely
        refresh_includes()
        st.rerun()
    
    # Visualize node's graph
    with st.expander("Graph", expanded=True):
        st.graphviz_chart(draw_node())

# Write terminal command for package generation
with st.expander("ROS2 pkg create command:", expanded=True):
    st.code(f'ros2  pkg create --build-type ament_cmake {st.session_state["node_info"]["package_name"]}', language="bash")

# Generate package's files
files = generate_files()
tabs = st.tabs(files.keys())
index = 0
for fname, fcontent in files.items():
    with tabs[index]:
        if ".hpp" in fname or ".cpp" in fname:
            st.code(fcontent, language="cpp")
        elif ".xml" in fname:
            st.code(fcontent, language="xml")
        elif fname == "CMakeLists.txt":
            st.code(fcontent, language="cmake")
        else:
            st.code(fcontent)
    index += 1

# For debug
# st.write(st.session_state["node_info"])

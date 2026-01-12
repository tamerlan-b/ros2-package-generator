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

def add_qos():
    default_qos = {"history": "Keep last", "queue_size": None, "reliability": "Reliable", "durability": "Volatile"}
    qos = {}
    st.text("QoS will be here")
    qos["history"] = st.radio("History", ["Keep last", "Keep all"], horizontal=True)
    qos["is_keep_all"] = qos["history"] == "Keep all"
    qos["queue_size"] = 1
    if not qos["is_keep_all"]:
        qos["queue_size"] = st.number_input("Queue size", min_value=1, step=1)
    qos["reliability"] = st.radio("Reliability", ["Reliable", "Best effort"], horizontal=True)
    qos["is_best_effort"] = qos["reliability"] == "Best effort"
    
    qos["durability"] = st.radio("Durability", ["Volatile", "Transient local"], horizontal=True)
    qos["is_transient_local"] = qos["durability"] == "Transient local"
    
    def is_default_qos(custom_qos: dict, qos_default: dict):
        for key in ["history", "reliability", "durability"]:
            if custom_qos[key] != qos_default[key]:
                return False
        return True
    
    qos["is_default"] = is_default_qos(qos, default_qos)
    return qos

def get_pub_sub_info():
    info = {}
    tags = st_tags(
        label='Message type',
        text='Press Enter to add',
        value=[],
        suggestions=st.session_state["msg_autocomplete"],
        maxtags=1,
        key="msgs_tags"
    )
    info["msg_type"] = None if len(tags) == 0 else tags[0]
    # TODO: check if variable's name is free
    info["var_name"] = st.text_input("Variable name", placeholder="cloud_sub")
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
    if not name_free:
        st.error(f'Name {info["var_name"]} is already used for another variable')
            
    info["topic"] = st.text_input("Topic name", placeholder="/points")
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

@st.dialog("Remove node elements")
def remove_node_elements():
    # Generate checkboxes
    checkboxes = {'sub': [], 'pub': []}
    if len(st.session_state["node_info"]["publishers"]) > 0:
        st.text("Publishers:")
    for pub in st.session_state["node_info"]["publishers"]:
        checkboxes['pub'].append(st.checkbox(f'`{pub["var_name"]}` (`{pub["msg_type"]}`)'))
    
    if len(st.session_state["node_info"]["subscribers"]) > 0:
        st.text("Subscribers:")
    for sub in st.session_state["node_info"]["subscribers"]:
        checkboxes['sub'].append(st.checkbox(f'`{sub["var_name"]}` (`{sub["msg_type"]}`)'))
    
    # Remove selected items
    if st.button("Remove selected items"):
        st.session_state["node_info"]["publishers"] = [p for i, p in enumerate(st.session_state["node_info"]["publishers"], 0) if checkboxes['pub'][i] == False]
        st.session_state["node_info"]["subscribers"] = [s for i, s in enumerate(st.session_state["node_info"]["subscribers"], 0) if checkboxes['sub'][i] == False]
        # TODO: Remove includes more safely
        st.session_state["node_info"]['include_pkgs'] = set()
        st.session_state["node_info"]['includes'] = set()
        
        for p in st.session_state["node_info"]["publishers"]:
            add_msg_include(p["msg_type"])
        for s in st.session_state["node_info"]["subscribers"]:
            add_msg_include(s["msg_type"])
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
            'publishers': [
                {"msg_type": "sensor_msgs::msg::Image", "var_name": "img_pub", "topic": "/image", "qos": {"is_default": True, "queue_size": 4}},
                {"msg_type": "std_msgs::msg::String", "var_name": "text_pub", "topic": "/text", "qos": {"is_default": True, "queue_size": 10}},
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

    col1, col2, col3 = st.columns(3)
    with col1:
        if st.button("Add Subscriber"):
            add_subscriber()
    with col2:
        if st.button("Add Publisher"):
            add_publisher()
    with col3:
        if st.button("Remove node elements", type="primary"):
            remove_node_elements()

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

# Visualize node's structure
with st.expander("Node structure"):
    node_struct_str = 'Publishers:'
    for pub in st.session_state["node_info"]["publishers"]:
        node_struct_str += f'\n- `{pub["var_name"]}` (`{pub["msg_type"]}`)'
    
    node_struct_str += '\n\nSubscribers:'
    for sub in st.session_state["node_info"]["subscribers"]:
        node_struct_str += f'\n- `{sub["var_name"]}` (`{sub["msg_type"]}`)'
    st.markdown(node_struct_str)
    
    # Visualize node's graph
    st.graphviz_chart(draw_node())

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

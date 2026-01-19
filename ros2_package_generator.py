#! /usr/bin/python3

import streamlit as st
from streamlit_tags import st_tags
from graphviz import Digraph
import zipfile
import io
from typing import Dict
from generator_core import Ros2PkgGenerator

st.title("ROS2 Package Generator")

if "msg_autocomplete" not in st.session_state:
    st.session_state["param_types"] =  [
        "bool", "int", "double", "std::string", "std::vector<int>", "std::vector<double>", "std::vector<std::string>"]
    st.session_state["msg_autocomplete"] = []
    with open("ros_messages.txt", 'r') as f:
        st.session_state["msg_autocomplete"] = f.readlines()

if 'gen' not in st.session_state:
    st.session_state['gen'] = Ros2PkgGenerator()

def get_index(value, iterable, default_index=-1):
    for i, item in enumerate(iterable, 0):
        if item == value:
            return i
    return default_index

def add_qos(prior_qos: Dict = {}) -> Dict:
    default_qos = {"history": "Keep last", "queue_size": None, "reliability": "Reliable", "durability": "Volatile"}
    qos = {}
    history_opts = ["Keep last", "Keep all"]
    qos["history"] = st.radio("History", history_opts, index=0 if "history" not in prior_qos else get_index(prior_qos["history"], history_opts, 0), horizontal=True)
    qos["is_keep_all"] = qos["history"] == "Keep all"
    qos["queue_size"] = 1
    if not qos["is_keep_all"]:
        qos["queue_size"] = st.number_input("Queue size", min_value=1, value=prior_qos.get("queue_size", 1), step=1)
    
    reliability_opts = ["Reliable", "Best effort"]
    qos["reliability"] = st.radio("Reliability", reliability_opts, index=0 if "reliability" not in prior_qos else get_index(prior_qos["reliability"], reliability_opts, 0), horizontal=True)
    qos["is_best_effort"] = qos["reliability"] == "Best effort"
    
    durability_opts = ["Volatile", "Transient local"]
    qos["durability"] = st.radio("Durability", durability_opts, index=0 if "durability" not in prior_qos else get_index(prior_qos["durability"], durability_opts, 0), horizontal=True)
    qos["is_transient_local"] = qos["durability"] == "Transient local"
    
    def is_default_qos(custom_qos: dict, qos_default: dict):
        for key in ["history", "reliability", "durability"]:
            if custom_qos[key] != qos_default[key]:
                return False
        return True
    
    qos["is_default"] = is_default_qos(qos, default_qos)
    return qos

def get_pub_sub_info(prior_info: Dict={}) -> Dict:
    info = {}
    tags = st_tags(
        label='Message type',
        text='Press Enter to add',
        value=[] if 'msg_type' not in prior_info else [prior_info["msg_type"]],
        suggestions=st.session_state["msg_autocomplete"],
        maxtags=1,
        key="msgs_tags"
    )
    info["msg_type"] = None if len(tags) == 0 else tags[0]
    info["var_name"] = st.text_input("Variable name", placeholder="cloud_sub", value=prior_info.get("var_name", ""))            
    info["topic"] = st.text_input("Topic name", placeholder="/points", value=prior_info.get("topic", ""))
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
        st.session_state['gen'].add_subscription(sub_info)
        st.rerun()

@st.dialog("Add Publisher")
def add_publisher():
    pub_info = get_pub_sub_info()
    with st.expander('QoS settings'):
        pub_info["qos"] = add_qos()
    if st.button("Submit"):
        st.session_state['gen'].add_publisher(pub_info)
        st.rerun()

@st.dialog("Add Parameter")
def add_parameter():
    param_info = {}
    param_info["name"] = st.text_input("Name")        
    param_info["type"] = st.selectbox("Type", options=st.session_state["param_types"])
    
    param_info["default"] = st.text_input("Default value")
    if param_info["default"] == "":
        st.error("You should enter default value")
    
    if st.button("Submit"):
        st.session_state['gen'].add_param(param_info)
        st.rerun()

@st.dialog("Add Timer")
def add_timer():
    timer_info = {}
    timer_info["var_name"] = st.text_input("Variable name")
    timer_info["period"] = st.number_input("Period in milliseconds", min_value=1, step=1)
    timer_info["callback"] = st.text_input("Callback function name")
    if st.button("Submit"):
        st.session_state['gen'].add_timer(timer_info)
        st.rerun()

@st.dialog("Edit Publisher")
def edit_publisher(pub_var_name):
    editing_pub, index = st.session_state['gen'].get_publisher(pub_var_name)
    pub_info = get_pub_sub_info(editing_pub)
    with st.expander('QoS settings'):
        pub_info["qos"] = add_qos(editing_pub["qos"])
    if st.button("Apply"):
        st.session_state['gen'].update_publisher(pub_info, index)
        st.rerun()

@st.dialog("Edit Subscriber")
def edit_subscriber(sub_var_name):
    editing_sub, index = st.session_state['gen'].get_subscription(sub_var_name)
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
        st.session_state['gen'].update_subscription(sub_info, index)
        st.rerun()

@st.dialog("Edit Parameter")
def edit_parameter(param_name):
    editing_param, index = st.session_state['gen'].get_param(param_name)
    param_info = {}
    param_info["name"] = st.text_input("Name", value=editing_param.get("name", ""))
    param_info["type"] = st.selectbox("Type", st.session_state["param_types"], index=get_index(editing_param["type"], st.session_state["param_types"], 0))
    param_info["default"] = st.text_input("Default value", value=editing_param.get("default", ""))
    if param_info["default"] == "":
        st.error("You should enter default value")
    
    if st.button("Submit"):
        st.session_state['gen'].update_param(param_info, index)
        st.rerun()

@st.dialog("Edit Timer")
def edit_timer(timer_var_name):
    editing_timer, index = st.session_state['gen'].get_timer(timer_var_name)
    timer_info = {}
    timer_info["var_name"] = st.text_input("Variable name", value=editing_timer.get("var_name", ""))
    timer_info["period"] = st.number_input("Period in milliseconds", min_value=1, step=1, value=editing_timer.get("period", ""))
    timer_info["callback"] = st.text_input("Callback function name", value=editing_timer.get("callback", ""))
    if st.button("Submit"):
        st.session_state['gen'].update_timer(timer_var_name, index)
        st.rerun()

with st.sidebar:
    
    # TODO: Remove button later
    if st.button("Fill by default"):
        st.session_state["gen"]["node_filename"] = "node"
        st.session_state["gen"]['node_classname'] = 'MyNode'
        st.session_state["gen"]['node_name'] = 'node'
        st.session_state["gen"]['is_component'] = True
        st.session_state["gen"]['include_pkgs'] = {}
        st.session_state["gen"]['includes'] = {}
        st.session_state["gen"]['params'] = []
        st.session_state["gen"]['publishers'] = []
        st.session_state["gen"]['subscribers'] = []
        st.session_state["gen"]["timers"] = []
        st.session_state["gen"]["package_name"] = "my_package"
        st.session_state["gen"]["cmake_target_name"] = "my_library"
        st.session_state["gen"].add_publisher({"msg_type": "sensor_msgs::msg::Image", "var_name": "img_pub", "topic": "/image", "qos": {"is_default": True, "queue_size": 4}})
        st.session_state["gen"].add_subscription({"msg_type": "sensor_msgs::msg::PointCloud2", "var_name": "cloud_sub", "callback": "cloud_callback", "callback_arg_type": "sptr", "topic": "/points", "qos": {"is_default": True, "queue_size": 4}})
        st.session_state["gen"].add_subscription({"msg_type": "geometry_msgs::msg::PoseStamped", "var_name": "pose_sub", "callback": "pose_callback", "callback_arg_type": "sptr", "topic": "/initial_pose", "qos": {"is_default": True, "queue_size": 4}})
        st.session_state["gen"].add_timer({"var_name": "my_timer", "period": 50, "callback": "my_timer_callback"},)
        st.session_state["gen"].add_param({"name": "buffer_size", "type": "int", "default": "10"})
        st.session_state["gen"].add_param({"name": "path_to_onnx", "type": "std::string", "default": "\"\""})

    # TODO: Support newer ROS2 distros
    st.session_state["gen"]["ros_distro"]  = st.selectbox("ROS2 Distro", options=["Foxy"])
    st.session_state['gen']["package_name"] = st.text_input("Package name", "my_package")
    st.session_state['gen']["node_filename"] = st.text_input("Node filename", "my_node")
    st.session_state['gen']["cmake_target_name"] = st.text_input("CMake target name", "my_node_component")
    st.session_state['gen']["node_classname"] = st.text_input("Node C++ classname", "MyNode")
    st.session_state['gen']["node_name"] = st.text_input("Node name", "my_node")
    st.session_state['gen']["node_ns"] = st.text_input("Node namespace", "my_ns")
    node_type = st.radio("Node type", ["node", "component"], index=1, horizontal=True)
    st.session_state['gen']["is_component"] = node_type == "component"
    st.session_state['gen']["tf_listener"] = st.checkbox("Add tf listener", False)

def draw_node() -> Digraph:
    dot = Digraph(comment='ROS2 Node', format='svg')
    dot.attr(rankdir='LR')  # Left to right

    # Set styles
    dot.attr('node', shape='box', style='filled', color='lightblue2')
    dot.attr('edge', arrowhead='normal')
    
    # Add node (in the middle)
    dot.node('NODE', st.session_state['gen']['node_name'], shape='box', style='filled', 
            fillcolor='lightblue', fontsize='16', fontname='Arial')

    # Add input topics (subscribers) - to the left
    for i, sub in enumerate(st.session_state['gen']['subscribers']):
        dot.node(f'IN_{i}', sub["topic"], shape='parallelogram', 
                style='filled', fillcolor='lightcoral')
        dot.edge(f'IN_{i}', 'NODE', label='')

    # Add output topics (publishers) - to the right
    for i, pub in enumerate(st.session_state['gen']['publishers']):
        dot.node(f'OUT_{i}', pub["topic"], shape='parallelogram', 
                style='filled', fillcolor='lightgreen')
        dot.edge('NODE', f'OUT_{i}', label='')

    return dot

with st.expander("Node structure", expanded=True):
    
    def text_with_button(text: str, button_icon: str="➕", help: str=None, on_click=None, btn_key=None):
        text_col, btn_col = st.columns([2, 1], vertical_alignment='center')
        text_col.write(text)
        return btn_col.button(button_icon, help=help, on_click=on_click, key=btn_key)
    
    def checkboxes_with_button(text: str, button_icon: str="➕", help: str=None, on_click=None, btn_key=None):
        cb_col, btn_col = st.columns([2, 1], vertical_alignment='center')
        btn_col.button(button_icon, help=help, on_click=on_click, key=btn_key)
        return cb_col.checkbox(text)
     
    checkboxes = {'sub': {}, 'pub': {}, 'params': {}, 'timers': {}}
        
    text_with_button("📥 Subscribers:", "➕", help="Add subscriber", on_click=lambda: add_subscriber())
    for sub in st.session_state['gen']["subscribers"]:
        var_name = sub["var_name"]
        checkboxes['sub'][var_name] = checkboxes_with_button(f'`{sub["var_name"]}` (`{sub["msg_type"]}`)', "✏️", help="Edit", btn_key=sub["var_name"], on_click=lambda var=var_name: edit_subscriber(var))
    
    text_with_button("📤 Publishers:", "➕", help="Add publisher", on_click=lambda: add_publisher())
    for pub in st.session_state['gen']["publishers"]:
        var_name = pub["var_name"]
        checkboxes['pub'][var_name] = checkboxes_with_button(f'`{pub["var_name"]}` (`{pub["msg_type"]}`)', "✏️", help="Edit", btn_key=pub["var_name"], on_click=lambda var=var_name: edit_publisher(var))
    
    text_with_button("⏱️ Timers:", "➕", help="Add timer", on_click=lambda: add_timer())
    for tim in st.session_state['gen']["timers"]:
        var_name = tim["var_name"]
        checkboxes['timers'][var_name] = checkboxes_with_button(f'`{tim["var_name"]}`: `{tim["period"]}ms`', "✏️", help="Edit", btn_key=tim["var_name"], on_click=lambda var=var_name: edit_timer(var))
    
    text_with_button("🔧 Parameters:", "➕", help="Add parameter", on_click=lambda: add_parameter())
    for par in st.session_state['gen']["params"]:
        var_name = par["name"]
        checkboxes['params'][var_name] = checkboxes_with_button(f'`{par["name"]}` (`{par["type"]}`)', "✏️", help="Edit", btn_key=par["name"], on_click=lambda var=var_name: edit_parameter(var))
    
    # Remove selected items
    if st.button("Remove selected items 🗑️", type="primary"):
        st.session_state['gen'].remove_publishers([k for k, v in checkboxes['pub'].items() if v])
        st.session_state['gen'].remove_subscriptions([k for k, v in checkboxes['sub'].items() if v])
        st.session_state['gen'].remove_params([k for k, v in checkboxes['params'].items() if v])
        st.session_state['gen'].remove_timers([k for k, v in checkboxes['timers'].items() if v])
        st.rerun()
    
    # Visualize node's graph
    with st.expander("Graph", expanded=True):
        st.graphviz_chart(draw_node())

def create_package_archive_structure(pkg_name: str, node_name: str, 
                                    files_content: Dict[str, str]) -> io.BytesIO:
    zip_buffer = io.BytesIO()
    
    with zipfile.ZipFile(zip_buffer, 'w', zipfile.ZIP_DEFLATED) as zf:
        # 1. Header file
        hpp_path = f"{pkg_name}/include/{pkg_name}/{node_name}.hpp"
        zf.writestr(hpp_path, files_content.get('hpp', ''))
        
        # 2. Source file
        cpp_path = f"{pkg_name}/src/{node_name}.cpp"
        zf.writestr(cpp_path, files_content.get('cpp', ''))
        
        # 3. CMakeLists.txt
        cmake_path = f"{pkg_name}/CMakeLists.txt"
        zf.writestr(cmake_path, files_content.get('cmake', ''))
        
        # 4. package.xml
        package_xml_path = f"{pkg_name}/package.xml"
        zf.writestr(package_xml_path, files_content.get('package_xml', ''))
    
    zip_buffer.seek(0)
    return zip_buffer

# Generate package's files
files = st.session_state['gen'].generate_files()

def simple_download_button():
    zip_files = {
        'hpp': [v for f, v in files.items() if ".hpp" in f][0],
        'cpp': [v for f, v in files.items() if ".cpp" in f][0],
        'cmake': [v for f, v in files.items() if "CMakeLists.txt" == f][0],
        'package_xml': [v for f, v in files.items() if "package.xml" == f][0],
    }
    
    zip_buffer = create_package_archive_structure(
        st.session_state["gen"]["package_name"], 
        st.session_state["gen"]["node_filename"], 
        zip_files)
    
    st.download_button(
        "📦 Download Package",
        data=zip_buffer,
        file_name=f'{st.session_state["gen"]["package_name"]}.zip',
        mime="application/zip"
    )

# Add "Download Package" button
simple_download_button()

# Write terminal command for package generation
with st.expander("ROS2 pkg create command:", expanded=True):
    st.code(f'ros2  pkg create --build-type ament_cmake {st.session_state["gen"]["package_name"]}', language="bash")

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

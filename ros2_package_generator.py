#! /usr/bin/python3

import streamlit as st
from streamlit_tags import st_tags
from graphviz import Digraph
import zipfile
import io
from typing import Dict
from generator_core import Ros2PkgGenerator

st.title("ROS2 Package Generator")

st.set_page_config(page_title="ROS2 Package Generator", page_icon="resources/icon.png")

def parse_ros_interfaces_file(filename: str):
    mapping = {
        "Messages:": "msg",
        "Services:": "srv", 
        "Actions:": "action"
    }
    result = {}
    current_key = None
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line in mapping:
                current_key = mapping[line]
                result[current_key] = []
            elif current_key and line:
                result[current_key].append(line.replace('/', '::'))
    return result

if "autocompletes" not in st.session_state:
    autcompletes = {}
    autcompletes = parse_ros_interfaces_file("ros2_interfaces.txt")
    autcompletes["params"] =  [
        "bool", "int", "double", "std::string", "std::vector<int>", "std::vector<double>", "std::vector<std::string>"]
    st.session_state['autocompletes'] = autcompletes

if 'gen' not in st.session_state:
    st.session_state['gen'] = Ros2PkgGenerator()

if 'view' not in st.session_state:
    st.session_state['view'] = 'Desktop'

def text_with_button(text: str, button_icon: str="➕", help: str=None, on_click=None, btn_key=None):
    text_col, btn_col = st.columns([2, 1], vertical_alignment='center')
    text_col.write(text)
    return btn_col.button(button_icon, help=help, on_click=on_click, key=btn_key)

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
        suggestions=st.session_state['autocompletes']['msg'],
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
    param_info["type"] = st.selectbox("Type", options=st.session_state['autocompletes']['params'])
    
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

@st.dialog("Add Service server")
def add_service():
    srv_info = {}
    srv_info["name"] = st.text_input("Service name")
    tags = st_tags(
        label='Type',
        text='Press Enter to add',
        value=[],
        suggestions=st.session_state['autocompletes']['srv'],
        maxtags=1,
        key="srvs_tags"
    )
    srv_info["type"] = None if len(tags) == 0 else tags[0]
    srv_info["var_name"] = st.text_input("Variable name", placeholder="service")
    srv_info["callback"] = st.text_input("Service handler method name", placeholder="add_two_ints")
    st.code(f'void {srv_info["callback"]}(const std::shared_ptr<srv_info["type"]::Request> request, std::shared_ptr<srv_info["type"]::Response> response);', language="cpp")
    
    if st.button("Submit"):
        st.session_state['gen'].add_service(srv_info)
        st.rerun()

@st.dialog("Add Service client")
def add_client():
    client_info = {}
    client_info["srv_name"] = st.text_input("Service name")
    tags = st_tags(
        label='Type',
        text='Press Enter to add',
        value=[],
        suggestions=st.session_state['autocompletes']['srv'],
        maxtags=1,
        key="srvs_tags"
    )
    client_info["type"] = None if len(tags) == 0 else tags[0]
    client_info["var_name"] = st.text_input("Variable name", placeholder="client")
    if st.button("Submit"):
        st.session_state['gen'].add_client(client_info)
        st.rerun()

@st.dialog("Add Action server")
def add_action_server():
    action_srv_info = {}
    action_srv_info["name"] = st.text_input("Action name")
    tags = st_tags(
        label='Type',
        text='Press Enter to add',
        value=[],
        suggestions=st.session_state['autocompletes']['action'],
        maxtags=1,
        key="actions_tags"
    )
    action_srv_info["type"] = None if len(tags) == 0 else tags[0]
    action_srv_info["var_name"] = st.text_input("Variable name", placeholder="action_server")
    
    action_srv_info["handle_goal"] = st.text_input("Goal handle method name", placeholder="handle_goal")
    action_srv_info["handle_cancel"] = st.text_input("Goal cancel handle method name", placeholder="handle_cancel")
    action_srv_info["handle_accepted"] = st.text_input("Goal accept handle method name", placeholder="handle_accepted")
    action_srv_info["execute"] = st.text_input("Goal execute method name", placeholder="execute")
    
    if st.button("Submit"):
        st.session_state['gen'].add_action_server(action_srv_info)
        st.rerun()

@st.dialog("Add Action client")
def add_action_client():
    action_client_info = {}
    action_client_info["srv_name"] = st.text_input("Action name")
    tags = st_tags(
        label='Type',
        text='Press Enter to add',
        value=[],
        suggestions=st.session_state['autocompletes']['action'],
        maxtags=1,
        key="actions_tags"
    )
    action_client_info["type"] = None if len(tags) == 0 else tags[0]
    action_client_info["var_name"] = st.text_input("Variable name", placeholder="action_client")
    
    action_client_info["goal_response_callback"] = st.text_input("Goal response callback name", placeholder="goal_response_cb")
    action_client_info["feedback_callback"] = st.text_input("Feedback callback name", placeholder="feedback_response_cb")
    action_client_info["result_callback"] = st.text_input("Result callback name", placeholder="result_response_cb")
    
    if st.button("Submit"):
        st.session_state['gen'].add_action_client(action_client_info)
        st.rerun()

@st.dialog("Add synchronized subscriber")
def __add_sync_sub():
    num_max_subs = 9
    sync_sub_info = {}
    sync_sub_info["var_name"] = st.text_input("Variable name")
    sync_sub_info["callback"] = st.text_input("Synchronized callback function name")
    sync_sub_info["sync_policy"] = st.selectbox("Synchronization policy", ["ExactTime", "ApproximateTime"]) #, "ApproximateEpsilonTime"])
    sync_sub_info["queue_size"] = st.number_input("Queue size", min_value=1, step=1)
    # "epsilon": "", # rclcpp::Duration (only for "ApproximateEpsilonTime")
    
    with st.form("Add subscription", clear_on_submit=True):
        st.subheader("Subscription params")
        sub_info = get_pub_sub_info()
        with st.expander('QoS settings'):
            sub_info["qos"] = add_qos()
        if st.form_submit_button("Add topic"):
            # TODO: add check for empty fields
            if len(st.session_state['temp_subs']) < num_max_subs:
                st.session_state['temp_subs'].append(sub_info)
            else:
                st.error("Number of synchronized topics in C++ API cannot exceed 9")
    
    def remove_sub(var_name: str):
        index = next(iter([i for i, s in enumerate(st.session_state['temp_subs']) if s["var_name"] == var_name]), -1)
        if index >= 0:
            del st.session_state['temp_subs'][index]
    
    for sub in st.session_state['temp_subs']:
        var_name = sub["var_name"]
        text_with_button(f'`{var_name}` (`{sub["msg_type"]}`)', "🗑️", help="Remove subscriber", on_click=lambda var=var_name: remove_sub(var), btn_key=var_name)
    
    if st.button("Submit"):
        # TODO: add check for fields and num subs
        sync_sub_info["subs"] = st.session_state['temp_subs'].copy()
        st.session_state["gen"].add_sync_subscription(sync_sub_info)
        st.session_state['temp_subs'] = []
        st.rerun()

def add_sync_sub():
    st.session_state['temp_subs'] = []
    __add_sync_sub()

@st.dialog("Edit Publisher")
def edit_publisher(pub_var_name: str):
    editing_pub, index = st.session_state['gen'].get_publisher(pub_var_name)
    pub_info = get_pub_sub_info(editing_pub)
    with st.expander('QoS settings'):
        pub_info["qos"] = add_qos(editing_pub["qos"])
    if st.button("Apply"):
        st.session_state['gen'].update_publisher(pub_info, index)
        st.rerun()

@st.dialog("Edit Subscriber")
def edit_subscriber(sub_var_name: str):
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
def edit_parameter(param_name: str):
    editing_param, index = st.session_state['gen'].get_param(param_name)
    param_info = {}
    param_info["name"] = st.text_input("Name", value=editing_param.get("name", ""))
    param_info["type"] = st.selectbox("Type", st.session_state['autocompletes']['params'], index=get_index(editing_param["type"], st.session_state['autocompletes']['params'], 0))
    param_info["default"] = st.text_input("Default value", value=editing_param.get("default", ""))
    if param_info["default"] == "":
        st.error("You should enter default value")
    
    if st.button("Submit"):
        st.session_state['gen'].update_param(param_info, index)
        st.rerun()

@st.dialog("Edit Timer")
def edit_timer(timer_var_name: str):
    editing_timer, index = st.session_state['gen'].get_timer(timer_var_name)
    timer_info = {}
    timer_info["var_name"] = st.text_input("Variable name", value=editing_timer.get("var_name", ""))
    timer_info["period"] = st.number_input("Period in milliseconds", min_value=1, step=1, value=editing_timer.get("period", ""))
    timer_info["callback"] = st.text_input("Callback function name", value=editing_timer.get("callback", ""))
    if st.button("Submit"):
        st.session_state['gen'].update_timer(timer_info, index)
        st.rerun()

@st.dialog("Edit Service server")
def edit_service(srv_var_name: str):
    editing_srv, index = st.session_state['gen'].get_service(srv_var_name)
    srv_info = {}
    srv_info["name"] = st.text_input("Service name", value=editing_srv.get("name", ""))        
    tags = st_tags(
        label='Type',
        text='Press Enter to add',
        value=[] if 'type' not in editing_srv else [editing_srv["type"]],
        suggestions=st.session_state['autocompletes']['srv'],
        maxtags=1,
        key="srvs_tags"
    )
    srv_info["type"] = None if len(tags) == 0 else tags[0]
    srv_info["var_name"] = st.text_input("Variable name", value=editing_srv.get("var_name", ""))
    srv_info["callback"] = st.text_input("Service handler method name", value=editing_srv.get("callback", ""))
        
    st.code(f'void {srv_info["callback"]}(const std::shared_ptr<srv_info["type"]::Request> request, std::shared_ptr<srv_info["type"]::Response> response);', language="cpp")
    
    if st.button("Submit"):
        st.session_state['gen'].update_service(srv_info, index)
        st.rerun()

@st.dialog("Edit Service client")
def edit_client(client_var_name: str):
    editing_client, index = st.session_state['gen'].get_client(client_var_name)
    client_info = {}
    client_info["srv_name"] = st.text_input("Service name", value=editing_client.get("srv_name", ""))
    tags = st_tags(
        label='Type',
        text='Press Enter to add',
        value=[] if 'type' not in editing_client else [editing_client["type"]],
        suggestions=st.session_state['autocompletes']['srv'],
        maxtags=1,
        key="srvs_tags"
    )
    client_info["type"] = None if len(tags) == 0 else tags[0]
    client_info["var_name"] = st.text_input("Variable name", value=editing_client.get("var_name", ""))
    if st.button("Submit"):
        st.session_state['gen'].update_client(client_info, index)
        st.rerun()

@st.dialog("Edit Action server")
def edit_action_server(action_srv_var_name: str):
    editing_action_srv, index = st.session_state['gen'].get_action_server(action_srv_var_name)
    action_srv_info = {}
    action_srv_info["name"] = st.text_input("Action name", value=editing_action_srv.get("name", ""))
    tags = st_tags(
        label='Type',
        text='Press Enter to add',
        value=[] if 'type' not in editing_action_srv else [editing_action_srv["type"]],
        suggestions=st.session_state['autocompletes']['action'],
        maxtags=1,
        key="actions_tags"
    )
    action_srv_info["type"] = None if len(tags) == 0 else tags[0]
    action_srv_info["var_name"] = st.text_input("Variable name", placeholder="action_server", value=editing_action_srv.get("var_name", ""))
    
    action_srv_info["handle_goal"] = st.text_input("Goal handle method name", value=editing_action_srv.get("handle_goal", ""))
    action_srv_info["handle_cancel"] = st.text_input("Goal cancel handle method name", value=editing_action_srv.get("handle_cancel", ""))
    action_srv_info["handle_accepted"] = st.text_input("Goal accept handle method name", value=editing_action_srv.get("handle_accepted", ""))
    action_srv_info["execute"] = st.text_input("Goal execute method name", value=editing_action_srv.get("execute", ""))
    
    if st.button("Submit"):
        st.session_state['gen'].update_action_server(action_srv_info, index)
        st.rerun()

@st.dialog("Edit Action client")
def edit_action_client(action_client_var_name: str):
    editing_action_client, index = st.session_state['gen'].get_action_client(action_client_var_name)
    action_client_info = {}
    action_client_info["srv_name"] = st.text_input("Action name", value=editing_action_client.get("srv_name", ""))
    tags = st_tags(
        label='Type',
        text='Press Enter to add',
        value=[] if 'type' not in editing_action_client else [editing_action_client["type"]],
        suggestions=st.session_state['autocompletes']['action'],
        maxtags=1,
        key="actions_tags"
    )
    action_client_info["type"] = None if len(tags) == 0 else tags[0]
    action_client_info["var_name"] = st.text_input("Variable name", placeholder="action_server", value=editing_action_client.get("var_name", ""))
    
    action_client_info["goal_response_callback"] = st.text_input("Goal response callback name", value=editing_action_client.get("goal_response_callback", ""))
    action_client_info["feedback_callback"] = st.text_input("Feedback callback name", value=editing_action_client.get("feedback_callback", ""))
    action_client_info["result_callback"] = st.text_input("Result callback name", value=editing_action_client.get("result_callback", ""))
    
    if st.button("Submit"):
        st.session_state['gen'].update_action_client(action_client_info, index)
        st.rerun()

@st.dialog("Edit synchronized subscriber")
def __edit_sync_sub(sync_sub_var_name: str):
    editing_sync_sub, index = st.session_state['gen'].get_sync_subscription(sync_sub_var_name)
    num_max_subs = 9
    sync_sub_info = {}
    sync_sub_info["var_name"] = st.text_input("Variable name", value=editing_sync_sub.get("var_name", ""))
    sync_sub_info["callback"] = st.text_input("Synchronized callback function name", value=editing_sync_sub.get("callback", ""))
    sync_sub_info["sync_policy"] = st.selectbox("Synchronization policy", ["ExactTime", "ApproximateTime"])
    sync_sub_info["queue_size"] = st.number_input("Queue size", min_value=1, step=1, value=editing_sync_sub.get("queue_size", 1))
    # "epsilon": "", # rclcpp::Duration (only for "ApproximateEpsilonTime")
    
    with st.form("Add subscription", clear_on_submit=True):
        st.subheader("Subscription params")
        # TODO: add ability to edit subscribers
        sub_info = get_pub_sub_info()
        with st.expander('QoS settings'):
            sub_info["qos"] = add_qos()
        if st.form_submit_button("Add topic"):
            # TODO: add check for empty fields
            if len(st.session_state['temp_subs']) < num_max_subs:
                st.session_state['temp_subs'].append(sub_info)
            else:
                st.error("Number of synchronized topics in C++ API cannot exceed 9")
    
    def remove_sub(var_name: str):
        index = next(iter([i for i, s in enumerate(st.session_state['temp_subs']) if s["var_name"] == var_name]), -1)
        if index >= 0:
            del st.session_state['temp_subs'][index]
    
    for sub in st.session_state['temp_subs']:
        var_name = sub["var_name"]
        text_with_button(f'`{var_name}` (`{sub["msg_type"]}`)', "🗑️", help="Remove subscriber", on_click=lambda var=var_name: remove_sub(var), btn_key=var_name)
    
    if st.button("Submit"):
        # TODO: add check for fields and num subs
        sync_sub_info["subs"] = st.session_state['temp_subs'].copy()
        st.session_state["gen"].update_sync_subscription(sync_sub_info, index)
        st.session_state['temp_subs'] = []
        st.rerun()
    pass

def edit_sync_sub(sync_sub_var_name: str):
    # TODO: fill temp_subs
    editing_sync_sub, index = st.session_state['gen'].get_sync_subscription(sync_sub_var_name)
    st.session_state['temp_subs'] = editing_sync_sub["subs"]
    __edit_sync_sub(sync_sub_var_name)

with st.sidebar:
    
    st.session_state['view'] = st.radio("Page view", ["Desktop", "Mobile"], horizontal=True)
    
    # TODO: Remove button later
    if st.button("Fill package by default"):
        st.session_state["gen"]["node_filename"] = "node"
        st.session_state["gen"]['node_classname'] = 'MyNode'
        st.session_state["gen"]['node_name'] = 'node'
        st.session_state["gen"]['is_component'] = True
        st.session_state["gen"]['include_pkgs'] = {}
        st.session_state["gen"]['includes'] = {}
        st.session_state["gen"]["package_name"] = "my_package"
        st.session_state["gen"]["cmake_target_name"] = "my_library"
        st.session_state["gen"].add_publisher({"msg_type": "sensor_msgs::msg::Image", "var_name": "img_pub", "topic": "/image", "qos": {"is_default": True, "queue_size": 4}})
        st.session_state["gen"].add_subscription({"msg_type": "sensor_msgs::msg::PointCloud2", "var_name": "cloud_sub", "callback": "cloud_callback", "callback_arg_type": "sptr", "topic": "/points", "qos": {"is_default": True, "queue_size": 4}})
        st.session_state["gen"].add_timer({"var_name": "my_timer", "period": 50, "callback": "my_timer_callback"},)
        st.session_state["gen"].add_param({"name": "buffer_size", "type": "int", "default": "10"})
        st.session_state["gen"].add_service({"name": "test_empty", "type": "std_srvs::srv::Empty", "var_name": "service", "callback": "service_callback"})
        st.session_state["gen"].add_client({"srv_name": "test_empty", "type": "std_srvs::srv::Empty", "var_name": "client"})
        st.session_state["gen"].add_action_server({"name": "fibonacci", "var_name": "action_server_", "type": "tf2_msgs::action::LookupTransform", "handle_goal": "handle_goal", "handle_cancel": "handle_cancel", "handle_accepted": "handle_accepted", "execute": "execute"})
        st.session_state["gen"].add_action_client({"srv_name": "fibonacci", "var_name": "action_client_", "type": "tf2_msgs::action::LookupTransform", "goal_response_callback": "goal_response_cb", "feedback_callback": "feedback_response_cb", "result_callback": "result_response_cb"})

    # TODO: Support newer ROS2 distros
    st.session_state["gen"]["ros_distro"]  = st.selectbox("ROS2 Distro", options=["Foxy", "Galactic", "Humble", "Iron", "Jazzy"])
    st.session_state['gen']["package_name"] = st.text_input("Package name", "my_package")
    st.session_state['gen']["node_filename"] = st.text_input("Node filename", "my_node")
    st.session_state['gen']["cmake_target_name"] = st.text_input("CMake target name", "my_node_component")
    st.session_state['gen']["node_classname"] = st.text_input("Node C++ classname", "MyNode")
    st.session_state['gen']["node_name"] = st.text_input("Node name", "my_node")
    st.session_state['gen']["node_ns"] = st.text_input("Node namespace", "my_ns")
    node_type = st.radio("Node type", ["node", "component"], index=1, horizontal=True)
    st.session_state['gen']["is_component"] = node_type == "component"

def create_elems_code_cols(is_mobile: bool):
    if is_mobile:
        return st.container(), st.container()
    else:
        return st.columns(2)

st.set_page_config(layout="centered" if st.session_state['view'] == 'Mobile' else "wide")
col_elems, col_code = create_elems_code_cols(st.session_state['view'] == 'Mobile')

with col_elems:

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

        # TODO: add visualization for services
        return dot

    elems_tab, graph_tab = st.tabs(["Node elements", "Graph"])
    
    with elems_tab:
        with st.expander("Node structure", expanded=True):
            
            st.session_state['gen']["tf_listener"] = st.toggle("🧭 TF Listener")
            
            def checkboxes_with_button(text: str, button_icon: str="➕", help: str=None, on_click=None, btn_key=None):
                cb_col, btn_col = st.columns([2, 1], vertical_alignment='center')
                btn_col.button(button_icon, help=help, on_click=on_click, key=btn_key)
                return cb_col.checkbox(text)
            
            checkboxes = {'sub': {}, 'pub': {}, 'params': {}, 'timers': {}, 'srv': {}, 'client': {}, 'action_srv': {}, 'action_client': {}, 'sync_sub': {}}
                
            text_with_button("📥 Subscribers:", "➕", help="Add subscriber", on_click=lambda: add_subscriber())
            for sub in st.session_state['gen']["subscribers"]:
                var_name = sub["var_name"]
                checkboxes['sub'][var_name] = checkboxes_with_button(f'`{var_name}` (`{sub["msg_type"]}`)', "✏️", help="Edit", btn_key=var_name, on_click=lambda var=var_name: edit_subscriber(var))
            
            text_with_button("📤 Publishers:", "➕", help="Add publisher", on_click=lambda: add_publisher())
            for pub in st.session_state['gen']["publishers"]:
                var_name = pub["var_name"]
                checkboxes['pub'][var_name] = checkboxes_with_button(f'`{var_name}` (`{pub["msg_type"]}`)', "✏️", help="Edit", btn_key=var_name, on_click=lambda var=var_name: edit_publisher(var))
            
            text_with_button("⏱️ Timers:", "➕", help="Add timer", on_click=lambda: add_timer())
            for tim in st.session_state['gen']["timers"]:
                var_name = tim["var_name"]
                checkboxes['timers'][var_name] = checkboxes_with_button(f'`{var_name}`: `{tim["period"]}ms`', "✏️", help="Edit", btn_key=var_name, on_click=lambda var=var_name: edit_timer(var))
            
            text_with_button("🔧 Parameters:", "➕", help="Add parameter", on_click=lambda: add_parameter())
            for par in st.session_state['gen']["params"]:
                var_name = par["name"]
                checkboxes['params'][var_name] = checkboxes_with_button(f'`{par["name"]}` (`{par["type"]}`)', "✏️", help="Edit", btn_key=par["name"], on_click=lambda var=var_name: edit_parameter(var))
            
            text_with_button("👂 Service servers:", "➕", help="Add service server", on_click=lambda: add_service())
            for srv in st.session_state['gen']["services"]:
                var_name = srv["var_name"]
                checkboxes['srv'][var_name] = checkboxes_with_button(f'`{var_name}` (`{srv["type"]}`)', "✏️", help="Edit", btn_key=var_name, on_click=lambda var=var_name: edit_service(var))
            
            text_with_button("🗣️ Service clients:", "➕", help="Add service client", on_click=lambda: add_client())
            for client in st.session_state['gen']["clients"]:
                var_name = client["var_name"]
                checkboxes['client'][var_name] = checkboxes_with_button(f'`{var_name}` (`{client["type"]}`)', "✏️", help="Edit", btn_key=var_name, on_click=lambda var=var_name: edit_client(var))
            
            text_with_button("🎬 Action servers:", "➕", help="Add action server", on_click=lambda: add_action_server())
            for action_srv in st.session_state['gen']["action_servers"]:
                var_name = action_srv["var_name"]
                checkboxes['action_srv'][var_name] = checkboxes_with_button(f'`{var_name}` (`{action_srv["type"]}`)', "✏️", help="Edit", btn_key=var_name, on_click=lambda var=var_name: edit_action_server(var))
            
            text_with_button("🎯 Action clients:", "➕", help="Add action client", on_click=lambda: add_action_client())
            for action_client in st.session_state['gen']["action_clients"]:
                var_name = action_client["var_name"]
                checkboxes['action_client'][var_name] = checkboxes_with_button(f'`{var_name}` (`{action_client["type"]}`)', "✏️", help="Edit", btn_key=var_name, on_click=lambda var=var_name: edit_action_client(var))
            
            text_with_button("🔀 Synchronized subscribers (message filters):", "➕", help="Add sync subscriber", on_click=lambda: add_sync_sub())
            for sync_sub in st.session_state['gen']["sync_subscribers"]:
                var_name = sync_sub["var_name"]
                checkboxes['sync_sub'][var_name] = checkboxes_with_button(f'`{var_name}` ({len(sync_sub["subs"])} topics)', "✏️", help="Edit", btn_key=var_name, on_click=lambda var=var_name: edit_sync_sub(var))
            
            # Remove selected items
            if st.button("Remove selected items 🗑️", type="primary"):
                st.session_state['gen'].remove_publishers([k for k, v in checkboxes['pub'].items() if v])
                st.session_state['gen'].remove_subscriptions([k for k, v in checkboxes['sub'].items() if v])
                st.session_state['gen'].remove_params([k for k, v in checkboxes['params'].items() if v])
                st.session_state['gen'].remove_timers([k for k, v in checkboxes['timers'].items() if v])
                st.session_state['gen'].remove_services([k for k, v in checkboxes['srv'].items() if v])
                st.session_state['gen'].remove_clients([k for k, v in checkboxes['client'].items() if v])
                st.session_state['gen'].remove_action_servers([k for k, v in checkboxes['action_srv'].items() if v])
                st.session_state['gen'].remove_action_clients([k for k, v in checkboxes['action_client'].items() if v])
                st.session_state['gen'].remove_sync_subscriptions([k for k, v in checkboxes['sync_sub'].items() if v])
                st.rerun()
            
    # Visualize node's graph
    with graph_tab:
        st.graphviz_chart(draw_node())

with col_code:

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

    
    col_command, col_button = st.columns([0.8, 0.2], vertical_alignment='center')
    
    with col_command:
        # Write terminal command for package generation
        with st.expander("ROS2 pkg create command:", expanded=True):
            st.code(f'ros2  pkg create --build-type ament_cmake {st.session_state["gen"]["package_name"]}', language="bash")

    with col_button:
        # Add "Download Package" button
        simple_download_button()
        
    
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
    # st.write(st.session_state['gen'].config)

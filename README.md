# ROS2 Package Generator

Tool for quickly creating ROS2 packages with a C++ or Python node

**Try demo here:** https://ros2-package-generator.onrender.com/

![screenshot](docs/screenshot.png)

### Features

##### ⚡ **Instant Generation**
Complete ROS2 package with proper structure (CMakeLists.txt, package.xml, .hpp, .cpp files) created in seconds with one click.

##### 📝 **Zero Boilerplate Code**
Automatic generation of all template constructs. No more manual writing of:
- Classes inheriting from `rclcpp::Node` / `rclpy.node.Node`
- Publisher/subscriber declarations
- Constructor initialization
- Callback functions
- Node parameters

##### 🐍 **C++ and Python nodes**
Pick a language per node:
- **C++** (`rclcpp`): publishers, subscribers, timers, parameters, services, actions, synchronized subscribers
- **Python** (`rclpy`): publishers, subscribers, timers, parameters, services and a TF listener — actions and synchronized subscribers are C++-only for now (see TODO)

##### 🔄 **All ROS2 Component Types**
Support for all major ROS2 node elements:
- [x] **📤 Publishers** (with QoS, topic, message type configuration)
- [x] **📥 Subscribers** (with auto-generated callback functions)
- [x] **🔧 Services** (servers and clients)
- [x] **⏱️ Timers** (periodic callbacks)
- [x] **⚙️ Parameters** (declaration and initialization)
- [x] **🎯 Actions** (action servers and clients)
- [x] **🔀 Synchronized subscribers** via [message_filters](https://github.com/ros2/message_filters)

##### 🎨 **Visual Constructor**
Intuitive interface for:
- Adding and removing components
- Configuring each element's properties
- Real-time preview of generated code

##### 📊 **Automatic Dependency Resolution**
Smart system that:
- Automatically adds `find_package()` to CMakeLists.txt
- Includes necessary header files
- Forms correct dependencies in package.xml

##### 🔍 **Code Preview**
Live preview of all generated files:
- Node `.hpp` file
- Implementation `.cpp` file
- CMakeLists.txt
- package.xml

##### ⚡ **One-Click Export**
Ready-to-use package structure:
- 📦 **ZIP archive** with complete ROS2 package structure
- 📁 **Automatic structure** (C++, `ament_cmake`):
  ```
  my_package/
  ├── include/my_package/node.hpp
  ├── src/node.cpp
  ├── CMakeLists.txt
  ├── package.xml
  └── README.md
  ```
- 📁 **Automatic structure** (Python, `ament_python`):
  ```
  my_package/
  ├── my_package/
  │   ├── __init__.py
  │   └── node.py
  ├── resource/my_package
  ├── setup.py
  ├── setup.cfg
  └── package.xml
  ```
- 🚀 **Build-ready** with `colcon build`

### Install

Clone repo:
```bash
git clone https://github.com/tamerlan-b/ros2-package-generator.git
```

Create virtual environment:
```bash
python3 -m venv ros2-env
source ros2-env/bin/activate
```

Install deps:
```bash
cd ros2-package-generator
pip install -r requirements.txt
```

### Launch

```bash
streamlit run ros2_package_generator.py
```

### TODO
- [x] Add the ability to edit publishers and subscribers
- [x] Update README
- [x] Add support for:
  - [x] ros2-params
  - [x] ros2-timers
  - [x] ros2-service
  - [x] ros2-actions
  - [x] message_filters
- [x] Add download button for generated files
- [ ] Visualize package structure (with directories)
- [x] Support newer ROS2 distros
- [x] Add Python (`rclpy`) node generation (publishers, subscribers, timers, parameters, services, TF listener)
- [x] Add Python support for actions and synchronized subscribers (`message_filters`)

| ROS2 Distro | Basic support (code can be compiled) | Extended support (new features) |
|-|:-:|:-:|
| galactic | ✅ | ✅ |
| humble | ✅ | ✅ |
| iron | ✅ | ✅ |
| jazzy | ✅ | ❌ |
| lyrical | ❌ | ❌ |


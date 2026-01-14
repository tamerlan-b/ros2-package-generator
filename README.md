# ROS2 Node Generator

Tool for quickly creating ROS2 Foxy packages with nodes

![screenshot](docs/screenshot.png)

### Install

Clone repo:
```bash
git clone https://github.com/tamerlan-b/ros2-node-generator.git
```

Create virtual environment:
```bash
python3 -m venv ros2-env
source ros2-env/bin/activate
```

Install deps:
```bash
cd ros2-node-generator
pip install -r requirements.txt
```

### Launch

```bash
streamlit run ros2_node_generator.py
```

### TODO
- [x] Add the ability to edit publishers and subscribers
- [ ] Update README
- [ ] Add support for:
  - [ ] ros2-params
  - [ ] ros2-timers
  - [ ] ros2-service
  - [ ] ros2-actions
  - [ ] message_filters
- [ ] Support newer ROS2 distros (humble, iron, jazzy, kilted)
- [ ] Add download button for generated files
- [ ] Visualize package structure (with directories)



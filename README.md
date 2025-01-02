# **Homework 1** 🎓🤖

Welcome to Homework 1! Follow the steps below to visualize and interact with the manipulator using Gazebo, RViz, and the onboard camera. 🚀

---

## **Instructions** 🛠️

### **1. Launch the Manipulator in Gazebo** 🌐
To start the manipulator simulation in Gazebo, open your first terminal and run:

```
ros2 launch arm_gazebo arm_gazebo.launch.py
```
### **2. Visualize the Manipulator in RViz**🖼️

In a second terminal, visualize the manipulator's 3D model and its state in RViz by running:
```
ros2 launch arm_description display.launch.py
```
### **3. View the Camera Output** 📷

In a third terminal, you can view the output of the manipulator's camera. Run the following command and select the topic /videocamera from the dropdown menu:
```
ros2 run rqt_image_view rqt_image_view
```
### **4. Start the Controller** 🎮

To control the manipulator, open a fourth terminal and start the controller node with:
```
ros2 run arm_control arm_controller_node
```

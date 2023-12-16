# Hand-Gestures-Control-for-Interbotix-WX-200-Robotic-Arm

# Introduction
- This project focuses on using a hand gesture control device to control the movement of the robotic arm intuitively
- The hand gesture control device is attached on a glove, so that the user can wear the glove and control the robotic arm easily
- Controls of the Robotic arm include pitch, yaw, roll and end-effector linear movements of the robotic arm and also opening/closing of the gripper
- Pitch, yaw, roll data are measured using a GY-85 9DOF IMU sensor, end-effector linear movements are measured using ultrasonic sensor and also, opening/closing   of gripper is measured with a flex sensor
- Such hand gestures control data is transmitted from Arduino to the Interbotix WX200 Robotic Arm

# Data Transmission
- With Arduino, the IMU sensor data are sent over the serial port to the computer (ROS2 C++ node to read array of floats and create sensor_data_publisher)
- Created a publisher node, sensor_data_publisher, which publishes the IMU data to the ROS2 topic, sensor_data
- Created a subscriber node, sensor_data_subscriber, which subscribes to the sensor_data topic and used the services from the Interbotix libraries to process      the IMU data in controlling the movement of the robotic arm

# Services used from in-built ROS2 Packages from WX200 Interbotix Arm
bot_set_single_joint_position(): parameters - "joint_name = " and "position = positional value for servo motor (in radians) = "
- To control a selected single positional motor from one of the joint linkages of the robotic arm

bot_set_ee_cartesian_trajectory(): parameters - "x/y/z = end effector coordinates of robotic arm"
- Input the end effector coordinates of the robotic arm, so that its in-built inverse kinematics engine will control the positional motors accordingly to move     the robotic arm to the given end effector position

bot.gripper.open() and bot.gripper.close()
- To program the gripper of the robotic arm to open and close respectively

# Controlling the Interbotix WX200 Robotic Arm with GY-85 9DOF IMU Sensor, HC-SR04 Ultrasonic Sensor and Flex Sensor (teleop_arm.py)
# GY-85 9DOF IMU Sensor
- Obtaining pitch, roll and yaw data from the IMU sensor, to control the movements of the end effector of the robotic arm (Refer to Arduino code)

# HC-SR04 Ultrasonic Sensor 
- Obtaining distance travelled from the ultrasonic sensor to control the linear movement (in the x-axis) of the end effector of the robotic arm (Refer to          Arduino code)
- The ultrasonic sensor helps to track the head movement of the user leaning forwards and backwards
- Linearly mapping the distance travelled from the ultrasonic sensor to x-coordinate positional values of the end effector of the robotic arm
- User's head leans forward, end effector x-coordinates increases. User's head leans backward, end effector x-coordinataes decreases
- For the linear movement (x-axis) of the end effector of robotic arm, need to control the positional servo motors located in the 'shoulder' and 'elbow' of the    robotic arm
- Considered the robotic arm as a 2DOF arm in the planar view and needed an Inverse Kinematics Solver for 2DOF Robotic Arm to determine the positional values of    the servo positional motors, in order to control the robotic arm to move to the given end effector position 
- Calling the service, bot_set_single_position(joint_name = 'shoulder', position = positional value calculated from IK Solver for 2DOF Robotic Arm)
- Calling the service, bot_set_single_position(joint_name = 'elbow', position = positional value calculated from IK Solver for 2DOF Robotic Arm)

# Flex Sensor
-  When the user clenches his fist with the glove (attached is the hand gestures control device), the flex sensor would bend and its output would control the       robotic arm gripper to close
-  When the user opens his first with the glove, the flex sensor would go back to its orignal position (remain flat) and its output would control the robotic       arm gripper to remain open


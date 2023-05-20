#!/usr/bin/python3

import tkinter as tk
from PIL import Image, ImageTk
import numpy as np
import matplotlib.pyplot as plt
import sys
import rospy

from std_msgs.msg import Int32, String
from sensor_msgs.msg import Image as SensorImage
from sensor_msgs.msg import NavSatFix

import cv2
from cv_bridge import CvBridge


rospy.init_node("gui_node")

# Create the main window
root = tk.Tk()

class State():
    def __init__(self):
        self.heading = None
        self.colour_image = np.zeros((200, 200, 3), dtype=np.uint8)
        self.current_state = "MANUAL"
        self.current_gps = None


state = State()














def on_closing():
    # Perform any cleanup or additional actions before exiting the program
    root.destroy()  # Close the GUI window
    sys.exit()  # Exit the program

root.protocol("WM_DELETE_WINDOW", on_closing)


root.title("Robot State Dashboard")
root.geometry("610x700")

# First Third - Robot Image
first_third = tk.Frame(root, width=600, height=300)
first_third.pack(fill=tk.X)

# Create a frame for the robot images
robot_image_frame = tk.Frame(first_third)
robot_image_frame.pack(side=tk.LEFT, padx=10)

# Create the first robot image label on the left
robot_image_label1 = tk.Label(robot_image_frame, width=200, height=200, bg="black")
robot_image_label1.grid(row=0, column=0, padx=20)

# Create the second robot image label on the right
robot_image_label2 = tk.Label(robot_image_frame, width=200, height=200, bg="black")
robot_image_label2.grid(row=0, column=1, padx=20)

# # Create the second robot image label on the right
# robot_image_label3 = tk.Label(robot_image_frame, width=200, height=200, bg="black")
# robot_image_label3.grid(row=0, column=2, padx=20)


# Generate a black RGB numpy array for robot images
robot_image_array = np.zeros((200, 200, 3), dtype=np.uint8)
robot_image1 = Image.fromarray(robot_image_array)
robot_image_tk1 = ImageTk.PhotoImage(image=robot_image1)
robot_image_label1.config(image=robot_image_tk1)
robot_image_label1.image = robot_image_tk1

robot_image2 = Image.fromarray(robot_image_array)
robot_image_tk2 = ImageTk.PhotoImage(image=robot_image2)
robot_image_label2.config(image=robot_image_tk2)
robot_image_label2.image = robot_image_tk2

# robot_image3 = Image.fromarray(robot_image_array)
# robot_image_t3 = ImageTk.PhotoImage(image=robot_image3)
# robot_image_label3.config(image=robot_image_tk3)
# robot_image_label3.image = robot_image_tk3

# Second Third - Robot State Information
second_third = tk.Frame(root, width=600, height=300)
second_third.pack(fill=tk.X)

# Create bottom frame
bottom_frame = tk.Frame(root, width=600, height=300)
bottom_frame.pack(fill=tk.X)

# Labels and Values
labels = ["GPS", "Heading", "Collision Avoidance", "State", "Distance to Target", "Next Target GPS"]
values = ["", "", "", "", "", ""]

label_value_frames = []
label_labels = []
value_labels = []

# Create label and value labels
for i in range(len(labels)):
    frame = tk.Frame(second_third)
    frame.pack(side=tk.TOP, pady=5)
    label_value_frames.append(frame)

    label = tk.Label(frame, text=labels[i], font=("Arial", 12), width=18, anchor="w")
    label.pack(side=tk.LEFT)
    label_labels.append(label)

    value_label = tk.Label(frame, text=values[i], font=("Arial", 12), width=18, anchor="e")
    value_label.pack(side=tk.RIGHT)
    value_labels.append(value_label)

# Third Third - Subplots
third_third = tk.Frame(root, width=600, height=300)
third_third.pack(fill=tk.X)

# Create subplots
fig, subplots = plt.subplots(2, 5, figsize=(10, 4))

# Display black images in the subplots
for ax in subplots.flatten():
    ax.imshow(np.zeros((100, 100, 3), dtype=np.uint8))
    ax.axis('off')

plt.tight_layout()
# Function to update the robot state
def update_state():
    # Update the values of the robot state here
    current_gps = state.current_gps
    heading_angle = state.heading
    collision_avoidance = True
    robot_state = state.current_state
    distance_to_target = 10.5
    next_target_gps = "37.7749° N, 122.4194° W"

    # Update the GUI labels with the new values
    value_labels[0].config(text=current_gps)
    value_labels[1].config(text=str(heading_angle))
    value_labels[2].config(text=str(collision_avoidance))
    value_labels[3].config(text=robot_state)
    value_labels[4].config(text=str(distance_to_target))
    value_labels[5].config(text=next_target_gps)

    # Convert the cv2 frame to PIL Image

    # Create a CvBridge instance
    bridge = CvBridge()

    # Convert the sensor image to a cv2 frame
    cv_image = bridge.imgmsg_to_cv2(state.colour_image, desired_encoding='bgr8')
    resized_image = cv2.resize(cv_image, (200, 200))
    # Convert the cv2 frame to PIL Image
    robot_image1 = Image.fromarray(cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB))

    robot_image_tk1 = ImageTk.PhotoImage(image=robot_image1)
    robot_image_label1.config(image=robot_image_tk1)
    robot_image_label1.image = robot_image_tk1

    robot_image2 = Image.fromarray(robot_image_array)
    robot_image_tk2 = ImageTk.PhotoImage(image=robot_image2)
    robot_image_label2.config(image=robot_image_tk2)
    robot_image_label2.image = robot_image_tk2

    # Update the small images with black images
    small_image_array = np.zeros((100, 100, 3), dtype=np.uint8)
    # Create small image labels
    small_image_labels = []

    # Create the labels for small images
    for i in range(10):
        label_frame = tk.Frame(bottom_frame, width=100, height=100, bg="white")
        label_frame.grid(row=int(i/5), column=i%5, padx=10, pady=5)

        label = tk.Label(label_frame, width=100, height=100, bg="black")
        label.pack()

        small_image_labels.append(label)

    for label in small_image_labels:
        image = Image.fromarray(small_image_array)
        image_tk = ImageTk.PhotoImage(image=image)
        label.config(image=image_tk)
        label.image = image_tk

    # Update the subplots with black images
    for ax in subplots.flatten():
        ax.imshow(np.zeros((100, 100, 3), dtype=np.uint8))
        ax.axis('off')


if __name__ == "__main__":
    rospy.Subscriber('/imu_heading', Int32, lambda data: setattr(state, 'heading', data.data))
    rospy.Subscriber('/gui/colour_image', SensorImage, lambda data: setattr(state, 'colour_image', data))
    rospy.Subscriber('/fix', NavSatFix, lambda data: setattr(state, 'current_gps', f"{data.latitude}, {data.longitude}"))
    rospy.Subscriber('/gui/current_state', String, lambda data: setattr(state, 'current_state', data.data))

    rate = rospy.Rate(500)

    while not rospy.is_shutdown():
        root.update()  # Process any pending events in the GUI

        # Your GUI-related code here, if needed
        update_state()

        # rate.sleep()  # Sleep according to the desired rate

    root.mainloop()  # This line is executed after the while loop exits (ROS shutdown)

#         update_state()
        # Function to update the robot state and GUI
        # def update_state_and_gui():
            # root.after(1000, update_state_and_gui)

        # Schedule the first update
        # root.after(1000, update_state_and_gui)
        # rospy.spin()
# # Schedule the next update after 1 second
# root.after(1000, update_state)

# Start the GUI main loop

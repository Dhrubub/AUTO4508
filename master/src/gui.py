#!/usr/bin/python3

import tkinter as tk
from PIL import Image, ImageTk
import numpy as np
import matplotlib.pyplot as plt
import sys
import rospy

from std_msgs.msg import Int32, String, Float32, Bool, Float32MultiArray
from sensor_msgs.msg import Image as SensorImage
from sensor_msgs.msg import NavSatFix

import cv2
from cv_bridge import CvBridge
from enum import Enum
import math



rospy.init_node("gui_node")

# =============================================================================================
# NW = {"lat": -31.979722, "lon": 115.817004}
# NE = {"lat": -31.979722, "lon":	115.818504}
# SE = {"lat": -31.980922, "lon": 115.818504}
# SW = {"lat": -31.980922, "lon": 115.817004}

NW = {"lat": -31.979922, "lon": 115.817004}
NE = {"lat": -31.979922, "lon":	115.818204}
SE = {"lat": -31.980822, "lon": 115.818204}
SW = {"lat": -31.980822, "lon": 115.817004}

NW = {"lat": -31.980222, "lon": 115.817204}
NE = {"lat": -31.980222, "lon":	115.818004}
SE = {"lat": -31.980722, "lon": 115.818004}
SW = {"lat": -31.980722, "lon": 115.817204}

bbox = {
    'left' : SW['lon'],
    'right': NE['lon'],
    'top': NE['lat'],
    'bottom': SW['lat'],
    
}

bounds_coords = {
    'NW': (bbox['left'], bbox['top']),
    'NE': (bbox['right'], bbox['top']),
    'SE': (bbox['right'], bbox['bottom']),
    'SW': (bbox['left'], bbox['bottom'])
}

coords = [
    {'lat': -31.980622, 'lon': 115.817494},
    {'lat': -31.980326, 'lon': 115.81747},
    {'lat': -31.98016, 'lon': 115.817204},
    {'lat': -31.980058, 'lon': 115.817542}
]



class Colour(Enum):
    EMPTY = [0, 0, 0]
    PATH = [255, 255, 255]
    CONE = [255, 127, 0]
    OBSTACLE = [0, 255, 0]
    BUCKET = [0, 127, 255]

# =============================================================================================







# =============================================================================================
# Create the main window
root = tk.Tk()

class State():
    def __init__(self):
        self.heading = None
        self.distance = None

        empty_image = np.zeros((200, 200, 3), dtype=np.uint8)
        # Assuming your cv2 frame is named 'frame'
        ros_image = SensorImage()
        ros_image.header.stamp = rospy.Time.now()
        ros_image.header.frame_id = 'your_frame_id'
        ros_image.height = empty_image.shape[0]
        ros_image.width = empty_image.shape[1]
        ros_image.encoding = 'bgr8'  # Specify the image encoding (e.g., 'bgr8' for BGR color format)
        ros_image.data = empty_image.tobytes()  # Convert the frame to a byte string
        ros_image.step = len(ros_image.data) // ros_image.height  # Compute the byte step size
    
        self.colour_image = ros_image

        empty_image = np.zeros((100, 300, 3), dtype=np.uint8)
        # Assuming your cv2 frame is named 'frame'
        ros_image = SensorImage()
        ros_image.header.stamp = rospy.Time.now()
        ros_image.header.frame_id = 'your_frame_id'
        ros_image.height = empty_image.shape[0]
        ros_image.width = empty_image.shape[1]
        ros_image.encoding = 'bgr8'  # Specify the image encoding (e.g., 'bgr8' for BGR color format)
        ros_image.data = empty_image.tobytes()  # Convert the frame to a byte string
        ros_image.step = len(ros_image.data) // ros_image.height  # Compute the byte step size
    
        self.colour_image = ros_image



        self.output_image = ros_image
        self.distance = None

        self.captures = []
        # for _ in range(10):
        #     empty_image = np.zeros((100, 100, 3), dtype=np.uint8)
        #     ros_image = SensorImage()
        #     ros_image.header.stamp = rospy.Time.now()
        #     ros_image.header.frame_id = 'your_frame_id'
        #     ros_image.height = empty_image.shape[0]
        #     ros_image.width = empty_image.shape[1]
        #     ros_image.encoding = 'bgr8'  # Specify the image encoding (e.g., 'bgr8' for BGR color format)
        #     ros_image.data = empty_image.tobytes()  # Convert the frame to a byte string
        #     ros_image.step = len(ros_image.data) // ros_image.height  # Compute the byte step size

        #     self.captures.append(ros_image)




        
        self.current_state = "MANUAL"
        self.current_gps = None
        self.current_target_gps = None
        self.grid = np.full((200, 200, 3), Colour.EMPTY.value)



    def convert_to_cartesian(self, coords, size=200):
        x_dist = abs(bbox['right'] - bbox['left'])
        y_dist = abs(bbox['top'] - bbox['bottom'])

        x = int(abs(coords['lon'] - bbox['left']) / x_dist * size)
        y = int(abs(coords['lat'] - bbox['top']) / y_dist * size)

        return (x, y)
    
    def add_point(self, coords, colour):
        x, y = self.convert_to_cartesian(coords)
        if np.array_equal(self.grid[y][x], Colour.EMPTY.value):
            if not np.array_equal(colour.value, Colour.PATH.value):
                if y-1 > 0: self.grid[y-1][x] = colour.value
                if y+1 < 200: self.grid[y+1][x] = colour.value
                if x-1 > 0: self.grid[y][x-1] = colour.value
                if x+1 < 200: self.grid[y][x+1] = colour.value

            if 0 <= y < 200 and 0 <= x < 200:
                self.grid[y][x] = colour.value


state = State()

def on_closing():
    # Perform any cleanup or additional actions before exiting the program
    root.destroy()  # Close the GUI window
    sys.exit()  # Exit the program

root.protocol("WM_DELETE_WINDOW", on_closing)


root.title("Robot State Dashboard")
root.geometry("900x700")

# First Third - Robot Image
first_third = tk.Frame(root, width=600, height=300)
first_third.pack(fill=tk.X)

# Create a frame for the robot images
robot_image_frame = tk.Frame(first_third)
robot_image_frame.pack(side=tk.LEFT, padx=10)

# Create the first robot image label on the left
robot_image_label1 = tk.Label(robot_image_frame, width=200, height=200, bg="black")
robot_image_label1.grid(row=0, column=0, padx=25)

# Create the third robot image label in the middle
robot_image_label3 = tk.Label(robot_image_frame, width=300, height=100, bg="black")
robot_image_label3.grid(row=0, column=1, padx=25)

# Create the second robot image label on the right
robot_image_label2 = tk.Label(robot_image_frame, width=200, height=200, bg="black")
robot_image_label2.grid(row=0, column=2, padx=25)



# Generate a black RGB numpy array for robot images
robot_image_array = np.zeros((200, 200, 3), dtype=np.uint8)
robot_image1 = Image.fromarray(robot_image_array)
robot_image_tk1 = ImageTk.PhotoImage(image=robot_image1)
robot_image_label1.config(image=robot_image_tk1)
robot_image_label1.image = robot_image_tk1

robot_image3 = Image.fromarray(robot_image_array)
robot_image_tk3 = ImageTk.PhotoImage(image=robot_image3)
robot_image_label3.config(image=robot_image_tk3)
robot_image_label3.image = robot_image_tk3

robot_image2 = Image.fromarray(robot_image_array)
robot_image_tk2 = ImageTk.PhotoImage(image=robot_image2)
robot_image_label2.config(image=robot_image_tk2)
robot_image_label2.image = robot_image_tk2


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

    label = tk.Label(frame, text=labels[i], font=("Arial", 9), width=24, anchor="w")
    label.pack(side=tk.LEFT)
    label_labels.append(label)

    value_label = tk.Label(frame, text=values[i], font=("Arial", 9), width=24, anchor="e")
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

# =============================================================================================

# Function to update the robot state
def update_state():
    # Update the values of the robot state here
    current_gps = state.current_gps
    heading_angle = state.heading
    collision_avoidance = ''
    robot_state = state.current_state
    distance_to_target = state.distance
    next_target_gps = current_target_gps

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


   # Convert the sensor image to a cv2 frame
    cv_image = bridge.imgmsg_to_cv2(state.output_image, desired_encoding='bgr8')
    resized_image = cv2.resize(cv_image, (300, 100))
    # Convert the cv2 frame to PIL Image
    robot_image3 = Image.fromarray(cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB))

    robot_image_tk3 = ImageTk.PhotoImage(image=robot_image3)
    robot_image_label3.config(image=robot_image_tk3)
    robot_image_label3.image = robot_image_tk3

    map_grid = state.grid.astype(np.uint8)
    robot_image2 = Image.fromarray(map_grid)
    robot_image_tk2 = ImageTk.PhotoImage(image=robot_image2)
    robot_image_label2.config(image=robot_image_tk2)
    robot_image_label2.image = robot_image_tk2

    # Update the small images with black images
    # Create small image labels
    small_image_labels = []

    # Create the labels for small images
    for i in range(len(state.captures)):
        label_frame = tk.Frame(bottom_frame, width=100, height=100, bg="white")
        label_frame.grid(row=int(i/6), column=i%6, padx=10, pady=5)

        label = tk.Label(label_frame, width=100, height=100, bg="black")
        label.pack()

        small_image_labels.append(label)

    for i, label in enumerate(small_image_labels):
        image_tk = ImageTk.PhotoImage(image=state.captures[i])
        label.config(image=image_tk)
        label.image = image_tk

    # Update the subplots with black images
    for ax in subplots.flatten():
        ax.imshow(np.zeros((100, 100, 3), dtype=np.uint8))
        ax.axis('off')

def update_captures(data):
    bridge = CvBridge()

    # Convert the sensor image to a cv2 frame
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    resized_image = cv2.resize(cv_image, (100, 100))
    # Convert the cv2 frame to PIL Image
    robot_image = Image.fromarray(cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB))

    state.captures.append(robot_image)

pixel_colour = Colour.PATH


def gps_callback(data):
    global pixel_colour
    state.current_gps = f"{data.latitude:.6f}, {data.longitude:.6f}"
    coord = {'lat': data.latitude , 'lon': data.longitude}
    if not (math.isnan(coord['lat']) or math.isnan(coord['lon'])):
        state.add_point(coord, pixel_colour)
        pixel_colour = Colour.PATH

def target_reached(data):
    global pixel_colour
    if data.data:
        pixel_colour = Colour.CONE


def current_state(data):
    global pixel_colour
    state.current_state = data.data
    if state.current_state == "OBSTACLE_AVOIDING":
        pixel_colour = Colour.OBSTACLE


def bucket_capture(data):
    coord = {'lat': data.data[0] , 'lon': data.data[1]}
    state.add_point(coord, COLOR.BUCKET)
    
def current_target_gps(data):
    state.current_target_gps = f"{data[0]:.6f}, {data[1]:.6f}"




if __name__ == "__main__":
    rospy.Subscriber('/imu_heading', Int32, lambda data: setattr(state, 'heading', data.data))
    rospy.Subscriber('/gui/colour_image', SensorImage, lambda data: setattr(state, 'colour_image', data))
    rospy.Subscriber('/gui/output_image', SensorImage, lambda data: setattr(state, 'output_image', data))
    rospy.Subscriber('/fix', NavSatFix, gps_callback)
    rospy.Subscriber('/gui/current_state', String, current_state)
    rospy.Subscriber('/gui/capture', SensorImage, update_captures)
    rospy.Subscriber('/gui/distance', Float32, lambda data: setattr(state, 'distance', data.data))
    rospy.Subscriber('/target_reached', Bool, target_reached)
    rospy.Subscriber('/gui/bucket', Float32MultiArray, bucket_capture)
    rospy.Subscriber('/gui/current_target', Float32MultiArray, current_target_gps)


    rate = rospy.Rate(500)

    coords = [
        {'lat': -31.980622, 'lon': 115.817494},
        {'lat': -31.980326, 'lon': 115.81747},
        {'lat': -31.98016, 'lon': 115.817204},
        {'lat': -31.980058, 'lon': 115.817542}
    ]

    while not rospy.is_shutdown():
        root.update()  # Process any pending events in the GUI

        # Your GUI-related code here, if needed
        update_state()


    root.mainloop()  # This line is executed after the while loop exits (ROS shutdown)


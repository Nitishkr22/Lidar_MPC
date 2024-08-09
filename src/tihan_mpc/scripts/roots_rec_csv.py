#!/usr/bin/env python
import rospy
import math as m
import numpy as np
import csv
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import NavSatFix, Imu
# from novatel_oem7_msgs.msg import INSPVA
from std_msgs.msg import Int32
from std_msgs.msg import Float32 as Float32Msg
from sensor_msgs.msg import Imu

import time
from geometry_msgs.msg import PoseStamped, Vector3
import pandas as pd
from scipy.io import savemat
import yaml

# Initialize global variables
lat = lon = x = y = 0.0
v = v_long = v_lat = 0.0
psi = long_accel = lat_accel = yaw_rate = 0.0
df = 0

# Flag to check if first valid data has been received
first_valid_data_received = False

# Flag to check if the first row has been written
first_row_written = False

# Initialize CSV file
csv_file = "../paths/zedtest.csv"
headers = ["a", "psi", "df", "lon", "lat", "mode", "v", "y", "x", "t"]

with open(csv_file, mode='w', newline='') as file:
    writer = csv.DictWriter(file, fieldnames=headers)
    writer.writeheader()

# ROS node initialization
rospy.init_node('state_publisher', anonymous=True)

def extract_ros_time(msg):
    return msg.header.stamp.secs + 1e-9 * msg.header.stamp.nsecs

def _parse_gps_fix(msg):
    global lat, lon, x, y,first_valid_data_received
    lat = msg.latitude
    lon = msg.longitude
    x, y = latlon_to_XY(lat, lon)
    first_valid_data_received = True
    # tm_gps = extract_ros_time(msg)

def _parse_gps_vel(msg):
    global psi, v, v_long, v_lat,first_valid_data_received
    if psi == 0.0:
        return  # Wait until we get the orientation first

    x_vel = msg.x
    y_vel = msg.y
    
    v = m.sqrt(x_vel**2 + y_vel**2)
    v_long = m.cos(psi) * x_vel + m.sin(psi) * y_vel
    v_lat = -m.sin(psi) * x_vel + m.cos(psi) * y_vel
    first_valid_data_received = True
    # tm_vel = extract_ros_time(msg)

def _parse_imu_data(msg):
    global long_accel, lat_accel, psi, yaw_rate,first_valid_data_received

    long_accel = msg.linear_acceleration.x
    lat_accel = -msg.linear_acceleration.y
    yaw_rate = msg.angular_velocity.z
    first_valid_data_received = True
    # tm_imu = extract_ros_time(msg)
        

def _parse_steering_angle(msg):
    global df,first_valid_data_received
    # df = m.radians(msg.data) / 15.87
    df = m.radians(msg.data) / 10.57
    first_valid_data_received = True
    # tm_df = time.time()

def latlon_to_XY(lat1, lon1, lat0=0.0, lon0=0.0):
    # R_earth = 6371000
    # delta_lat = m.radians(lat1 - lat0)
    # delta_lon = m.radians(lon1 - lon0)
    # lat_avg = 0.5 * (m.radians(lat1) + m.radians(lat0))
    # X = R_earth * delta_lon * m.cos(lat_avg)
    # Y = R_earth * delta_lat
    X = float(lat1)
    Y = float(lon1)
    return X, Y

def ndt_pose_callback(data):
    # curr_time = data.header.stamp.to_sec()
    global lat, lon, x, y, psi,first_valid_data_received
    lat = data.pose.position.x
    lon = data.pose.position.y
    x, y = lat,lon
    tm_gps = extract_ros_time(data)
    ##################################################
    ori = data.pose.orientation
    quat = (ori.x, ori.y, ori.z, ori.w)
    _, _, yaw = euler_from_quaternion(quat)
    # psi = -yaw + 0.5 * m.pi
    psi = yaw
    psi = (psi + np.pi) % (2. * np.pi) - np.pi
    first_valid_data_received = True

# ROS subscribers
# rospy.Subscriber('/gps/fix', NavSatFix, _parse_gps_fix, queue_size=1)

# rospy.Subscriber("/novatel/oem7/inspva", INSPVA, _parse_gps_vel, queue_size=1)
# rospy.Subscriber('/steering_angle', Float32Msg, _parse_steering_angle, queue_size=1)

rospy.Subscriber('/ndt_pose', PoseStamped, ndt_pose_callback)
# rospy.Subscriber('/livox/imu', Imu, _parse_imu_data, queue_size=1)
rospy.Subscriber('/zed/imu', Imu, _parse_imu_data, queue_size=1)

rospy.Subscriber('/calculated_velocity', Vector3,_parse_gps_vel, queue_size=10)
rospy.Subscriber('/steering_angle', Int32, _parse_steering_angle, queue_size=1)


# Main loop to print and save data
with open(csv_file, mode='a', newline='') as file:
    writer = csv.DictWriter(file, fieldnames=headers)
    
    while not rospy.is_shutdown():
        if first_valid_data_received:
            current_time = time.time()
            data_row = {
                "a": long_accel,
                "psi": psi,
                "df": df,
                "lon": lon,
                "lat": lat,
                "mode": "Real" if not first_row_written else "",  # Set "Real" for the first row
                "v": v,
                "y": y,
                "x": x,
                "t": current_time
            }
            
            writer.writerow(data_row)
            first_row_written = True  # Mark the first row as written
            
            print("lat: ", lat)
            print("lon: ", lon)
            print("psi: ", psi)
            print("v: ", v)
            print("v_long: ", v_long)
            print("v_lat: ", v_lat)
            print("steer", df)
            print("X: ",x)
            print("Y: ",y)
            print("---------------------------------------------------------------------------------------------")
            
        rospy.sleep(0.1)  # Adjust the sleep time as needed

# Convert CSV to MAT file and create YAML file after ROS shutdown

def convert_csv_to_mat_and_yaml(csv_file):
    # Load the CSV file into a DataFrame
    df = pd.read_csv(csv_file)

    # Extract the first entry of "mode" and convert it to a shape of (1,)
    mode_first_entry = np.array([df["mode"].values[0]])

    # Convert the rest of the variables to 1xN arrays
    variables = {
        "a": np.array([df["a"].values]),
        "psi": np.array([df["psi"].values]),
        "df": np.array([df["df"].values]),
        "lon": np.array([df["lon"].values]),
        "lat": np.array([df["lat"].values]),
        "mode": mode_first_entry,
        "v": np.array([df["v"].values]),
        "y": np.array([df["y"].values]),
        "x": np.array([df["x"].values]),
        "t": np.array([df["t"].values])
    }

    # Save the dictionary of arrays to a .mat file
    savemat(csv_file.replace(".csv", ".mat"), variables)
    print("MAT file saved successfully.")

    # Calculate X0 and Y0 using the first two rows of lat and lon
    lat0 = df["lat"].values[0]
    lon0 = df["lon"].values[0]
    lat1 = df["lat"].values[1]
    lon1 = df["lon"].values[1]

    X0, Y0 = latlon_to_XY(lat0, lon0, lat1, lon1)

    # Create YAML file with initial values
    yaml_file = csv_file.replace("paths", "launch").replace(".csv", ".yaml")
    initial_values = {
        "lat0": float(lat0),
        "lon0": float(lon0),
        "yaw0": 0.0,
        "X0": X0,
        "Y0": Y0,
        "Psi0": float(df["psi"].values[0]),
        "V0": 0.0
    }

    with open(yaml_file, 'w') as file:
        yaml.dump(initial_values, file, default_flow_style=False)
    print("YAML file saved successfully.")

rospy.on_shutdown(lambda: convert_csv_to_mat_and_yaml(csv_file))

rospy.spin()

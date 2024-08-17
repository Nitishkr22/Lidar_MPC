from std_msgs.msg import Float32 as Float32Msg
import rospy
import time
import math as m
from geometry_msgs.msg import Quaternion, Vector3
import csv

min_distance = 45
last_received_time = None
timeout = 0.2  # Time in seconds to wait before resetting min_distance
data_received = False
def obj_distance(msg):
    global min_distance, last_received_time, data_received
    min_distance = msg.data
    last_received_time = time.time()
    data_received = True

feedback_speed = 0
def _parse_gps_vel(msg):
    global feedback_speed, data_received

    x_vel = msg.x
    y_vel = msg.y
    
    feedback_speed = m.sqrt(x_vel**2 + y_vel**2) * 3.6
    data_received = True

def create_csv_file(filename='vehicle_data.csv'):
    """Creates a CSV file and writes the header row."""
    with open(filename, 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(['Timestamp', 'Min Distance', 'Feedback Speed'])
    print(f"CSV file '{filename}' created and initialized.")

def append_to_csv(filename, min_distance, feedback_speed):
    """Appends a new row of data to the CSV file."""
    with open(filename, 'a', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow([time.time(), min_distance, feedback_speed])


if __name__ == '__main__':
    rospy.init_node('distance_reader')
    rospy.Subscriber('/zed_distance', Float32Msg, obj_distance, queue_size=10)
    rospy.Subscriber('/calculated_velocity', Vector3,_parse_gps_vel, queue_size=10)

    rate = rospy.Rate(20)

    create_csv_file('vehicle_data.csv')
    while not rospy.is_shutdown():
        print("speed: ",feedback_speed)
        if last_received_time and (time.time() - last_received_time > timeout):
            min_distance = 45
        print("distance: ",min_distance)
        if data_received:
            append_to_csv('vehicle_data.csv', min_distance, feedback_speed)
        rate.sleep()





        # if (coll==2):
        #     accelerate(int(0))
        #     if(brake_counter<1):
        #         apply_brake(0.8)  # 0.8 to apply full brake
        #         brake_counter=brake_counter+1
        #     upvel = 0

        # elif (coll==1):
            
        #     if(brake_counter<1):
        #         vel = 21
        #         apply_brake(0.5)
        #         brake_counter=brake_counter+1
        #     # apply_brake(0.2)
        #     vel = vel-1
        #     set_steer(int(steer_output/2.8))
        #     upvel = vel
        #     upvel = max(0,upvel)
        #     accelerate(int(upvel))
        #     prev_vel = upvel
        #     rate.sleep()
        #     continue
        #     # if vel>25:
        #     #     upvel = 26
        #     # else:
        #     #     upvel = vel
        #     # accelerate(int(upvel))
            
        # else:
        #     brake_counter = 0
        #     # set_forward()
        #     remove_brake()
        #     # set_forward()
        # brake_counter = 0
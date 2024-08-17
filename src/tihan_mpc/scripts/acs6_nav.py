from pymodbus.client import ModbusTcpClient as ModbusClient 
import time
import rospy
import math
import atexit
import time
import numpy as np
from std_msgs.msg import String 
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float32, Float64, Float32MultiArray, Float64MultiArray, Int32MultiArray
import os
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32 as Float32Msg
from tihan_mpc.msg import mpc_path
import math as m
from geometry_msgs.msg import Quaternion, Vector3
import csv
# import pid

host = '192.168.140.5'  
port = 502
client = ModbusClient(host, port)

client.connect()
UNIT= 0x1
#client.set_slave(UNIT)
end_point = False
coll = 0
data_received = False

print("connected")
# brake_counter=0
def read_angle():
   read1 = client.read_holding_registers(address=1, count=1, unit=UNIT)
   current_angle = read1.registers[0]
   # curr_ang = current_angle * 0.35
   normalised_angle = current_angle-482
   angle_pub.publish(normalised_angle)
   # print("current steering: ",normalised_angle)
   
   return normalised_angle

def set_steer(angle):
    if(angle<0):
        y=abs(angle)
        write2= client.write_coils(20, True, unit=UNIT) 
        Write = client.write_registers(400,y, unit=UNIT) 
        time.sleep(0.1)
    else:
        
        write2= client.write_coils(20, False, unit=UNIT)
        Write = client.write_registers(400,angle, unit=UNIT)

def set_forward():
    Write3 = client.write_coils(502,True, unit=UNIT) #502 #11
    Write4 = client.write_coils(504, False, unit=UNIT)
def set_reverse():
    Write3 = client.write_coils(502,False, unit=UNIT)
    Write4 = client.write_coils(504, True, unit=UNIT)
def set_neutral():
    Write3 = client.write_coils(502,False, unit=UNIT)
    Write4 = client.write_coils(504, False, unit=UNIT)
 
   
def remove_brake():
    R_pwm = client.write_coil(4, True, unit=UNIT) 
    L_PWM = client.write_coil(3, False, unit=UNIT)

def apply_brake(seed):
    R_pwm = client.write_coil(3, True, unit=UNIT) 
    L_PWM = client.write_coil(4, False, unit=UNIT)
    time.sleep(seed)
    R_pwm = client.write_coil(3, False, unit=UNIT) 
    L_PWM = client.write_coil(4, False, unit=UNIT)

def accelerate(value):
    #value1=value*10
    Write2 = client.write_registers(500,value, unit=UNIT)
    
# global previous_velocity
# previous_velocity=0
def increase_velocity(previous_velocity, velocity):
    #time.sleep(0.25)
    if velocity > previous_velocity:
       new_velocity=previous_velocity+0.2
       return min(41,new_velocity)
    else:
       return velocity
    
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

steermpc = 0.0
def steering_mpc(msg):
    global steermpc, data_received
    steermpc = msg.data
    data_received = True

def update_mpc_trajectory(msg):
    global end_point,data_received

    end_point = msg.wp_end
    data_received = True

def zed_collision(msg):
    global coll,data_received
    coll = msg.data
    data_received = True

feedback_speed = 0
def _parse_gps_vel(msg):
    global feedback_speed,data_received

    x_vel = msg.x
    y_vel = msg.y
    
    feedback_speed = m.sqrt(x_vel**2 + y_vel**2) * 3.6
    data_received = True

min_distance = 100
last_received_time = None
timeout = 0.2  # Time in seconds to wait before resetting min_distance
def obj_distance(msg):
    global min_distance, last_received_time,data_received
    min_distance = msg.data
    last_received_time = time.time()
    data_received = True

def find_closest_key(dictionary, target_value):
    # Find the key that corresponds to the closest value in the dictionary
    closest_key = min(dictionary, key=lambda k: abs(dictionary[k] - target_value))
    return closest_key
# while(1):
#     remove_brake()
if __name__ == '__main__':
    # dict_speed dictionary contains key:value pair of accn(voltage_values):velocity(kmph) for the DBW system
    dict_speed = {0:0,1:0,2:0,3:0,4:0,5:0,6:0,7:0,8:0,9:0,
            10:0,11:0,12:1,13:1,14:2,15:2,16:3,17:3,18:4,19:4,20:5,21:5,22:6,
            23:6,24:7,25:7,26:8,27:8,28:9,29:10,30:11,31:12,32:12,33:13,34:14,35:15,
            36:16,37:17,38:18,39:19,40:20,41:20,42:21,43:22,44:23,45:24}
    rospy.init_node('angle_reader')
    angle_pub = rospy.Publisher('current_steer', Int32, queue_size=10)
    rospy.Subscriber("/control/steer_angle", Float32Msg, steering_mpc, queue_size=1)
    rospy.Subscriber('/vehicle/mpc_path', mpc_path, update_mpc_trajectory, queue_size=1)
    rospy.Subscriber('/collision', Float32Msg, zed_collision, queue_size=1)
    rospy.Subscriber('/calculated_velocity', Vector3,_parse_gps_vel, queue_size=10)
    rospy.Subscriber('/zed_distance',Float32Msg,obj_distance,queue_size=10)

    rate = rospy.Rate(10)
    keys_with_value = find_closest_key(dict_speed, 24)
    desired_angle = 0
    prev_vel = 0
    brake_counter = 0
    k = 2.0
    Vmax = 40.0
    stop_dist = 5.0
    # vel = 0
    i = 0

    create_csv_file('vehicle_data_16.csv')

    while not rospy.is_shutdown():
        if last_received_time and (time.time() - last_received_time > timeout):
            min_distance = 100
        print(feedback_speed)
        target_vel = 16#kmph
        acc_value = find_closest_key(dict_speed, target_vel)
        initial_vel = find_closest_key(dict_speed, int(feedback_speed))
        feedback_angle = read_angle()
        steer_output = int(math.degrees(steermpc)*10.57)
        Write = client.write_registers(22,2300, unit=UNIT)
        # set_neutral()
        set_forward()
        # vel = increase_velocity(prev_vel,35)
        # vel = increase_velocity(initial_vel,acc_value)
        vel = increase_velocity(prev_vel,acc_value)
        # if(vel==prev_vel):
        #     prev_vel +=1
        # prev_vel = vel
        print("steering feedback: ",feedback_angle)
        print("mpc angle: ",steer_output)

        if abs(steer_output)<30:
           upvel = vel
        else:
            if vel>27:
                upvel = 29
            else:
                upvel = vel

        # accelerate(int(upvel))
        print(f"Steer Output: {steer_output}")
        print("sssssssss: ",end_point)
        print("ccccccccc: ",coll)
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
        print(",mmmmmmm: ",min_distance)
        
        acc_coll = max(0,41*(1 - np.exp(-(k/Vmax)*(min_distance - stop_dist))))
        upvel = min(acc_coll,upvel)
        diff = int(feedback_speed)-dict_speed[int(upvel)]
        if diff>2:
            i = i+0.0045
            i = min(0.8,i)
            # if(brake_counter<1):
            apply_brake(i)  # 0.8 to apply full brake
                # brake_counter=brake_counter+1
        else:
            # brake_counter = 0
            i = 0
            remove_brake()
        
        if(min_distance<stop_dist):
            if(brake_counter<1):
                apply_brake(0.8)  # 0.8 to apply full brake
                brake_counter=brake_counter+1
        else:
            brake_counter = 0
            remove_brake()
        print("accccacacac: ",acc_coll)
        # upvel = min(acc_coll,upvel)
        accelerate(int(upvel))
        prev_vel = upvel
        if (end_point):
            accelerate(int(0))
            # set_neutral()
        # print(brake_counter)

        set_steer(int(steer_output/2.8))
        if data_received:
            append_to_csv('vehicle_data_16.csv', min_distance, feedback_speed)
        
        rate.sleep()

        
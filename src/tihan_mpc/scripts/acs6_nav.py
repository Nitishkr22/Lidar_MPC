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
# import pid

host = '192.168.140.5'  
port = 502
client = ModbusClient(host, port)

client.connect()
UNIT= 0x1
#client.set_slave(UNIT)
end_point = False

print("connected")

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
    R_pwm = client.write_coil(3, True, unit=UNIT) 
    L_PWM = client.write_coil(4, False, unit=UNIT)

def accelerate(value):
    #value1=value*10
    Write2 = client.write_registers(500,value, unit=UNIT)
    
global previous_velocity
previous_velocity=0
def increase_velocity(previous_velocity, velocity):
    #time.sleep(0.25)
    if velocity > previous_velocity:
       new_velocity=previous_velocity+0.2
       return min(45,new_velocity)
    else:
       return velocity
        
def exit():
   #  accelerate(0)
   #  set_neutral()
    set_steer(0)
    time.sleep(0.1)
   #  remove_brake()

# class PIDController:
#     def __init__(self, Kp, Ki, Kd, output_limits=(-134, 134)):
#         self.Kp = Kp
#         self.Ki = Ki
#         self.Kd = Kd
#         self.output_limits = output_limits
#         self.integral = 0
#         self.previous_error = 0
#         self.feedforward_gain = 134 / 370
    
#     def update(self, setpoint, feedback):
#         # Normalize angles from [-370, 370] to [-1, 1]
#         setpoint_normalized = setpoint / 370
#         feedback_normalized = feedback / 370
        
#         # Calculate error
#         error = setpoint_normalized - feedback_normalized
        
#         # Proportional term
#         P = self.Kp * error
        
#         # Integral term
#         self.integral += error
#         I = self.Ki * self.integral
        
#         # Derivative term
#         derivative = error - self.previous_error
#         D = self.Kd * derivative
        
#         # PID output
#         output = P + I + D
        
#         # Feedforward term to directly drive to the setpoint
#         feedforward = self.feedforward_gain * setpoint
        
#         # Combined output
#         output_combined = output + feedforward
        
#         # Save error for next derivative calculation
#         self.previous_error = error
        
#         # Apply output limits
#         output_combined = max(self.output_limits[0], min(self.output_limits[1], output_combined))
        
#         return output_combined
steermpc = 0.0
def steering_mpc(msg):
    global steermpc
    steermpc = msg.data

def update_mpc_trajectory(msg):
    global end_point

    end_point = msg.wp_end
# while 1:
#     set_forward()

if __name__ == '__main__':
    rospy.init_node('angle_reader')
    angle_pub = rospy.Publisher('current_steer', Int32, queue_size=10)
    rospy.Subscriber("/control/steer_angle", Float32Msg, steering_mpc, queue_size=1)
    rospy.Subscriber('/vehicle/mpc_path', mpc_path, update_mpc_trajectory, queue_size=1)

    rate = rospy.Rate(10) 

   # Tune PID
#    pid = PIDController(Kp=1.5574, Ki=2.1085, Kd=1.0655)

    desired_angle = 0
    prev_vel = 0
    while not rospy.is_shutdown():
        feedback_angle = read_angle()
        steer_output = int(math.degrees(steermpc)*10.9)
        Write = client.write_registers(22,2300, unit=UNIT)
        # set_neutral()
        set_forward()
        # set_reverse()
        # accelerate(25)
        vel = increase_velocity(prev_vel,35)
        # prev_vel = vel
        # accelerate(int(vel))
        print("steering feedback: ",feedback_angle)
        print("mpc angle: ",steer_output)
        if abs(steer_output)<30:
           upvel = vel
        else:
            if vel>26:
                upvel = 30
            else:
                upvel = vel
        accelerate(int(upvel))
        prev_vel = upvel
    #   steer_input = pid.update(desired_angle, feedback_angle)
        print(f"Steer Output: {steer_output}")
        print("sssssssss: ",end_point)
        if (end_point==True):
            accelerate(int(0))
            set_neutral()


        set_steer(int(steer_output/2.8))
        rate.sleep()

        
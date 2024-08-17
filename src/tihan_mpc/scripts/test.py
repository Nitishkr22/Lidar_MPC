from std_msgs.msg import Float32 as Float32Msg
import rospy
import time

min_distance = 45
last_received_time = None
timeout = 0.2  # Time in seconds to wait before resetting min_distance

def obj_distance(msg):
    global min_distance, last_received_time
    min_distance = msg.data
    last_received_time = time.time()

if __name__ == '__main__':
    rospy.init_node('distance_reader')
    rospy.Subscriber('/zed_distance', Float32Msg, obj_distance, queue_size=10)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if last_received_time and (time.time() - last_received_time > timeout):
            min_distance = 45
        print(min_distance)
        rate.sleep()

def find_closest_key(dictionary, target_value):
    # Find the key that corresponds to the closest value in the dictionary
    closest_key = min(dictionary, key=lambda k: abs(dictionary[k] - target_value))
    return closest_key

if __name__ == '__main__':
    dict_speed = {
        10: 0, 11: 0, 12: 1, 13: 1, 14: 2, 15: 2, 16: 3, 17: 3, 18: 4, 19: 4, 
        20: 5, 21: 5, 22: 6, 23: 6, 24: 7, 25: 7, 26: 8, 27: 8, 28: 9, 29: 10, 
        30: 11, 31: 12, 32: 12, 33: 13, 34: 14, 35: 15, 36: 16, 37: 17, 38: 18, 
        39: 19, 40: 20, 41: 20, 42: 21, 43: 22, 44: 23, 45: 24
    }

    rospy.init_node('angle_reader')
    angle_pub = rospy.Publisher('current_steer', Int32, queue_size=10)
    rospy.Subscriber("/control/steer_angle", Float32Msg, steering_mpc, queue_size=1)
    rospy.Subscriber('/vehicle/mpc_path', mpc_path, update_mpc_trajectory, queue_size=1)
    rospy.Subscriber('/collision', Float32Msg, zed_collision, queue_size=1)
    rospy.Subscriber('/calculated_velocity', Vector3, _parse_gps_vel, queue_size=10)

    rate = rospy.Rate(10)
    desired_angle = 0
    prev_vel = 0
    brake_counter = 0
    
    while not rospy.is_shutdown():
        feedback_angle = read_angle()
        steer_output = int(math.degrees(steermpc) * 10.57)
        Write = client.write_registers(22, 2300, unit=UNIT)
        set_forward()
        
        # Find the desired key to achieve a velocity close to the target (e.g., 35 km/h)
        target_velocity = 35  # Example target velocity
        desired_acceleration_key = find_closest_key(dict_speed, target_velocity)
        
        if abs(steer_output) < 30:
            upvel = target_velocity
        else:
            if target_velocity > 27:
                upvel = 28
            else:
                upvel = target_velocity

        if coll == 2:
            desired_acceleration_key = 10  # Stopping the vehicle
            if brake_counter < 1:
                apply_brake()
                brake_counter += 1
            upvel = 0

        elif coll == 1:
            if target_velocity > 25:
                upvel = 26
            else:
                upvel = target_velocity
        
        else:
            brake_counter = 0
            remove_brake()

        # Use the desired acceleration key to set the vehicle's acceleration
        accelerate(desired_acceleration_key)
        prev_vel = upvel
        
        if end_point:
            accelerate(10)  # Stopping the vehicle at the endpoint

        set_steer(int(steer_output / 2.8))
        rate.sleep()

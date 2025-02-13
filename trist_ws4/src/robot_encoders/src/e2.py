#!/usr/bin/env python3

def encoder_callback():
    global left_encoder_ticks, right_encoder_ticks, last_left_ticks, last_right_ticks
    global robot_x, robot_y, robot_theta, last_time

    # Open serial port to read from Arduino
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        while not rospy.is_shutdown():
            try:
                # Read raw data from the serial port (binary data)
                line = ser.readline()  # This reads the data as raw bytes
                if line:
                    # Try to decode the byte data manually or print for debugging
                    try:
                        line_str = line.decode('utf-8').strip()  # Attempt to decode to UTF-8
                    except UnicodeDecodeError:
                        rospy.logwarn(f"Received non-UTF-8 data: {line}")
                        continue  # Skip invalid data

                    rospy.loginfo(f"Received data: {line_str}")

                    # Assuming Arduino sends left and right encoder counts as "L:left_ticks R:right_ticks"
                    data = line_str.split()
                    if len(data) == 2:
                        left_ticks = int(data[0].split(":")[1])
                        right_ticks = int(data[1].split(":")[1])

                        # Compute the change in encoder ticks
                        delta_left_ticks = left_ticks - last_left_ticks
                        delta_right_ticks = right_ticks - last_right_ticks

                        # Update last encoder ticks
                        last_left_ticks = left_ticks
                        last_right_ticks = right_ticks

                        # Compute linear and angular velocities based on encoder counts
                        left_distance = (delta_left_ticks / ENCODER_PPR) * (2 * math.pi * WHEEL_RADIUS)
                        right_distance = (delta_right_ticks / ENCODER_PPR) * (2 * math.pi * WHEEL_RADIUS)

                        # Compute robot's change in position
                        delta_distance = (left_distance + right_distance) / 2.0
                        delta_theta = (right_distance - left_distance) / BASE_WIDTH

                        # Update robot's position
                        current_time = rospy.Time.now()
                        dt = (current_time - last_time).to_sec() if last_time else 0.0

                        if dt > 0:
                            # Update robot's position in the world frame
                            robot_x += delta_distance * math.cos(robot_theta)
                            robot_y += delta_distance * math.sin(robot_theta)
                            robot_theta += delta_theta

                            # Publish Odometry message
                            odom = Odometry()
                            odom.header.stamp = current_time
                            odom.header.frame_id = "odom"
                            odom.child_frame_id = "base_link"

                            odom.pose.pose.position.x = robot_x
                            odom.pose.pose.position.y = robot_y
                            odom.pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, robot_theta))

                            odom.twist.twist.linear.x = delta_distance / dt
                            odom.twist.twist.angular.z = delta_theta / dt

                            odom_pub.publish(odom)

                            # Publish TF transform for the robot's position
                            br.sendTransform((robot_x, robot_y, 0),
                                             quaternion_from_euler(0, 0, robot_theta),
                                             current_time,
                                             "base_link",
                                             "odom")

                        # Update last time
                        last_time = current_time

            except Exception as e:
                rospy.logwarn(f"Error reading serial data: {e}")
                time.sleep(0.1)


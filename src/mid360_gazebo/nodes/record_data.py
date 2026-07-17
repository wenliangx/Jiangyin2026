#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64
import csv
import os


class DataRecorderNode:
    def __init__(self):
        rospy.init_node('data_recorder', anonymous=True)

        # Output file path
        self.output_dir = rospy.get_param('~output_dir', '/tmp')
        self.filename = rospy.get_param('~filename', 'pose_errors.csv')
        self.duration = rospy.get_param('~duration', 60.0)  # seconds to record

        self.full_path = os.path.join(self.output_dir, self.filename)
        self.start_time = None
        self.row_count = 0

        # Create CSV file with headers
        self.csv_file = open(self.full_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'time', 'dx', 'dy', 'dz', 'd_yaw',
            'horizontal_error', 'total_error'
        ])

        # Subscribe to error topics
        self.sub_dx = rospy.Subscriber('/pose_comparison/dx', Float64, self.callback)
        self.sub_dy = rospy.Subscriber('/pose_comparison/dy', Float64, self.callback)
        self.sub_dz = rospy.Subscriber('/pose_comparison/dz', Float64, self.callback)
        self.sub_d_yaw = rospy.Subscriber('/pose_comparison/d_yaw', Float64, self.callback)
        self.sub_h_error = rospy.Subscriber('/pose_comparison/horizontal_error', Float64, self.callback)
        self.sub_t_error = rospy.Subscriber('/pose_comparison/total_error', Float64, self.callback)

        self.current_data = {}
        self.rate = rospy.Rate(10.0)  # Record at 10Hz

    def callback(self, msg):
        topic_name = rospy.get_caller_id().split('/')[-2]
        self.current_data[topic_name] = msg.data

    def run(self):
        rospy.loginfo("Recording pose errors to: %s", self.full_path)
        rospy.loginfo("Duration: %.1f seconds", self.duration)

        while not rospy.is_shutdown():
            if self.start_time is None:
                self.start_time = rospy.get_time()
                rospy.loginfo("Started recording at t=%.2f", self.start_time)

            current_time = rospy.get_time() - self.start_time

            # Check duration
            if current_time >= self.duration:
                break

            # Record data when we have all fields
            if len(self.current_data) == 6:
                try:
                    self.writer.writerow([
                        f"{current_time:.3f}",
                        f"{self.current_data['dx']:.6f}",
                        f"{self.current_data['dy']:.6f}",
                        f"{self.current_data['dz']:.6f}",
                        f"{self.current_data['d_yaw']:.6f}",
                        f"{self.current_data['horizontal_error']:.6f}",
                        f"{self.current_data['total_error']:.6f}"
                    ])
                    self.csv_file.flush()
                    self.row_count += 1

                    # Print progress every 10 seconds
                    if int(current_time) % 10 == 0 and int(current_time) > 0:
                        rospy.loginfo("Recorded %d rows, time=%.1fs", self.row_count, current_time)

                except Exception as e:
                    pass

                # Clear data for next cycle
                self.current_data.clear()

            self.rate.sleep()

        # Cleanup
        self.csv_file.close()
        rospy.loginfo("Recording complete. Total rows: %d", self.row_count)
        rospy.loginfo("Data saved to: %s", self.full_path)


if __name__ == '__main__':
    try:
        node = DataRecorderNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3

import rospy
import rosnode
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import UInt32, Duration

import os
import subprocess


class DataRecorder():
    def __init__(self):
        self.start_recording_service = rospy.Service('/data_recording/start_recording', Trigger, self.start_recording)
        self.stop_recording_service = rospy.Service('/data_recording/stop_recording', Trigger, self.stop_recording)
        self.stop_recording_service = rospy.Service('/data_recording/toggle_recording', Trigger, self.toggle_recording)

        # Current bag size in bytes
        self.size_pub = rospy.Publisher('/data_recording/bag_size', UInt32, queue_size=1)

        # Current bag duration
        self.duration_pub = rospy.Publisher('/data_recording/duration', Duration, queue_size=10)

        self.process = None
        self.recording = False
        self.command = []

        self.get_parameters()

        rospy.loginfo('Data recorder ready')
    
    def get_parameters(self):
        self.output_directory = rospy.get_param('/data_recording/output_directory', "~/Documents/rosbag/test/")
        self.output_directory = os.path.expanduser(self.output_directory) # make ~ usable in the path
        self.bag_name = rospy.get_param('/data_recording/bag_name', "my_bag.bag")

        # Ensure that bag_name ends in .bag
        if self.bag_name[-4:] != ".bag":
            self.bag_name += ".bag"
        self.bag_file = os.path.join(self.output_directory, self.bag_name)

        self.topics = rospy.get_param('/data_recording/topics', ["/rosout"])
        if not self.topics:
            rospy.logerr('No Topics Specified.')


    def toggle_recording(self, req):
        if self.recording:
            return self.stop_recording(req)
        else:
            return self.start_recording(req)

    def start_recording(self, req):
        if self.recording:
            rospy.logerr('Already Recording')
            return TriggerResponse(False, 'Already Recording')
        
        self.get_parameters()
        
        # Check if directory is real
        if not os.path.isdir(self.output_directory):
            rospy.logerr('No such directory: ' + self.output_directory)
            return TriggerResponse(False, "Directory doesn't exist")
        
        self.command = ['rosbag', 'record', '-e'] + self.topics + ['__name:=data_recording_myrecorder'] + ['-O', self.bag_file]

        self.process = subprocess.Popen(self.command, stdout=subprocess.PIPE)
        self.recording = True
        self.start_time = rospy.get_rostime()

        rospy.loginfo('Started recorder, PID %s' % self.process.pid)
        return TriggerResponse(True, 'Started recorder, PID %s' % self.process.pid)

    def stop_recording(self, req):
        if not self.recording:
            rospy.logerr('Not Recording')
            return TriggerResponse(False, 'Not Recording')

        rosnode.kill_nodes(['/data_recording_myrecorder'])

        self.process = None
        self.recording = False

        rospy.loginfo('Stopped Recording')
        return TriggerResponse(True, 'Stopped Recording')
    
    def spin(self):
        rate = rospy.Rate(2)

        while not rospy.is_shutdown():
            
            if self.recording:

                # Check if temporary file has been created
                if os.path.exists(self.bag_file + ".active"):
                    # Publish the current rosbag size
                    bag_size = os.path.getsize(self.bag_file + ".active")
                    size_msg =UInt32()
                    size_msg.data = bag_size
                    self.size_pub.publish(size_msg)
                    # rospy.loginfo("Bag file size: " + humanize.naturalsize(bag_size))
    
                # Publish rosbag duration
                elapsed_time = rospy.get_rostime() - self.start_time
                duration_msg = Duration()
                duration_msg.data = elapsed_time
                self.duration_pub.publish(duration_msg)
                

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('data_recording')
    datarecorder = DataRecorder()
    datarecorder.spin()
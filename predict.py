import torch
import torch.nn as nn
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from model import PosePredictionModel
import transforms3d.euler as tf_euler
import math

input_size = 3
hidden_size = 128
output_size = 2

class DataSubscribe():
    def __init__(self, model) -> None:
        self.last_pose_sub = rospy.Subscriber('/last_pose', PoseStamped, self.last_pose_callback)
        self.odom_sub = rospy.Subscriber('/odom_fusion', Odometry, self.odom_fusion_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.predict_pub = rospy.Publisher('predict_pose', PoseStamped, queue_size=1)
        self.model = model
        self.last_pos_x = 0.0
        self.last_pos_y = 0.0
        self.last_ori_z = 0.0
        self.last_ori_w = 0.0
        self.odom_v = 0.0
        self.odom_w = 0.0
        self.odom_fusion_v = 0.0
        self.odom_fusion_w = 0.0

    def last_pose_callback(self, last_msg:PoseStamped):
        last_position = last_msg.pose.position
        last_orientation = last_msg.pose.orientation
        self.last_pos_x = last_position.x
        self.last_pos_y = last_position.y
        self.last_ori_z = last_orientation.z
        self.last_ori_w = last_orientation.w

        qua = [self.last_ori_w, 0.0, 0.0, self.last_ori_z]
        last_yaw = tf_euler.quat2euler(qua, axes='sxyz')[2]
        input_data = torch.tensor([self.odom_fusion_v, self.odom_fusion_w, last_yaw], dtype=torch.float32)
        with torch.no_grad():
            predicted_output = self.model(input_data)

        next_yaw = last_yaw + 0.09839830205101541 * self.odom_fusion_w
        # next_yaw = last_yaw + 0.10927975861673422 * self.odom_w + -0.0001377448141874914
        quaternion = tf_euler.euler2quat(0.0, 0.0, next_yaw)
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.stamp = rospy.Time.now()
        pose_stamped_msg.header.frame_id = 'map'
        if math.fabs(self.odom_v) < pow(10, -3) and math.fabs(self.odom_w) < pow(10, -3):
            pose_stamped_msg.pose.position.x = self.last_pos_x
            pose_stamped_msg.pose.position.y = self.last_pos_y
            pose_stamped_msg.pose.position.z = 0.0
            pose_stamped_msg.pose.orientation.x = 0.0
            pose_stamped_msg.pose.orientation.y = 0.0
            pose_stamped_msg.pose.orientation.z = self.last_ori_z
            pose_stamped_msg.pose.orientation.w = self.last_ori_w
        elif math.fabs(self.odom_v) < pow(10, -3):
            pose_stamped_msg.pose.position.x = self.last_pos_x
            pose_stamped_msg.pose.position.y = self.last_pos_y
            pose_stamped_msg.pose.position.z = 0.0
            pose_stamped_msg.pose.orientation.x = 0.0
            pose_stamped_msg.pose.orientation.y = 0.0
            pose_stamped_msg.pose.orientation.z = quaternion[3]
            pose_stamped_msg.pose.orientation.w = quaternion[0]
        elif math.fabs(self.odom_w) < pow(10, -3):
            pose_stamped_msg.pose.position.x = self.last_pos_x + predicted_output[0].item()
            pose_stamped_msg.pose.position.y = self.last_pos_y + predicted_output[1].item()
            pose_stamped_msg.pose.position.z = 0.0
            pose_stamped_msg.pose.orientation.x = 0.0
            pose_stamped_msg.pose.orientation.y = 0.0
            pose_stamped_msg.pose.orientation.z = self.last_ori_z
            pose_stamped_msg.pose.orientation.w = self.last_ori_w
        else:
            pose_stamped_msg.pose.position.x = self.last_pos_x + predicted_output[0].item()
            pose_stamped_msg.pose.position.y = self.last_pos_y + predicted_output[1].item()
            pose_stamped_msg.pose.position.z = 0.0
            pose_stamped_msg.pose.orientation.x = 0.0
            pose_stamped_msg.pose.orientation.y = 0.0
            pose_stamped_msg.pose.orientation.z = quaternion[3]
            pose_stamped_msg.pose.orientation.w = quaternion[0]
        self.predict_pub.publish(pose_stamped_msg)

    def odom_callback(self, odom_msg:Odometry):
        self.odom_v = odom_msg.twist.twist.linear.x
        self.odom_w = odom_msg.twist.twist.angular.z

    def odom_fusion_callback(self, odom_msg:Odometry):
        self.odom_fusion_v = odom_msg.twist.twist.linear.x
        self.odom_fusion_w = odom_msg.twist.twist.angular.z

        # pose_stamped_msg = PoseStamped()
        # pose_stamped_msg.header.stamp = rospy.Time.now()
        # pose_stamped_msg.header.frame_id = 'map'
        # pose_stamped_msg.pose.position.x = odom_msg.pose.pose.position.x
        # pose_stamped_msg.pose.position.y = odom_msg.pose.pose.position.y
        # pose_stamped_msg.pose.position.z = 0.0
        # pose_stamped_msg.pose.orientation.x = 0.0
        # pose_stamped_msg.pose.orientation.y = 0.0
        # pose_stamped_msg.pose.orientation.z = odom_msg.pose.pose.orientation.z
        # pose_stamped_msg.pose.orientation.w = odom_msg.pose.pose.orientation.w
        # self.predict_pub.publish(pose_stamped_msg)

if __name__ == "__main__":
    rospy.init_node('predict')
    
    model = PosePredictionModel(input_size, hidden_size, output_size)
    model.load_state_dict(torch.load('/home/drl03/catkin_songcan/src/slam_gmapping/pose_prediction_model.pth'))
    model.eval()  

    data_sub = DataSubscribe(model)
    rospy.spin()
    
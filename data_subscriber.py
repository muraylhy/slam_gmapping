import rospy
import csv
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

data_path = '/home/drl03/catkin_songcan/src/slam_gmapping/amcl_and_odom_data.csv'

def amcl_pose_callback(amcl_msg:PoseWithCovarianceStamped):
    global amcl_pos_x, amcl_pos_y, amcl_ori_z, amcl_ori_w
    amcl_position = amcl_msg.pose.pose.position
    amcl_orientation = amcl_msg.pose.pose.orientation
    amcl_pos_x, amcl_pos_y = amcl_position.x, amcl_position.y
    amcl_ori_z, amcl_ori_w = amcl_orientation.z, amcl_orientation.w

def odom_callback(odom_msg:Odometry):
    global odom_v, odom_w, odom_x, odom_y, odom_z, odom_ori_w
    odom_v = odom_msg.twist.twist.linear.x
    odom_w = odom_msg.twist.twist.angular.z
    odom_x = odom_msg.pose.pose.position.x
    odom_y = odom_msg.pose.pose.position.y
    odom_z = odom_msg.pose.pose.orientation.z
    odom_ori_w = odom_msg.pose.pose.orientation.w

    # with open(data_path, mode='a') as csv_file:
    #     fieldnames = ['odom_v', 'odom_w', 'odom_x','odom_y', 'odom_z', 'odom_ori_w']
    #     writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

    #     if csv_file.tell() == 0:
    #         writer.writeheader()

    #     writer.writerow({'odom_v': odom_v, 'odom_w': odom_w, 'odom_x': odom_x, 'odom_y': odom_y, 'odom_z': odom_z, 'odom_ori_w': odom_ori_w})
    
def odom_fusion_callback(odom_msg:Odometry):
    global odom_fusion_v, odom_fusion_w, odom_fusion_x, odom_fusion_y, odom_fusion_z, odom_fusion_ori_w
    odom_fusion_v = odom_msg.twist.twist.linear.x
    odom_fusion_w = odom_msg.twist.twist.angular.z
    odom_fusion_x = odom_msg.pose.pose.position.x
    odom_fusion_y = odom_msg.pose.pose.position.y
    odom_fusion_z = odom_msg.pose.pose.orientation.z
    odom_fusion_ori_w = odom_msg.pose.pose.orientation.w

    # with open(data_path, mode='a') as csv_file:
    #     fieldnames = ['odom_fusion_v', 'odom_fusion_w', 'odom_fusion_x','odom_fusion_y', 'odom_fusion_z', 'odom_fusion_ori_w']
    #     writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

    #     if csv_file.tell() == 0:
    #         writer.writeheader()

    #     writer.writerow({'odom_fusion_v': odom_fusion_v, 'odom_fusion_w': odom_fusion_w, 'odom_fusion_x': odom_fusion_x, 'odom_fusion_y': odom_fusion_y, 'odom_fusion_z': odom_fusion_z, 'odom_fusion_ori_w': odom_fusion_ori_w})


def collect_data(timer_event):
    timestamp = rospy.Time.now()
    with open(data_path, mode='a') as csv_file:
        fieldnames = ['timestamp', 'amcl_pos_x', 'amcl_pos_y', 'amcl_ori_z', 'amcl_ori_w', 
                      'odom_v', 'odom_w', 'odom_x', 'odom_y', 'odom_z', 'odom_ori_w',
                      'odom_fusion_v', 'odom_fusion_w', 'odom_fusion_x','odom_fusion_y', 'odom_fusion_z', 'odom_fusion_ori_w']
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

        if csv_file.tell() == 0:
            writer.writeheader()

        writer.writerow({'timestamp': timestamp, 'amcl_pos_x': amcl_pos_x, 'amcl_pos_y': amcl_pos_y, 'amcl_ori_z': amcl_ori_z, 'amcl_ori_w': amcl_ori_w, 
                         'odom_v': odom_v, 'odom_w': odom_w, 'odom_x': odom_x, 'odom_y': odom_y, 'odom_z': odom_z, 'odom_ori_w': odom_ori_w, 
                         'odom_fusion_v': odom_fusion_v, 'odom_fusion_w': odom_fusion_w, 'odom_fusion_x': odom_fusion_x, 'odom_fusion_y': odom_fusion_y, 'odom_fusion_z': odom_fusion_z, 'odom_fusion_ori_w': odom_fusion_ori_w})


if __name__ == '__main__':
    rospy.init_node('data_subscriber')
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback)
    rospy.Subscriber('/odom_fusion', Odometry, odom_fusion_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Timer(rospy.Duration(0.1), collect_data)
    rospy.spin()
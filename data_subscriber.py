import rospy
import csv
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

data_path = '/home/drl03/catkin_songcan/src/slam_gmapping/odom_data_01.csv'

# def temp_pose_callback(temp_msg:PoseWithCovarianceStamped):
#     global temp_pos_x, temp_pos_y, temp_ori_z, temp_ori_w, temp_odom_v, temp_odom_w, temp_imu_w
#     temp_pos_x, temp_pos_y = temp_msg.pose.pose.position.x, temp_msg.pose.pose.position.y
#     temp_ori_z, temp_ori_w= temp_msg.pose.pose.orientation.z, temp_msg.pose.pose.orientation.w
#     temp_odom_v, temp_odom_w, temp_imu_w = temp_msg.pose.pose.orientation.x, temp_msg.pose.pose.orientation.y, temp_msg.pose.pose.position.z 
#     timestamp = rospy.Time.now()
#     with open(data_path, mode='a') as csv_file:
#         fieldnames = ['timestamp', 'temp_pos_x', 'temp_pos_y', 'temp_ori_z', 'temp_ori_w', 'temp_odom_v', 'temp_odom_w', 'temp_imu_w']
#         writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

#         if csv_file.tell() == 0:
#             writer.writeheader()

#         writer.writerow({'timestamp': timestamp, 'temp_pos_x': temp_pos_x, 'temp_pos_y': temp_pos_y, 'temp_ori_z': temp_ori_z, 'temp_ori_w': temp_ori_w, 'temp_odom_v': temp_odom_v, 'temp_odom_w': temp_odom_w, 'temp_imu_w': temp_imu_w})

def amcl_pose_callback(amcl_msg:PoseWithCovarianceStamped):
    global amcl_pos_x, amcl_pos_y, amcl_pos_z, amcl_ori_z, amcl_ori_w
    amcl_position = amcl_msg.pose.pose.position
    amcl_orientation = amcl_msg.pose.pose.orientation
    amcl_pos_x, amcl_pos_y, amcl_pos_z = amcl_position.x, amcl_position.y, amcl_position.z
    amcl_ori_z, amcl_ori_w = amcl_orientation.z, amcl_orientation.w

def odom_callback(odom_msg:Odometry):
    global odom_v, odom_w, odom_x, odom_y, odom_z, odom_ori_w, time
    odom_v = odom_msg.twist.twist.linear.x
    odom_w = odom_msg.twist.twist.angular.z
    odom_x = odom_msg.pose.pose.position.x
    odom_y = odom_msg.pose.pose.position.y
    odom_z = odom_msg.pose.pose.orientation.z
    odom_ori_w = odom_msg.pose.pose.orientation.w

    time = odom_msg.header.stamp
    timestamp = rospy.Time.now()
    print(timestamp.to_sec(), "              ", time.to_sec())
    # with open(data_path, mode='a') as csv_file:
    #     fieldnames = ['time', 'odom_v', 'odom_w', 'odom_x','odom_y', 'odom_z', 'odom_ori_w']
    #     writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

    #     if csv_file.tell() == 0:
    #         writer.writeheader()

    #     writer.writerow({'time': time, 'odom_v': odom_v, 'odom_w': odom_w, 'odom_x': odom_x, 'odom_y': odom_y, 'odom_z': odom_z, 'odom_ori_w': odom_ori_w})
    
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
        # fieldnames = ['timestamp', 'temp_pos_x', 'temp_pos_y', 'temp_ori_z', 'temp_ori_w', 'temp_v', 'temp_w']
                    #   'amcl_pos_x', 'amcl_pos_y', 'amcl_pos_z', 'amcl_ori_z', 'amcl_ori_w',
        fieldnames = ['timestamp', 'time', 'odom_v', 'odom_w', 'odom_x', 'odom_y', 'odom_z', 'odom_ori_w']
                    # 'odom_fusion_v', 'odom_fusion_w', 'odom_fusion_x','odom_fusion_y', 'odom_fusion_z', 'odom_fusion_ori_w']
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

        if csv_file.tell() == 0:
            writer.writeheader()

        # writer.writerow({'timestamp': timestamp, 'temp_pos_x': temp_pos_x, 'temp_pos_y': temp_pos_y, 'temp_ori_z': temp_ori_z, 'temp_ori_w': temp_ori_w, 'temp_v': temp_v, 'temp_w': temp_w})
                        #  'amcl_pos_x': amcl_pos_x, 'amcl_pos_y': amcl_pos_y, 'amcl_pos_z': amcl_pos_z, 'amcl_ori_z': amcl_ori_z, 'amcl_ori_w': amcl_ori_w,
        writer.writerow({'timestamp': timestamp, 'time': time, 'odom_v': odom_v, 'odom_w': odom_w, 'odom_x': odom_x, 'odom_y': odom_y, 'odom_z': odom_z, 'odom_ori_w': odom_ori_w})
                        # 'odom_fusion_v': odom_fusion_v, 'odom_fusion_w': odom_fusion_w, 'odom_fusion_x': odom_fusion_x, 'odom_fusion_y': odom_fusion_y, 'odom_fusion_z': odom_fusion_z, 'odom_fusion_ori_w': odom_fusion_ori_w})
    print(timestamp.to_sec(), "                   ", time.to_sec())

if __name__ == '__main__':
    rospy.init_node('data_subscriber')
    # rospy.Subscriber('/temp_pose_vw', PoseWithCovarianceStamped, temp_pose_callback)
    # rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback)
    # rospy.Subscriber('/odom_fusion', Odometry, odom_fusion_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback, queue_size=10)
    # rospy.Timer(rospy.Duration(0.1), collect_data)
    rospy.spin()
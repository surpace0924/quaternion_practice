#!/usr/bin/env python3

import numpy as np

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped


class Main:
    def __init__(self):
        super().__init__()
        rospy.init_node('tip_path_plan_action_server', anonymous=True)

        print(self.generate_orthogonal_vector(np.array([1, 0, 0]), 90))

        # カウンタの設定
        self.count = 0
        
        # パブリッシャの設定
        self.pub_origin_pose = rospy.Publisher('origin_pose', PoseStamped, queue_size=10)
        self.pub_ref_pose = rospy.Publisher('ref_pose', PoseStamped, queue_size=10)

        # タイマーの設定
        rospy.Timer(rospy.Duration(0.1), self.loop_cb)


    # 一定周期で呼び出されるコールバック関数
    def loop_cb(self, event):
        origin_pose_msg = self.to_pose_msg([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 'map')

        vec1 = np.array([1, 1, 0])
        vec2 = np.array([0, 0, -1])

        ref_pose_msg = self.to_pose_msg([0.0, 0.0, 0.0], self.quaternion_from_x_axis_and_angle(vec1, 0), 'map')

        self.pub_origin_pose.publish(origin_pose_msg)
        self.pub_ref_pose.publish(ref_pose_msg)
        self.count += 1

    def easy(x, y, z):

        Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)


    # 位置姿勢をPoseStamped型に変換する関数
    def to_pose_msg(self, p, o, frame_id):
        pose_msg = PoseStamped()
        header_msg = Header(frame_id=frame_id, stamp=rospy.Time.now())
        pose_msg.header = header_msg
        pose_msg.pose.position = Point(x=p[0], y=p[1], z=p[2])
        pose_msg.pose.orientation = Quaternion(x=o[0], y=o[1], z=o[2], w=o[3])
        return pose_msg
    

    def quaternion_from_x_axis_and_angle(self, x_axis_direction, angle):
        # Step 1: Normalize the x-axis direction vector
        x_axis_direction = x_axis_direction / np.linalg.norm(x_axis_direction)
        
        # Step 2: Convert angle to radians
        theta = np.deg2rad(angle)
        
        # Step 3: Compute half angle
        half_angle = theta / 2.0
        
        # Step 4: Compute quaternion components
        w = np.cos(half_angle)
        x = x_axis_direction[0] * np.sin(half_angle)
        y = x_axis_direction[1] * np.sin(half_angle)
        z = x_axis_direction[2] * np.sin(half_angle)
        
        # Construct quaternion
        quaternion = Quaternion(w=w, x=x, y=y, z=z)
        
        return np.array([x, y, z, w])


    def generate_orthogonal_vector(self, vector, angle):
        # Step 1: Normalize the input vector
        vector = vector / np.linalg.norm(vector)
        
        # Step 2: Convert angle to radians
        theta = np.deg2rad(angle)
        
        # Step 3: Generate a rotation matrix
        # First, calculate a random vector that is not parallel to the input vector
        random_vector = np.random.rand(3)
        random_vector -= random_vector.dot(vector) * vector
        random_vector /= np.linalg.norm(random_vector)
        
        # Calculate the cross product to get the axis of rotation
        axis = np.cross(vector, random_vector)
        axis /= np.linalg.norm(axis)
        
        # Use Rodrigues' rotation formula to generate the rotated vector
        rotated_vector = (vector * np.cos(theta)) + (np.cross(axis, vector) * np.sin(theta)) + (axis * np.dot(axis, vector)) * (1 - np.cos(theta))
        
        # Normalize the rotated vector
        rotated_vector /= np.linalg.norm(rotated_vector)

        return rotated_vector


    # x, y軸の方向ベクトルからクォータニオンを計算する関数
    def quaternion_from_x_and_y_axes(self, x_axis_direction, y_axis_direction):
        # 正規化
        x_axis_direction = x_axis_direction / np.linalg.norm(x_axis_direction)
        y_axis_direction = y_axis_direction / np.linalg.norm(y_axis_direction)
        
        # z軸方向を計算
        z_axis_direction = np.cross(x_axis_direction, y_axis_direction)
        z_axis_direction /= np.linalg.norm(z_axis_direction)
        
        # x, y, z軸方向から回転行列を計算
        rotation_matrix = np.column_stack((x_axis_direction, y_axis_direction, z_axis_direction))
        
        # 回転行列からクォータニオンを計算
        trace = np.trace(rotation_matrix)
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) * s
            y = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) * s
            z = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) * s
        elif rotation_matrix[0, 0] > rotation_matrix[1, 1] and rotation_matrix[0, 0] > rotation_matrix[2, 2]:
            s = 2.0 * np.sqrt(1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2])
            w = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
            x = 0.25 * s
            y = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
            z = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
        elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
            s = 2.0 * np.sqrt(1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2])
            w = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
            x = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
            y = 0.25 * s
            z = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1])
            w = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
            x = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
            y = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
            z = 0.25 * s

        return np.array([x, y, z, w])




if __name__ == '__main__':
    try:
        Main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

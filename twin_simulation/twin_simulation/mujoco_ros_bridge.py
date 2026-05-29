import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import mujoco
import mujoco.viewer
import time
import os
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry

import math
import numpy as np
from sensor_msgs.msg import LaserScan

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class MuJoCoROSBridge(Node):
    def __init__(self):
        super().__init__('mujoco_ros_bridge')

        # Load the MJCF XML file
        package_share_dir = get_package_share_directory('twin_simulation')
        mjcf_xml = os.path.join(package_share_dir, 'urdf', 'twin_sim_car.xml')

        self.model = mujoco.MjModel.from_xml_path(mjcf_xml)
        self.data = mujoco.MjData(self.model)


        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.target_velocity= 0.0
        self.target_steering= 0.0

        self.get_logger().info('Starting MuJoCo Digital Twin Simulator...')
        self.run_simulation()

    def cmd_vel_callback(self, msg):
        """Map ROS 2 Twist messages to physical actuators"""
        self.target_velocity = msg.linear.x
        self.target_steering = msg.angular.z

    def run_simulation(self):
        fl_steer_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'fl_steer_joint')
        fr_steer_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'fr_steer_joint')
        rl_wheel_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'rl_wheel_joint')
        rr_wheel_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'rr_wheel_joint')

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running() and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0)

                # Applying steering angle
                self.data.qfrc_applied[self.model.jnt_dofadr[fl_steer_id]] = -100.0 * (self.data.qpos[self.model.jnt_qposadr[fl_steer_id]] - self.target_steering)
                self.data.qfrc_applied[self.model.jnt_dofadr[fr_steer_id]] = -100.0 * (self.data.qpos[self.model.jnt_qposadr[fr_steer_id]] - self.target_steering)


                # Applying drive torque
                rl_vel = self.data.qvel[self.model.jnt_dofadr[rl_wheel_id]]
                rr_vel = self.data.qvel[self.model.jnt_dofadr[rr_wheel_id]]

                # The distance from the center of the chassis to the wheel is 0.45 meters.
                # If steering left (positive), the left wheel slows down and the right speeds up.
                v_left = self.target_velocity - (self.target_steering * 0.45)
                v_right = self.target_velocity + (self.target_steering * 0.45)

                # Convert linear wheel speed to angular RPM
                target_angular_velocity_l = v_left / 0.23
                target_angular_velocity_r = v_right / 0.23

                # Apply independent torque to each wheel
                self.data.qfrc_applied[self.model.jnt_dofadr[rl_wheel_id]] = 20.0 * (target_angular_velocity_l - rl_vel)
                self.data.qfrc_applied[self.model.jnt_dofadr[rr_wheel_id]] = 20.0 * (target_angular_velocity_r - rr_vel)

                #Publish Odometry (Ground Truth)
                odom = Odometry()
                odom.header.stamp = self.get_clock().now().to_msg()
                odom.header.frame_id = 'world'

                #MuJoCo freejoin qpos stores: [x, y, z, qw, qx, qy, qz]
                odom.pose.pose.position.x = float(self.data.qpos[0])
                odom.pose.pose.position.y = float(self.data.qpos[1])
                odom.pose.pose.position.z = float(self.data.qpos[2])

                odom.pose.pose.orientation.w = float(self.data.qpos[3])
                odom.pose.pose.orientation.x = float(self.data.qpos[4])
                odom.pose.pose.orientation.y = float(self.data.qpos[5])
                odom.pose.pose.orientation.z = float(self.data.qpos[6])

                self.odom_pub.publish(odom)

                #TF Brodcaster for RVIZ

                #Transforming from World to base link first
                t_base = TransformStamped()
                t_base.header.stamp = self.get_clock().now().to_msg()
                t_base.header.frame_id = 'world'
                t_base.child_frame_id = 'base_link'
                t_base.transform.translation.x = float(self.data.qpos[0])
                t_base.transform.translation.y = float(self.data.qpos[1])
                t_base.transform.translation.z = float(self.data.qpos[2])
                t_base.transform.rotation.w = float(self.data.qpos[3])
                t_base.transform.rotation.x = float(self.data.qpos[4])
                t_base.transform.rotation.y = float(self.data.qpos[5])
                t_base.transform.rotation.z = float(self.data.qpos[6])
                self.tf_broadcaster.sendTransform(t_base)

                #Transforming from base_link to lidar Mount
                t_lidar = TransformStamped()
                t_lidar.header.stamp = self.get_clock().now().to_msg()
                t_lidar.header.frame_id = 'base_link'
                t_lidar.child_frame_id = 'lidar_mount'
                t_lidar.transform.translation.x = 0.0
                t_lidar.transform.translation.y = 0.0
                t_lidar.transform.translation.z = 0.1
                t_lidar.transform.rotation.w = 1.0
                self.tf_broadcaster.sendTransform(t_lidar)
                


                # Custom made 360-degree LiDar Scanner 
                num_of_rays = 36 # 10 degrees per ray
                max_range = 15.0 # Lasers reach 15 meters
                ranges = []

                # Getting the physical location and rotation of the lidar mount
                lidar_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'lidar_mount')
                site_pos = self.data.site_xpos[lidar_site_id]

                # Convert the 1D rotation array into a 3x3 rotation matrix
                site_mat = self.data.site_xmat[lidar_site_id].reshape(3,3)

                for i in range(num_of_rays):
                    angle = i * (2 * math.pi / num_of_rays)
                    local_dir = np.array([math.cos(angle), math.sin(angle), 0.0])

                    # Multiply by the car's rotation matrix so the lasers spin WITH the car
                    global_dir = site_mat.dot(local_dir)

                    # Firing the ray
                    geomid = np.array([-1], dtype=np.int32)
                    dist = mujoco.mj_ray(self.model, self.data, site_pos, global_dir, None, 1, -1, geomid)

                    if dist < 0:
                        dist = max_range
                    
                    ranges.append(float(dist))

                # Publish to ROS
                scan = LaserScan()
                scan.header.stamp = self.get_clock().now().to_msg()
                scan.header.frame_id = 'lidar_mount'
                scan.angle_min = 0.0
                scan.angle_max = 2*math.pi
                scan.angle_increment = (2*math.pi)/num_of_rays
                scan.range_min = 0.1
                scan.range_max = max_range
                scan.ranges = ranges

                self.scan_pub.publish(scan)

                # Step the physics engine
                mujoco.mj_step(self.model, self.data)

                # Updating the visualizer
                viewer.sync()
                time.sleep(0.005)

def main(args=None):
    rclpy.init(args=args)
    bridge = None
    try:
        bridge = MuJoCoROSBridge()
    except KeyboardInterrupt:
        pass
    finally:
        if bridge is not None:
            if hasattr(bridge, 'viewer') and bridge.viewer is not None:
                bridge.viewer.close()
            bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
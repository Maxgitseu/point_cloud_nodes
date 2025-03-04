import rclpy                                        # ROS2接口
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import os
import numpy as np
from std_msgs.msg import Header

class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('point_cloud_publisher')
        self.publisher = self.create_publisher(PointCloud2, 'point_cloud', 10)
        self.pcd_file = '/home/maxwell/data/1726729290081000000.pcd'
        self.timer = self.create_timer(0.1, self.timer_callback)
        
    def timer_callback(self):
        """定时读取PCD文件并发布点云"""
        try:
            # 检查文件是否存在
            if not os.path.exists(self.pcd_file):
                self.get_logger().error(f"PCD文件不存在: {self.pcd_file}")
                return
            
            # 读取PCD文件
            pcd = o3d.io.read_point_cloud(self.pcd_file)
            points = np.asarray(pcd.points)
            
            # 检查数据有效性
            if len(points) == 0:
                self.get_logger().warn("PCD文件无有效点云数据")
                return
            
            # 转换为ROS2消息
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'base_link'  # 设置坐标系
            
            # 生成PointCloud2消息
            cloud_msg = pc2.create_cloud_xyz32(header, points)
            self.publisher.publish(cloud_msg)
            self.get_logger().info(f"成功发布点云: {len(points)}个点")
            
        except Exception as e:
            self.get_logger().error(f"处理PCD文件失败: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
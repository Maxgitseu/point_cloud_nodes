import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# import open3d as o3d
# import threading

class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('point_cloud_subscriber')

        # 订阅者配置
        self.subscription = self.create_subscription(
            PointCloud2,
            'point_cloud',  # 与发布者的话题名一致
            self.callback,
            10
        )
        
        # 初始化Matplotlib
        plt.ion()  # 启用交互模式
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('ROS2 PointCloud Viewer')
        print("开始绘制")
        self.scatter = None

    def callback(self, msg):
        try:
            # 生成器转为结构化数组
            points_gen = pc2.read_points(
                msg, 
                field_names=("x", "y", "z"), 
                skip_nans=True
            )
            
            # 明确提取XYZ字段并转换为二维数组
            x = np.array([point[0] for point in points_gen])
            y = np.array([point[1] for point in points_gen])
            z = np.array([point[2] for point in points_gen])
            points = np.column_stack((x, y, z))  # 确保形状为 (N,3)
            print("点云形状：", points.shape)

            if len(points) == 0:
                self.get_logger().warn("收到空点云")
                return
            
            # 更新显示
            if self.scatter is not None:
                self.scatter.remove()
            self.scatter = self.ax.scatter(
                points[:,0], 
                points[:,1], 
                points[:,2],
                c=points[:,2],  # Z值作为颜色
                cmap='viridis',
                s=1
            )
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            self.get_logger().info(f"显示点云: {len(points)}点")

        except Exception as e:
            self.get_logger().error(f"处理点云失败: {str(e)}")
        
def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        plt.close()
        node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


    """# Open3D可视化初始化
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name='PointCloud Viewer')
        self.pcd = o3d.geometry.PointCloud()
        self.vis.add_geometry(self.pcd)
        
        # 使用ROS2定时器更新窗口
        self.timer = self.create_timer(0.033, self.visualization_update) #30Hz

        # 线程锁与退出标志
        self.lock = threading.Lock()
        self.exit_flag = False
        
        # 启动可视化线程
        threading.Thread(target=self.visualization_thread).start()

    def callback(self, msg):
        # ROS2回调：解析点云并更新显示数据
        try:
            # 从PointCloud2消息中提取点坐标
            points = np.array(list(pc2.read_points(
                msg, 
                field_names=("x", "y", "z"), 
                skip_nans=True
            )))
            
            if len(points) == 0:
                self.get_logger().warn("收到空点云")
                return
            
            # 更新Open3D点云对象
            self.pcd.points = o3d.utility.Vector3dVector(points)
            self.pcd.paint_uniform_color([0.5, 0.5, 0.5])
            if points.shape[0] > 0:
                z_min, z_max = np.min(points[:,2]), np.max(points[:,2])
                colors = (points[:,2] - z_min) / (z_max - z_min)
                colors = np.vstack([colors, colors, colors]).T
                self.pcd.colors = o3d.utility.Vector3dVector(colors)
            self.get_logger().info(f"已接收点云: {len(points)}点")
        except Exception as e:
            self.get_logger().error(f"解析点云失败: {str(e)}")

            # 更新Open3D点云对象
            with self.lock:
                self.pcd.points = o3d.utility.Vector3dVector(points)
                # 可选：计算点云颜色（示例：高度着色）
                self.pcd.paint_uniform_color([0.5, 0.5, 0.5])  # 重置颜色
                if points.shape[0] > 0:
                    z_min, z_max = np.min(points[:,2]), np.max(points[:,2])
                    colors = (points[:,2] - z_min) / (z_max - z_min)
                    colors = np.vstack([colors, colors, colors]).T  # 伪彩色映射
                    self.pcd.colors = o3d.utility.Vector3dVector(colors)
            
            self.get_logger().info(f"已接收点云: {len(points)}点")
            
        except Exception as e:
            self.get_logger().error(f"解析点云失败: {str(e)}")

    def visualization_thread(self):
            # 独立线程运行Open3D可视化
            while not self.exit_flag:
            with self.lock:
                self.vis.update_geometry(self.pcd)
                self.vis.poll_events()
                self.vis.update_renderer()
        self.vis.destroy_window()

    def visualization_update(self):
        # 在主线程中更新窗口
        self.vis.update_geometry(self.pcd)
        self.vis.poll_events()
        self.vis.update_renderer()

    def shutdown(self):
        # 安全关闭资源
        self.exit_flag = True
        self.vis.destroy_window()
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()"""
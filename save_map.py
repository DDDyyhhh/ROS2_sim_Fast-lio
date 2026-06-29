import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import sys

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver_node')
        self.get_logger().info("正在疯狂监听 /Laser_map 话题...")
        # 订阅全局点云地图
        self.sub = self.create_subscription(PointCloud2, '/Laser_map', self.callback, 1)

    def callback(self, msg):
        self.get_logger().info("✅ 截获地图数据！正在写入硬盘，请稍候...")
        # 提取点云坐标，过滤掉 NaN 异常点
        pts = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        
        # 强制写入绝对路径
        save_path = "/home/orangepi/ros2_ws/my_3d_map.pcd"
        
        with open(save_path, "w") as f:
            # 严格按照 PCD v0.7 格式标准写入文件头
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\n")
            f.write("FIELDS x y z\n")
            f.write("SIZE 4 4 4\n")
            f.write("TYPE F F F\n")
            f.write("COUNT 1 1 1\n")
            f.write(f"WIDTH {len(pts)}\nHEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {len(pts)}\n")
            f.write("DATA ascii\n")
            
            # 写入每一个特征点的 x y z 坐标
            for p in pts:
                f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
        
        self.get_logger().info(f"🎉 保存成功！文件已存至：{save_path}")
        self.get_logger().info("您现在可以按 Ctrl+C 关闭程序了。")
        sys.exit(0)

def main():
    rclpy.init()
    node = MapSaver()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
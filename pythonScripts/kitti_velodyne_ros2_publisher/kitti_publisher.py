import os
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from rclpy.time import Time as RclpyTime 

def read_velodyne_bin(file_path):
    scan = np.fromfile(file_path, dtype=np.float32).reshape(-1, 4)
    points = scan[:, 0:3]  # x,y,z
    return points

def create_pointcloud2(points, frame_id, stamp):
    num_points = points.shape[0]
    
    # Crea una colonna di timestamp (1 per punto)
    t_array = np.full((num_points, 1), stamp.sec + stamp.nanosec * 1e-9, dtype=np.float32)
    
    # Concatenazione: [x y z t]
    points_with_t = np.hstack((points, t_array))

    msg = PointCloud2()
    msg.header = Header()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = num_points

    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='t', offset=12, datatype=PointField.FLOAT32, count=1),
    ]

    msg.is_bigendian = False
    msg.point_step = 16  # 4 * 4 bytes float32
    msg.row_step = msg.point_step * num_points
    msg.is_dense = True
    msg.data = points_with_t.astype(np.float32).tobytes()

    return msg


class KittiPublisher(Node):
    def __init__(self, dataset_path):
        super().__init__('kitti_velodyne_publisher')
        self.dataset_path = dataset_path
        self.pub = self.create_publisher(PointCloud2, '/pointcloud_topic', 10)
        self.velodyne_dir = os.path.join(dataset_path, "velodyne")
        self.files = sorted(os.listdir(self.velodyne_dir))
        self.index = 0

        # Legge i timestamp come float (secondi)
        timestamps_path = os.path.join(dataset_path, "times.txt")
        with open(timestamps_path, "r") as f:
            self.timestamps = [float(line.strip()) for line in f.readlines()]

        if len(self.files) != len(self.timestamps):
            self.get_logger().error("Numero di point cloud e timestamp non coincide!")
            rclpy.shutdown()
            return

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def timer_callback(self):
        if self.index >= len(self.files):
            self.get_logger().info("Finito di pubblicare tutti i pointclouds!")
            rclpy.shutdown()
            return

        file_path = os.path.join(self.velodyne_dir, self.files[self.index])
        points = read_velodyne_bin(file_path)

        # Estrai il timestamp float in secondi e converti in ROS2 Time
        t = self.timestamps[self.index]
        sec = int(t)
        nanosec = int((t - sec) * 1e9)
        stamp = RclpyTime(seconds=sec, nanoseconds=nanosec).to_msg()

        msg = create_pointcloud2(points, 'velodyne', stamp)
        self.pub.publish(msg)

        self.get_logger().info(f"Pubblicato: {self.files[self.index]} @ {t:.6f} s")
        self.index += 1

def main(args=None):
    rclpy.init(args=args)
    dataset_path = "/home/otto/Desktop/kitti_data/sequences/00"  
    node = KittiPublisher(dataset_path)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()


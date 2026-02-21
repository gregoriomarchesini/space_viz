import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__('trajectory_visualizer')
        
        self.n_cubes = 5
        self.dt = 0.1
        self.max_history = 100  # Number of points to keep for each tail
        
        # State: [x, y, z, vx, vy, vz]
        self.states = [[float(i), float(i),float(i), 0.0, 0.0, 0.0] for i in range(self.n_cubes)]
        
        # History: List of lists to store past Points for each cube
        self.histories = [[] for _ in range(self.n_cubes)]

        self.marker_pub = self.create_publisher(MarkerArray, 'cube_markers', 10)
        self.path_pub = self.create_publisher(MarkerArray, 'trajectory_markers', 10)
        self.timer = self.create_timer(self.dt, self.run_loop)

    def run_loop(self):
        cubes = MarkerArray()
        paths = MarkerArray()
        now   = self.get_clock().now().to_msg()

        for i in range(self.n_cubes):
            # 1. Physics Update (Example: Circular Motion / Spiral)
            # You would replace this with your actual controller u
            ax = -0.5 * self.states[i][0]
            ay = -0.5 * self.states[i][1]
            u = [ax, ay, 0.05] 
            
            # v = v + u*dt ; p = p + v*dt
            self.states[i][3:6] = [v + a * self.dt for v, a in zip(self.states[i][3:6], u)]
            self.states[i][0:3] = [p + v * self.dt for p, v in zip(self.states[i][0:3], self.states[i][3:6])]

            # 2. Update History
            new_pt = Point(x=self.states[i][0], y=self.states[i][1], z=self.states[i][2])
            self.histories[i].append(new_pt)
            if len(self.histories[i]) > self.max_history:
                self.histories[i].pop(0)

            # 3. Create Cube Marker
            cube = Marker()
            cube.header.frame_id, cube.header.stamp = "world", now
            cube.ns, cube.id, cube.type = "cubes", i, Marker.CUBE
            cube.pose.position = new_pt
            cube.scale.x = cube.scale.y = cube.scale.z = 1.0
            cube.color.r, cube.color.g, cube.color.a = 1.0, 1.0, 1.0
            cubes.markers.append(cube)

            # 4. Create Trajectory (LINE_STRIP)
            path = Marker()
            path.header.frame_id, path.header.stamp = "world", now
            path.ns, path.id, path.type = "paths", i, Marker.LINE_STRIP
            path.action = Marker.ADD
            path.points = list(self.histories[i]) # All past points
            path.scale.x = 0.05 # Thickness of the line
            path.color.r = 0.0
            path.color.g = 0.5 + (0.1 * i)
            path.color.b = 1.0
            path.color.a = 0.6 # Slightly transparent
            paths.markers.append(path)

        self.marker_pub.publish(cubes)
        self.path_pub.publish(paths)

def main():
    rclpy.init()
    node = TrajectoryVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
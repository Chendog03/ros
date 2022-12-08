from eufs_msgs.msg import WaypointArrayStamped, Waypoint, ConeArrayWithCovariance, CarState
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from rclpy.node import Node
import rclpy

import numpy as np
from scipy import interpolate

import colorsys

class Planner(Node):
    def __init__(self, name):
        super().__init__(name)
        # Declare ROS parameters
        self.threshold = self.declare_parameter("threshold", 6.0).value
        
        self.car_position = [0.0, 0.0]

        # Create subscribers
        self.cones_sub = self.create_subscription(ConeArrayWithCovariance, "/fusion/cones", self.cones_callback, 1)
        self.car_sub = self.create_subscription(CarState, "/odometry_integration/car_state", self.car_callback, 1)

        # Create publishers
        self.track_line_pub = self.create_publisher(WaypointArrayStamped, "/trajectory", 1)
        self.visualization_pub = self.create_publisher(Marker, "/planner/viz", 1)
        
    def car_callback(self, msg):
        '''self.get_logger().info('Car State: "%s"' % msg)'''
    
        car_x = msg.pose.pose.position.x
        car_y = msg.pose.pose.position.y
        
        self.car_position = [car_x, car_y]
        

    def cones_callback(self, msg):
        blue_cones = self.convert(msg.blue_cones)
        yellow_cones = self.convert(msg.yellow_cones)
        orange_cones = self.to_2d_list(self.convert(msg.orange_cones))
        uncolored_cones = self.convert(msg.unknown_color_cones)

        # add uncolored lidar cones to the appropriate sides
        close_uncolored = uncolored_cones[np.abs(uncolored_cones) < 4]
        blue_cones = self.to_2d_list(np.concatenate((blue_cones, close_uncolored[close_uncolored.imag > 0])))
        yellow_cones = self.to_2d_list(np.concatenate((yellow_cones, close_uncolored[close_uncolored.imag < 0])))

        midpoints = self.find_midpoints(blue_cones, yellow_cones, orange_cones)
        midpoints = self.sort_midpoints(midpoints)

        # Convert back to complex
        midpoints_c = np.array([])
        for m in midpoints:
            midpoints_c = np.append(midpoints_c, m[0] + 1j * m[1])

        if len(midpoints_c) == 0:
            return

        # Here we interpolate the path to artificially increase the number of midpoints so that the controllers
        # have more information to work with you don't need to worry about this step
        try:
            tck, _, = interpolate.splprep(np.array([midpoints_c.real, midpoints_c.imag]), k=3, s=100)
            x_i, y_i = interpolate.splev(np.linspace(0, 1, 10), tck)
            midpoints_c = np.array(x_i + 1j * y_i)
        except:
            self.get_logger().info("Failed to interpolate")
            
        self.publish_path(midpoints_c)
        self.publish_visualisation(midpoints_c)
        
        
    def find_distance(self, cone1, cone2):
        d_x = cone1[0] - cone2[0]
        d_y = cone1[1] - cone2[1]
        
        return (np.sqrt(d_x**2 + d_y**2))
       
       
    def calculate_midpoint(self, cone1, cone2):
        d_x = cone1[0] - cone2[0]
        d_y = cone1[1] - cone2[1]
        
        return [cone1[0] - d_x/2, cone1[1] - d_y/2]     


    def find_midpoints(self, blue_cones, yellow_cones, orange_cones):
        """
        IMPLEMENT YOURSELF
        Find the midpoints along the track
        :param blue_cones: cone positions
        :param yellow_cones:
        :param orange_cones:
        :return: list of midpoints
        """
        
        midpoints = []
        
        for blue_cone in blue_cones:
            for yellow_cone in yellow_cones:
                if self.find_distance(blue_cone, yellow_cone) < self.threshold:
                    midpoints.append(self.calculate_midpoint(blue_cone, yellow_cone))
	       
        '''self.get_logger().info('\nMidpoints: \n' + self.format_array(midpoints))'''
        
        return midpoints

    def sort_midpoints(self, midpoints):
        """
        IMPLEMENT YOURSELF
        Sort the midpoints to so that each consecutive midpoints is further from the car along the path
        :param midpoints:
        :return: sorted midpoints
        """
        
        sorted_midpoints = []
        
        distances = []
        
        for i in range(len(midpoints)):
            midpoint = midpoints[i]
            distances.append(self.find_distance(self.car_position, midpoint))
            
        '''self.get_logger().info('\nDistances: \n' + self.format_array(distances))'''
            
        for i in range(len(distances)):
            min_distance = min(distances)
            min_index = distances.index(min_distance)
            
            sorted_midpoints.append(midpoints[min_index])
            del distances[min_index]
            del midpoints[min_index]
        
        '''self.get_logger().info('\nSorted Midpoints: \n' + self.format_array(sorted_midpoints))'''

        return sorted_midpoints
        
        
    def format_array(self, array):
        text = ""
        
        for item in array:
            text += '"%s" \n' % item
            
        return text
            
       

    def publish_path(self, midpoints):
        waypoint_array = WaypointArrayStamped()
        waypoint_array.header.frame_id = "base_footprint"
        waypoint_array.header.stamp = self.get_clock().now().to_msg()

        for p in midpoints:
            point = Point(x=p.real, y=p.imag)
            waypoint = Waypoint(position=point)
            waypoint_array.waypoints.append(waypoint)

        self.track_line_pub.publish(waypoint_array)

    
        
    def publish_visualisation(self, midpoints):
        marker = Marker()
        marker.header.frame_id = "base_footprint"
        marker.action = Marker.ADD
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.POINTS
        
        marker.id = 0
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.ns = "midpoints"
        
        for i in range(len(midpoints)):
            midpoint = midpoints[i]
            
            marker.points.append(Point(x=midpoint.real, y=midpoint.imag))
            
            rgb_values = colorsys.hsv_to_rgb(i/len(midpoints), 1.0, 1.0)
            marker.colors.append(ColorRGBA(a=1.0, r=rgb_values[0], g=rgb_values[1], b=rgb_values[2]))
            
        self.visualization_pub.publish(marker)
            


    def convert(self, cones):
        """
        Converts a cone array message into a np array of complex
        """
        return np.array([c.point.x + 1j * c.point.y for c in cones])

    def to_2d_list(self, arr):
        """
        Convert an array of complex numbers to a 2d array [[x1,y1], ...]
        :param arr: array of complex
        :return: 2d list
        """
        return [[x.real, x.imag] for x in arr]

def main():
    rclpy.init(args=None)
    node = Planner("local_planner")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

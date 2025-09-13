#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from my_rb1_ros.srv import Rotate, RotateResponse
import math

class RotateService:
    def __init__(self):
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.yaw = 0.0

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        self.yaw = yaw

    def handle_rotate(self, req):
        rospy.loginfo("Rotate request: %d degrees" % req.degrees)
        start_yaw = self.yaw
        target_angle = math.radians(req.degrees)

        twist = Twist()
        angular_speed = 0.3  # rad/s
        direction = 1 if target_angle > 0 else -1
        twist.angular.z = direction * angular_speed

        r = rospy.Rate(20)
        turned = 0.0

        while not rospy.is_shutdown() and abs(turned) < abs(target_angle):
            self.cmd_pub.publish(twist)
            current_yaw = self.yaw
            turned = self.normalize_angle(current_yaw - start_yaw)
            if direction * turned < 0:  # handle wrap-around at ±pi
                turned += direction * 2 * math.pi
            r.sleep()

        # stop robot
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

        return RotateResponse("Rotation of %d degrees completed" % req.degrees)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle

def main():
    rospy.init_node('rotate_service_node')
    service = RotateService()
    rospy.Service('/rotate_robot', Rotate, service.handle_rotate)
    rospy.loginfo("Rotate service is ready")
    rospy.spin()

if __name__ == '__main__':
    main()
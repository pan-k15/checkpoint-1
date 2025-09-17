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
        rospy.loginfo("[ROTATE SERVICE] REQUESTED: Rotate %d degrees" % req.degrees)

        start_yaw = self.yaw
        target_angle = math.radians(req.degrees)

        twist = Twist()
        direction = 1 if target_angle > 0 else -1
        r = rospy.Rate(50)  # 50 Hz loop

        turned = 0.0
        prev_yaw = start_yaw

        # Angular speed to complete rotation in ~3 sec
        angular_speed = abs(target_angle) / 2.0
        angular_speed = max(0.3, min(angular_speed, 1.5))  # clamp speed
        rospy.loginfo("[ROTATE SERVICE] Angular speed set to %.2f rad/s" % angular_speed)

        while not rospy.is_shutdown() and abs(turned) < abs(target_angle) - math.radians(2):
            current_yaw = self.yaw
            # Compute delta and fix wrap-around
            delta_yaw = current_yaw - prev_yaw
            if delta_yaw > math.pi:
                delta_yaw -= 2 * math.pi
            elif delta_yaw < -math.pi:
                delta_yaw += 2 * math.pi

            turned += delta_yaw
            prev_yaw = current_yaw

            twist.angular.z = direction * angular_speed
            self.cmd_pub.publish(twist)
            r.sleep()

        # Stop the robot
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

        rospy.loginfo("[ROTATE SERVICE] COMPLETED: Rotation of %d degrees finished" % req.degrees)
        return RotateResponse("Rotation of %d degrees completed" % req.degrees)

def main():
    rospy.init_node('rotate_service_node')
    service = RotateService()
    rospy.Service('/rotate_robot', Rotate, service.handle_rotate)
    rospy.loginfo("[ROTATE SERVICE] READY")
    rospy.spin()

if __name__ == '__main__':
    main()


from ackermann_msgs.msg import AckermannDriveStamped

class Driving:
    def __init__(self, drive_publisher):
        self.drive_publisher = drive_publisher

    def send_drive_command(self, speed, steering_angle):
        """Send drive commands to the robot."""
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.header.frame_id = 'base_link'
        self.drive_publisher.publish(drive_msg)

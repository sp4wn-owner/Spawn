import rospy
from sensor_msgs.msg import CompressedImage
import sys

def image_callback(msg):
    format = msg.format
    compressed_image_data = msg.data
    process_image(format, compressed_image_data)

def process_image(format, image_data):
    print(f'Received compressed image data in {format} format: {len(image_data)} bytes')
    sys.stdout.buffer.write(image_data)
    sys.stdout.flush()

def main():
    rospy.init_node('camera_subscriber', anonymous=True)
    rospy.Subscriber('/hdas/camera_head/left_raw/image_raw_color/compressed', CompressedImage, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
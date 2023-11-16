import cv2
import rosbag
from cv_bridge import CvBridge

# 输入的rosbag文件和图像消息的topic
bag_file = 'temp3.bag'
image_topic = 'image'
# 输出视频文件
output_video_path = '3.mp4'


# 打开rosbag文件
bag = rosbag.Bag(bag_file, 'r')
# 获取图像消息数量和第一个图像的大小
bridge = CvBridge()

# 创建视频写入对象
frame_size = (2208 ,1242)
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
video_writer = cv2.VideoWriter(output_video_path, fourcc, 10, frame_size)

# 读取图像消息并写入视频
for topic, msg, t in bag.read_messages(topics=[image_topic]):
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    cv2.imshow("1",cv_image)
    cv2.waitKey(1)
    video_writer.write(cv_image)

# 释放资源
bag.close()
video_writer.release()
cv2.destroyAllWindows()

print(f"视频已成功生成：{output_video_path}")

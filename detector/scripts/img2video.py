import cv2
import os

# 设置图像目录和输出视频文件名
image_dir = './1/'
output_video_path = 'output_video.mp4'

# 设置视频帧率和分辨率
fps = 10
first_image_path = os.path.join(image_dir, '0.jpg')
first_frame = cv2.imread(first_image_path)
frame_size = (first_frame.shape[1], first_frame.shape[0]) #2208 × 1242

# 创建视频写入对象
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
video_writer = cv2.VideoWriter(output_video_path, fourcc, fps, frame_size)

# 遍历0到832的数字，构建图像文件路径并写入视频
for num in range(833):  # range的终止值取不到，所以设置为833以包含832
    image_path = os.path.join(image_dir, f"{num}.jpg")
    frame = cv2.imread(image_path)

    # 如果图像读取成功，写入视频
    if frame is not None:
        frame = cv2.resize(frame, frame_size)
        video_writer.write(frame)

# 释放资源
video_writer.release()
cv2.destroyAllWindows()

print(f"视频已成功生成：{output_video_path}")

import rosbag

bag_file = '2023-11-14-11-23-27.bag'

bag = rosbag.Bag(bag_file,'r')
temp = rosbag.Bag("temp2.bag","w")
bagmessage = bag.read_messages("/image_topic")
for topic,msg,t in bagmessage :
	temp.write("image",msg)
print("11")
temp.close()
bag.close()
	

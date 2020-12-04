import rosbag
from geometry_msgs.msg import Point
# import pandas as pd

# The bag file should be in the same directory as your terminal
bag = rosbag.Bag('sample.bag')
# topic = '/your_topic'
# column_names = ['x', 'y']
# df = pd.DataFrame(columns=column_names)

for topic, msg, t in bag.read_messages(topics=topic):
	print(topic, mst, t)
#     x = msg.x
#     y = msg.y

#     df = df.append(
#         {'x': x,
#          'y': y},
#         ignore_index=True
#     )

# df.to_csv('out.csv')
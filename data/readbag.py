import os
import argparse
import rosbag
import pandas as pd

parser = argparse.ArgumentParser()
parser.add_argument('datafile', default='data_01', type=str, help='data file name')
args = parser.parse_args()
df = args.datafile

# bag
bag = rosbag.Bag(df+'/data.bag')

# parameters
param = pd.read_csv(df + '/param.csv')
param = param.set_index('param').T
param = param.to_dict('records')[0]
param['cf_dict'] = {'D'+str(d): cf for d, cf in enumerate(param['cf_dict_d'].split('_'))}
param['cf_dict'].update({'I'+str(i): cf for i, cf in enumerate(param['cf_dict_i'].split('_'))})

for p, cf in param['cf_dict']:

	df = pd.DataFrame(columns=['x', 'y', 'z', 'vx', 'vy', 'vz'])

	for topic, msg, ros_t in bag.read_messages(topic='/'+cf+'/mocap'):

		t = ros_t.secs + ros_t.nsecs * 1e-9

		x = msg.position.x
		y = msg.position.y
		z = msg.position.z

	    vx = msg.velocity.x
		vy = msg.velocity.y
		vz = msg.velocity.z

	    df = df.append(
	        {'t': t,
	         'x': x,
	         'y': y,
	         'z': z,
	         'vx': vx,
	         'vy': vy,
	         'vz': vz},
	        ignore_index=True
	    )

	df.to_csv(df+'/'+p+'/State.csv')
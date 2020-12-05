import os
import argparse
import rosbag
import pandas as pd

parser = argparse.ArgumentParser()
parser.add_argument('datafile', default='data_00', type=str, help='data file name')
args = parser.parse_args()
fdir = '/home/flora/crazyflie_ws/src/crazyflie_mdmifd/data/' + args.datafile

# bag
bag = rosbag.Bag(fdir+'/2020-12-05-14-14-15.bag')

# parameters
param = pd.read_csv(fdir + '/D0/param.csv')
param = param.set_index('param').T
param = param.to_dict('records')[0]
temp, cf_dict = param['cf_dict'].split('!'), dict()
for rcf in temp:
	r_cf = rcf.split('_')
	cf_dict.update({r_cf[0]:r_cf[1]}) # role: cf


# cf_dict = {'D'+str(d): cf for d, cf in enumerate(param['cf_dict'].split('_'))}

# param = pd.read_csv(df + '/I0/param.csv')
# param = param.set_index('param').T
# param = param.to_dict('records')[0]
# temp = param['cf_dict'].split('!')
# for rcf in temp:
# 	r_cf = rcf.split('_')
# 	cf_dict.update({r_cf[0]:r_cf[1]}) # role: cf
# cf_dict.update({'I'+str(i): cf for i, cf in enumerate(param['cf_dict'].split('_'))})

for p, cf in cf_dict.items():
	# print(p, cf)

	df = pd.DataFrame(columns=['x', 'y', 'z', 'vx', 'vy', 'vz'])

	for topic, msg, ros_t in bag.read_messages(topics=['/'+cf+'/mocap']):
		# print(msg)

		t = ros_t.secs + ros_t.nsecs * 1e-9

		x = msg.position[0]
		y = msg.position[1]
		z = msg.position[2]

		vx = msg.velocity[0]
		vy = msg.velocity[1]
		vz = msg.velocity[2]

		# print(t, x, y, z)

		df = df.append(
	    	{'t': t,
			 'x': x,
			 'y': y,
			 'z': z,
			 'vx': vx,
			 'vy': vy,
			 'vz': vz},
			 ignore_index=True)

		# print(fdir+'/'+p+'/State.csv')

	df.to_csv(fdir+'/'+p+'/State.csv')
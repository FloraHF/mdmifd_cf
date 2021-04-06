import os
import argparse
import rosbag
import pandas as pd

parser = argparse.ArgumentParser()
parser.add_argument('datafile', default='data_00', type=str, help='data file name')
args = parser.parse_args()
# fdir = '~/home/flora/crazyflie_ws/src/crazyflie_mdmifd/data/' + args.datafile
fdir = '/home/flora/crazyflie_mdmifd/data/' + args.datafile

# read the most recent bag
# print(next(os.walk(fdir)))
imax, tmax, dmax = 0, 0, ''
for i, t in enumerate(next(os.walk(fdir))[2]): # upstream dir, dir, files
	# print(i, t)
	ttemp = int(''.join([s for s in t.split('.')[0].split('-')]))
	if ttemp > tmax:
		imax = i
		tmax  = ttemp
		dmax = t
# print(fdir+'/'+dmax)
bag = rosbag.Bag(fdir+'/'+dmax)

# parameters
param = pd.read_csv(fdir + '/D0/param.csv').set_index('param').T.to_dict('records')[0]
temp, cf_dict = param['cf_dict'].split('!'), dict()
for rcf in temp:
	r_cf = rcf.split('_')
	cf_dict.update({r_cf[0]:r_cf[1]}) # role: cf

# for p, cf in cf_dict.items():
# 	for topic, msg, ros_t in bag.read_messages(topics=['/'+cf+'/cmd_vel']):
# 		print(msg.linear.x)

for p, cf in cf_dict.items():

	# locations and velocities
	df = pd.DataFrame(columns=['t', 'x', 'y', 'z', 'vx', 'vy', 'vz'])
	for topic, msg, ros_t in bag.read_messages(topics=['/'+cf+'/mocap']):

		t = ros_t.secs + ros_t.nsecs * 1e-9

		x = msg.position[0]
		y = msg.position[1]
		z = msg.position[2]

		vx = msg.velocity[0]
		vy = msg.velocity[1]
		vz = msg.velocity[2]

		df = df.append(
	    	{'t': t,
			 'x': x,
			 'y': y,
			 'z': z,
			 'vx': vx,
			 'vy': vy,
			 'vz': vz},
			 ignore_index=True)

	out_file = fdir+'/'+p+'/State.csv'
	if os.path.exists(out_file): 
		os.remove(out_file) 

	df.to_csv(out_file)

	# commands
	df = pd.DataFrame(columns=['t', 'vx', 'vy'])
	for topic, msg, ros_t in bag.read_messages(topics=['/'+cf+'/cmd_vel']):

		t = ros_t.secs + ros_t.nsecs * 1e-9

		vx = -msg.linear.y
		vy = msg.linear.x

		df = df.append(
	    	{'t': t,
			 'vx': vx,
			 'vy': vy},
			 ignore_index=True)

	out_file = fdir+'/'+p+'/Command.csv'
	if os.path.exists(out_file): 
		os.remove(out_file) 
		
	df.to_csv(out_file)	
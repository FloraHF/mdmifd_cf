#!/usr/bin/env python3

import os
import shutil
# import sys
# sys.path
# sys.path.append('/home/flora/crazyflie_ws/src/')
# import crazyflie_mdmifd.msg

import numpy as np
import rospy
from datetime import datetime

# messages
# from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from optitrack_broadcast.msg import Mocap

# services
from mdmi_game.srv import DroneStatus, DroneStatusResponse
# from crazyflie_mdmifd.msg import Mocap
from std_srvs.srv import Empty

# collision avoidance: orca
from pyorca.pyorca import Agent, orca

# for the game
from Geometries import CircleTarget
from utils import norm, dist, PlayerState
from utils import mcap_msg_to_player_state


class PlayerNode(object):

	def __init__(self, i, x, vmax, z=.5,
					   r=1., Rtarg=3.9, 
					   ni=1, nd=2, Rteam=5., Roppo=5., 
					   resid='res0', cf='cf0', 
					   cf_dict=''):
		# environment settings
		self.target = CircleTarget(Rtarg)
		self.ni = ni
		self.nd = nd
		self.t = 0.
		date, time = str(datetime.now()).split(' ')
		self.t0 = '-'.join([date] + time.split('.')[0].split(':'))

		# crazyflie frame
		self.cf_id = cf
		# print(cf_dict)
		temp, self.cf_dict = cf_dict.split('!'), dict()
		for rcf in temp:
			r_cf = rcf.split('_')
			self.cf_dict.update({r_cf[0]:r_cf[1]}) # role: cf
		# print(self.cf_dict)

		# player settings
		self.altitude = z
		self.r = r
		self.id = i
		self.Rt = Rteam
		self.Ro = Roppo

		self.status = ['prepare', '']
		self.x0 = x
		self.vmax = vmax
		self.state = PlayerState(self.t, self.x0, self.altitude)

		# id of all the players
		self.team_all = self.get_team()
		self.oppo_all = self.get_oppo()

		# states of neighbours
		self.state_team_neigh = dict()
		self.state_oppo_neigh = dict()

		# subscribers
		self.self_sub = rospy.Subscriber('/'+self.cf_id+'/mocap', Mocap, self.selfsub_callback)
		self.team_subs = [rospy.Subscriber('/'+self.cf_dict[str(p)]+'/mocap', Mocap, \
							self.get_statesub_callback(p, self.state_team_neigh, self.Rt)) for p in self.team_all] 
		self.oppo_subs = [rospy.Subscriber('/'+self.cf_dict[str(p)]+'/mocap', Mocap, \
							self.get_statesub_callback(p, self.state_oppo_neigh, self.Ro)) for p in self.oppo_all] 

		# publisher
		self.cmdX_pubs = rospy.Publisher('/'+self.cf_id+'/goal', PoseStamped, queue_size=1)
		self.cmdV_pubs = rospy.Publisher('/'+self.cf_id+'/cmdV', Twist, queue_size=1)
		# self.status_pubs = rospy.Publisher('/'+self.cf_id+'/status', String, queue_size=1)

		# service
		rospy.Service('/crazyflie2_'+str(self)+'/set_status', DroneStatus, self.status_srv_callback)

		self.takeoff_client = self.service_client(self.cf_id, '/cftakeoff')
		self.land_client = self.service_client(self.cf_id, '/cfland')
		self.play_client = self.service_client(self.cf_id, '/cfplay')
		self.auto_client = self.service_client(self.cf_id, '/cfauto')


		# create files for data saving, clear existing files if exist
		self.datadir = '/home/flora/crazyflie_ws/src/crazyflie_mdmifd/data/'+resid+'/'+str(self)
		if os.path.exists(self.datadir): 
			shutil.rmtree(self.datadir)
		os.makedirs(self.datadir)
		# print('!!!!!!!!!!!!!!', self.datadir)

		# record parameters used by the player
		with open(self.datadir + '/param.csv', 'w') as f:
			
			# title
			f.write('param,value\n')

			# target info
			f.write('target:type,%s\n'%str(self.target)) 
			f.write('target:x0,%.2f\n'%self.target.x0)
			f.write('target:y0,%.2f\n'%self.target.y0)
			if 'circle' in str(self.target):
				f.write('target:R,%.2f\n'%self.target.size)

			f.write('nd,%d\n'%self.nd)
			f.write('ni,%d\n'%self.ni)
			
			# player info
			f.write('id,%s\n'%str(self))
			f.write('r,%.2f\n'%self.r)
			f.write('vmax,%.2f\n'%self.vmax) 
			f.write('x0,%.2f\n'%self.x0[0])
			f.write('y0,%.2f\n'%self.x0[1])
			f.write('z,%.2f\n'%self.altitude)
			f.write('Rteam,%.2f\n'%self.Rt)
			f.write('Roppo,%.2f\n'%self.Ro)
			f.write('cf_dict,%s\n'%cf_dict)

	# ============ functions for the game ============
	def get_team(self):
		raise NotImplementedError

	def get_oppo(self):
		raise NotImplementedError

	def strategy(self):
		raise NotImplementedError

	def capture_handler(self):
		raise NotImplementedError

	def entering_handler(self):
		raise NotImplementedError

	def is_capture(self, oppo):
		if oppo not in self.state_oppo_neigh:
			return False
		return dist(self.state.x, self.state_oppo_neigh[oppo].x) < self.r

	# ============ actions for different status ============
	def get_cmdX_msg(self, x, z):
		cmdX_msg = PoseStamped()
		cmdX_msg.header.frame_id = '/world'
		cmdX_msg.header.stamp = rospy.Time.now()
		cmdX_msg.pose.position.x = x[0]
		cmdX_msg.pose.position.y = x[1]
		cmdX_msg.pose.position.z = z
		return cmdX_msg

	def standby(self):
		cmdX_msg = self.get_cmdX_msg(self.state.x, 0.2)
		self.cmdX_pubs.publish(cmdX_msg)

	# def prepare(self):
	# 	cmdX_msg = self.get_cmdX_msg(self.x0, self.altitude)
	# 	self.cmdX_pubs.publish(cmdX_msg)

	def win(self):
		pass

	def play(self):
		u_pref = self.strategy()
		orca_self = Agent(self.state.x, self.state.v, .1, u_pref)
		orca_other = []

		for other, state in self.state_team_neigh.items():
			orca_other.append(Agent(state.x, state.v, .1, (0, 0)))

		u, _ = orca(orca_self, orca_other, 2., 1e-2)
		
		# velocity
		cmdV_msg = Twist()
		cmdV_msg.linear.x = u[0]
		cmdV_msg.linear.y = u[1]

		# location
		cmdX_msg = self.get_cmdX_msg(self.x0, self.state.z)

		# publish
		self.cmdV_pubs.publish(cmdV_msg)
		self.cmdX_pubs.publish(cmdX_msg)

	def deploy(self):
		cmdX_msg = self.get_cmdX_msg(self.x0, self.altitude)
		self.cmdX_pubs.publish(cmdX_msg)

	def land(self):
		cmdX_msg = self.get_cmdX_msg(self.x0, 0.2)
		self.cmdX_pubs.publish(cmdX_msg)

	# ============ service clients ============
	def service_client(self, cf, name): # server for this service is defined in the controller
		srv_name = '/' + cf + name
		rospy.wait_for_service(srv_name)
		rospy.loginfo(str(self) +' found ' + srv_name + ' service')
		return rospy.ServiceProxy(srv_name, Empty)	

	# ============ service callbacks ============
	def status_srv_callback(self, req):
		self.status[0] = req.status
		print("received service call")
		if req.status == 'deploy':
			self.takeoff_client()
		if req.status == 'play':
			self.play_client()
		if req.status == 'land':
			self.land_client()
		if req.status == 'standby':
			self.auto_client()
		return DroneStatusResponse(self.status[0])

	# ============ subscriber callbacks ============
	def selfsub_callback(self, msg):
		state = mcap_msg_to_player_state(msg)
		self.state.update_xzv(t=state.t, x=state.x, z=state.z, v=state.v)

	def get_statesub_callback(self, p, pset, R):
		def sub_callback(msg):
			state = mcap_msg_to_player_state(msg)
			if dist(state.x, self.state.x) <= R and state.z > 0.23:
				pset.update({p: state})
			else:
				pset.pop(p, '')
		return sub_callback	

	# ============ main iteration ============
	def iteration(self, event):

		if not rospy.is_shutdown():
			# if str(self) == 'I2':
			# 	print(str(self), self.x0)

			if self.status[0] == 'play':
				self.play()
				self.capture_handler()
				self.entering_handler()

			if self.status[0] == 'standby':
				self.standby()

			if self.status[0] == 'win':
				self.win()

			if self.status[0] == 'deploy':
				self.deploy()

			if self.status[0] == 'land':
				self.status = ['prepare', '']
				self.land_client()

	# ============ class functions ============
	def __repr__(self):
		return ''

import os
import numpy as np
import pandas as pd

from scipy.interpolate import interp1d
from Envs.scenarios.game_mdmi.utils import prefstring_to_list

# result file
resid = 'res_00_05_value'
res_path = '/home/flora/mdmi_data/' + resid + '/'

# find out players recorded, and sort by their id
players = [p for p in next(os.walk(res_path))[1]]
defenders = sorted([p for p in players if 'D' in p], key=lambda x: int(x[1:]))
intruders = sorted([p for p in players if 'I' in p], key=lambda x: int(x[1:]))
players = defenders + intruders

# read player parameters
def read_exp_param(res_path=res_path):
	params = dict()
	for p in players:
		data = pd.read_csv(res_path + p + '/param.csv')
		data = data.set_index('param').T
		params[p] = data.to_dict('records')[0]
		for k in params[p]:
			if 'target:' in k:
				if 'type' in k:
					params[p][k] = params[p][k]
				else:
					params[p][k] = float(params[p][k])
			elif k != 'id' and k != 'iselect_mode':
				if k == 'ni' or k == 'nd':
					params[p][k] = int(params[p][k])
				else:
					params[p][k] = float(params[p][k])
			else:
				params[p][k] = params[p][k]

		params[p]['cf_dict'] = {'D'+str(d): cf for d, cf in enumerate(params[p]['cf_dict_d'].split('_'))}
		params[p]['cf_dict'].update({'I'+str(i): cf for i, cf in enumerate(params[p]['cf_dict_i'].split('_'))})

	xds=[np.array([params[d]['x0'], params[d]['y0']]) for d in defenders]
	xis=[np.array([params[i]['x0'], params[i]['y0']]) for i in intruders]

	return {'r': params['D0']['r'] 	,
			'nd': params['D0']['nd'],	# same for all players
			'ni': params['D0']['ni'],	# same for all players
			'Rt': params['D0']['Rteam'],
			'Ro': params['D0']['Roppo'],
			'vd': params['D0']['vmax'],	# same for all defenders
			'vi': params['I0']['vmax'],	# same for all intruders
			'xds': xds,
			'xis': xis,
			'x0targ': params['D0']['target:x0'],
			'y0targ': params['D0']['target:y0'],
			'Rtarg': params['D0']['target:R'],
			'cf_dict': params['D0']['target:R'],
			'iselect_mode': params['D0']['iselect_mode']}

# read players states
def read_exp_state(res_path=res_path, tmin=0., tmax=100.):
	state = dict()
	for p in players:
		data = pd.read_csv(res_path + p + '/State.csv')
		t = data['t'].to_numpy()
		tmin = max(min(t), tmin)
		tmax = min(max(t), tmax)
		state[p] = {k:interp1d(t, data[k].to_numpy(), fill_value='extrapolate') for k in ['x', 'y', 'z', 'vx', 'vy']}
		state[p].update({'tmin': t[0]})

	return state, tmin, tmax

# read player commands
def read_exp_cmd(res_path=res_path, tmin=0., tmax=100.):
	cmd = dict()
	for p in players:
		data = pd.read_csv(res_path + p + '/Command.csv')
		t = data['t'].to_numpy()
		tmin = max(min(t), tmin)
		tmax = min(max(t), tmax)
		cmd[p] = {k:interp1d(t, data[k].to_numpy()) for k in ['vx', 'vy']}
	return cmd, tmin, tmax


def read_exp_cap(res_path=res_path, tmax=100):
	cap_gazebo = {i:{'dcap': None, 'tcap': np.inf, 'tent': np.inf} for i in intruders}
	maxte = 0
	for i in intruders:
		cap_data = pd.read_csv(res_path + i + '/Dcap.csv')
		ent_data = pd.read_csv(res_path + i + '/Tent.csv')
		if not ent_data['t'].empty:
			# print(i, 'enters at', ent_data['t'].values[-1])
			cap_gazebo[i]['tent'] = ent_data['t'].values[-1]
			maxte = max(maxte, ent_data['t'].values[-1])
		else:
			if not cap_data['t'].empty:
				# print(i, 'is captured at', cap_data['t'].values[-1])
				cap_gazebo[i]['dcap'] = cap_data['d'].values[-1]
				cap_gazebo[i]['tcap'] = cap_data['t'].values[-1]
				maxte = max(maxte, cap_data['t'].values[-1])
		# print(maxte)
	return cap_gazebo, min(maxte, tmax)


def read_exp_assign(data_file, res_path=res_path, toffset=0):
	assign = dict()
	tc = 0
	for d in defenders:
		data = pd.read_csv(res_path + d + data_file)
		t = data['t'].to_numpy() + toffset
		# t = t - min(t)
		i = np.array([int(ii[1:]) for ii in data['i'].to_list()])
		e = data['e'].to_numpy()
		pref = [prefstring_to_list(pstr) for pstr in data['pref']]
		assign[d] = {'t': t,
					 'i': i,
					 'e': e,
					 'pref': pref,
					 'approx': interp1d(t, i)}
		tc = max(tc, t[-1])
	return assign, tc

if __name__ == '__main__':
	# p = read_gazebo_param()
	# # print(p)
	# s, tmin, tmax = read_gazebo_state()
	# s, tmin, tmax = read_gazebo_cmd()
	# cap = read_gazebo_cap()
	ass, tc = read_exp_assign('/Itarg.csv')
	for d, a in ass.items():
		print(a['t'])



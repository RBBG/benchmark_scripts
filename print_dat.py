# -*- coding: utf-8 -*-
"""
Created on Fri Nov 13 17:25:49 2015

@author: ruben
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.mlab as mlab
import cPickle as pickle
import random as rd
import sys

def unfold(data):
	len_w_test = []
	planners = []
	for plnr in xrange(len(data[0][0])-2):
		planners.append(data[0][0][plnr]['name'])		
	df = []     										#data_formatted as np.array
	for scene in xrange(len(data)):								#scene loop
		df.append([])
		for planner in xrange(len(data[scene][0])-2):   				#planner loop   (-start/goal)
			df[scene].append([[],[],[],[],[],[],[],[],[],[],[],[]])   							#adding planner
			for query in xrange(len(data[scene])-1):				#query loop			
			
				time, plantime, len_w, len_j, perc,data_max = 0,0,0,0,0,0
				data_min = 10000

				len_w = []
				len_j = []
				plantime = []
				time = []

				cnt = 0  				
				for iteration in xrange(len(data[scene][query][planner])-1):  	#iterations loop (-planner name)
				
				  if (data[scene][query][planner][iteration]['success']==1):
						cnt = cnt+1
						"""
						totaltime = data[scene][query][planner][iteration]['path'].joint_trajectory.header.seq
						totaltime += data[scene][query][planner][iteration]['path'].joint_trajectory.header.stamp.secs					
			
						time += totaltime
						plantime += data[scene][query][planner][iteration]['path'].joint_trajectory.header.seq
						len_j += data[scene][query][planner][iteration]['length']['joint_path']
						len_w += data[scene][query][planner][iteration]['length']['work_path']

						len_w_test.append(data[scene][query][planner][iteration]['length']['work_path'])
						
						data_max = max(data_max,data[scene][query][planner][iteration]['length']['joint_path'])
						data_min = min(data_min,data[scene][query][planner][iteration]['length']['work_path'])
						"""
						
						totaltime = data[scene][query][planner][iteration]['path'].joint_trajectory.header.seq
						totaltime += data[scene][query][planner][iteration]['path'].joint_trajectory.header.stamp.secs

						time.append(totaltime)
						plantime.append(data[scene][query][planner][iteration]['path'].joint_trajectory.header.seq)
						len_w.append(data[scene][query][planner][iteration]['length']['work_path'])
						len_j.append(data[scene][query][planner][iteration]['length']['joint_path'])

				
				if (cnt > 0):
					"""
					df[scene][planner][0].append(time/cnt)
					df[scene][planner][1].append(len_j/cnt)
					df[scene][planner][2].append(len_w/cnt)
					df[scene][planner][3].append((cnt*100)/(len(data[scene][query][planner])-1))
					df[scene][planner][4].append(plantime/cnt)
					df[scene][planner][5].append((time-plantime)/cnt)

					df[scene][planner][6].append(data_max)
					df[scene][planner][7].append(data_min)
					"""
					
					df[scene][planner][0].append(np.mean(time))
					df[scene][planner][1].append(np.mean(len_j))
					df[scene][planner][2].append(np.mean(len_w))

					df[scene][planner][3].append(cnt*100/(len(data[scene][query][planner])-1))
					df[scene][planner][4].append(np.mean(plantime))
					df[scene][planner][5].append(np.mean(time)-np.mean(plantime))

					df[scene][planner][6].append(max(len_j))
					df[scene][planner][7].append(min(len_j))

					df[scene][planner][8].append(max(len_w))
					df[scene][planner][9].append(min(len_w))

					df[scene][planner][10].append(np.std(time))

				else:

					df[scene][planner][0].append(0)
					df[scene][planner][1].append(0)
					df[scene][planner][2].append(0)
					df[scene][planner][3].append(0)
					df[scene][planner][4].append(0)
					df[scene][planner][5].append(0)

					df[scene][planner][6].append(0)
					df[scene][planner][7].append(0)
					df[scene][planner][8].append(0)
					df[scene][planner][9].append(0)

					df[scene][planner][10].append(0)

	return df,planners

def plot(scenenr, regex):
	#plt.figure()

	f, axarr = plt.subplots(2,2)
	
	n = len(planners)
	bar_jump = (8)/(10.0*n)
	bar_width = bar_jump*1

	bar_colors = []
	xdat = []
	
	for x in xrange(n): 
		bar_colors.append([rd.random(),rd.random(),rd.random()])
		xdat.append([a+x*bar_jump-0.4 for a in range(1,11,1)])

	bar_colors = [[72/255.0,217/255.0,24/255.0],[24/255.0,169/255.0,217/255.0],[217/255.0,72/255.0,24/255.0],[169/255.0,24/255.0,217/255.0],[24/255.0,125/255.0,169/255.0],[255/255.0,241/255.0,117/255.0]]

	#plt.subplot(221)
	axarr[0,0].set_title(r'Jointspace length')
	for x in xrange(n):
		if (planners[x]!="" and regex in planners[x]):
			axarr[0,0].bar(xdat[x],df[scenenr][x][1][:], 
				width = bar_width, 
				color = bar_colors[x],
				label = planners[x])

	axarr[0,0].set_xlabel('Problems')
	axarr[0,0].set_ylabel('2-norm in jointspace')
	axarr[0,0].set_xticks(np.arange(1,11,1))
	axarr[0,0].set_xticks(np.arange(0.6,10.6,1),minor=True)
	axarr[0,0].grid(b=True,which='minor')
	axarr[0,0].grid(b=True,which='major', axis ='y')
	axarr[0,0].set_xlim(left = 0, right = 11)

	axarr[0,1].set_title(r'Workspace Length')
	for x in xrange(n):
		if (planners[x]!="" and regex in planners[x]):
			axarr[0,1].bar(xdat[x],df[scenenr][x][2][:], width = bar_width, color = bar_colors[x],label=planners[x])

	axarr[0,1].set_xlabel('Problems')
	axarr[0,1].set_ylabel('2-norm in workspace')	
	axarr[0,1].set_xticks(np.arange(1,11,1))
	axarr[0,1].set_xticks(np.arange(0.6,10.6,1),minor=True)
	axarr[0,1].grid(b=True,which='minor')
	axarr[0,1].grid(b=True,which='major', axis ='y')
	axarr[0,1].set_xlim(left = 0, right = 11)

	axarr[0,1].legend(bbox_to_anchor=(1.02, 1),loc=2, borderaxespad=0.)

	axarr[1,0].set_title(r'Percentage Solved')
	for x in xrange(len(planners)):
		if (planners[x]!="" and regex in planners[x]): 			
			axarr[1,0].bar(xdat[x],df[scenenr][x][3][:], width = bar_width, color = bar_colors[x],label=planners[x])

	axarr[1,0].set_xlabel('Problems')
	axarr[1,0].set_ylabel('Percentage solved')
	axarr[1,0].set_xticks(np.arange(1,11,1))
	axarr[1,0].set_xticks(np.arange(0.6,10.6,1),minor=True)
	axarr[1,0].grid(b=True,which='minor')
	axarr[1,0].grid(b=True,which='major', axis ='y')
	axarr[1,0].set_xlim(left = 0, right = 11)

	axarr[1,1].set_title(r'Time')
	for x in xrange(len(planners)):
		if (planners[x]!="" and regex in planners[x]):
			plot_data = df[scenenr][x][0][:]

			error_std = df[scenenr][x][10][:]
			error = [[0]*len(error_std), error_std]		

			#xdat = [a+x*0.05 for a in range(0,10,1)]   #offset them slightly for visibility

			#axarr[0,0].errorbar(xdat,plot_data, yerr=error,fmt='--o',label=planners[x])
			axarr[1,1].bar(xdat[x],plot_data,
				yerr = error,
				width = bar_width,
				color = bar_colors[x],
				label = planners[x])
			

			#lbl = axarr[0,0].plot(df[scenenr][x][0][:],label=planners[x])
		


		#if (planners[x]!=""): axarr[0,0].plot(df[scenenr][x][4][:],'--',color=lbl[0].get_color(),label=planners[x]+"_geometric")

	axarr[1,1].set_xlabel('Problems')
	axarr[1,1].set_ylabel('Time in ms')	
	axarr[1,1].set_xticks(np.arange(1,11,1))
	axarr[1,1].set_xticks(np.arange(0.6,10.6,1),minor=True)
	axarr[1,1].grid(b=True,which='minor')
	axarr[1,1].grid(b=True,which='major', axis ='y')
	axarr[1,1].set_xlim(left = 0, right = 11)
	axarr[1,1].set_ylim(bottom = 0)

	axarr[1,1].legend(bbox_to_anchor=(1.02, 1),loc=2, borderaxespad=0.)

def print_dat(scenenr, regex):
	
	n = len(planners)
	print len(df[scenenr][0][0][:])
	for x in xrange(n):
		frac = sum(df[scenenr][x][0][:])/sum(df[scenenr][0][0][:])
		
		print planners[x] + " totaltime: " + str(sum(df[scenenr][x][0][:])/len(df[scenenr][x][0][:]))
		print planners[x] + " avg percentage: " + str(sum(df[scenenr][x][3][:])/len(df[scenenr][x][3][:]))
		print planners[x] + " avg jointspace length: " + str(sum(df[scenenr][x][1][:])/len(df[scenenr][x][1][:]))

if len(sys.argv) > 1:
    load_string = sys.argv[1]
else:
    load_string = "results/benchmark_data.p"


data = pickle.load(open(load_string, "rb" ))

(df,planners) = unfold(data)
for x in xrange(len(df)):
	print_dat(x,"")

plt.show()
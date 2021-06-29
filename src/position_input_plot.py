import matplotlib.pyplot as plt
import numpy as np
from matplotlib.font_manager import FontProperties

font = FontProperties(fname=r"/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",size=12)
x=1
u=0

if(x==1):
	f0= open("/home/cby/drake_learning/src/drake_learning2/src/priordata/x_planed_outsearch.txt", 'r', True)
	#f1= open("/home/cby/drake_learning/src/drake_learning2/src/priordata/x_planed_outsearch.txt", 'r', True)
	#f2= open("/home/cby/drake_learning/src/drake_learning2/src/priordata/x_planed_outsearch.txt", 'r', True)
	#f3= open("/home/cby/drake_learning/src/drake_learning2/src/priordata/x_planed_outsearch.txt", 'r', True)
	fu0= open("/home/cby/drake_learning/src/drake_learning2/src/priordata/u_planed_outsearch.txt", 'r', True)
	#fu1= open("/home/cby/drake_learning/src/drake_learning2/src/priordata/u_planed_outsearch.txt", 'r', True)
	#fu2= open("/home/cby/drake_learning/src/drake_learning2/src/priordata/u_planed_outsearch.txt", 'r', True)
	#fu3= open("/home/cby/drake_learning/src/drake_learning2/src/priordata/u_planed_outsearch.txt", 'r', True)
list_x0_0 = []          
list_x0_1 = []          
list_x0_2 = []          
list_x0_3 = []          
list_x0_4 = []          
list_x0_5 = []          
list_x0_6 = []      
list_x0_7 = []         
list_x0_8 = []         
list_x0_9 = []          
list_x0_10 = []          
list_x0_11 = []          
list_x0_12 = []          
list_x0_13 = []          
list_x0_14 = []          
list_x0_15 = []          
list_x0_16 = []          
list_x0_17 = []          
list_x0_18 = []      
list_x0_19 = []         
list_x0_20 = []         
list_x0_21 = []          
list_x0_22 = []          
list_x0_23 = []       
list_x0_24 = []         
list_x0_25 = []         
list_x0_26 = []          
list_x0_27 = []          
list_x0_28 = []   
list_u0 = []          
list_u1 = []          
list_u2 = []          
list_u3 = []        
list_u4 = []          
list_u5 = []        
list_u6 = []          
list_u7 = []        

for num in range(0,38):
	list_x0_0.append(round(float(f0.readline().rstrip()),3))#x
	list_x0_1.append(round(float(f0.readline().rstrip()),3))#z
	list_x0_2.append(round(float(f0.readline().rstrip()),3))#rh
	list_x0_3.append(round(float(f0.readline().rstrip()),3))#lh
	list_x0_4.append(round(float(f0.readline().rstrip()),3))#rk
	list_x0_5.append(round(float(f0.readline().rstrip()),3))#lk
	list_x0_6.append(round(float(f0.readline().rstrip()),3))#xdot
	list_x0_7.append(round(float(f0.readline().rstrip()),3))#zdot
	list_x0_8.append(round(float(f0.readline().rstrip()),3))#rhdot
	list_x0_9.append(round(float(f0.readline().rstrip()),3))#lhdot
	list_x0_10.append(round(float(f0.readline().rstrip()),3))#rkdot
	list_x0_11.append(round(float(f0.readline().rstrip()),3))#lkdot
#	list_x0_12.append(round(float(f0.readline().rstrip()),3))
#	list_x0_13.append(round(float(f0.readline().rstrip()),3))
#	list_x0_14.append(round(float(f0.readline().rstrip()),3))
#	list_x0_15.append(round(float(f0.readline().rstrip()),3))
#	list_x0_16.append(round(float(f0.readline().rstrip()),3))
#	list_x0_17.append(round(float(f0.readline().rstrip()),3))
#	list_x0_18.append(round(float(f0.readline().rstrip()),3))
#	list_x0_19.append(round(float(f0.readline().rstrip()),3))
#	list_x0_20.append(round(float(f0.readline().rstrip()),3))
#	list_x0_21.append(round(float(f0.readline().rstrip()),3))
#	list_x0_22.append(round(float(f0.readline().rstrip()),3))
#	list_x0_23.append(round(float(f0.readline().rstrip()),3))
#	list_x0_24.append(round(float(f0.readline().rstrip()),3))
#	list_x0_25.append(round(float(f0.readline().rstrip()),3))
#	list_x0_26.append(round(float(f0.readline().rstrip()),3))
#	list_x0_27.append(round(float(f0.readline().rstrip()),3))
#	list_x0_28.append(round(float(f0.readline().rstrip()),3))
	list_u0.append(round(float(fu0.readline().rstrip()),3))
	list_u1.append(round(float(fu0.readline().rstrip()),3))
	list_u2.append(round(float(fu0.readline().rstrip()),3))
	list_u3.append(round(float(fu0.readline().rstrip()),3))
#	list_u4.append(round(float(fu0.readline().rstrip()),3))
#	list_u5.append(round(float(fu0.readline().rstrip()),3))
#	list_u6.append(round(float(fu0.readline().rstrip()),3))
#	list_u7.append(round(float(fu0.readline().rstrip()),3))

#关节位置
fig=plt.figure(figsize=(20, 12))#
times = np.linspace(0, 37, 38)	
plt.title('驱动关节完整步态位置曲线(左右髋关节hip::pitch)', fontproperties=font, fontsize=18) 
plt.xlabel("时间(timestep)",fontproperties=font,fontsize=15)
plt.ylabel("位置(rad)",fontproperties=font,fontsize=15)
my_x_ticks = np.arange(0, 40, 2)#1.0hz
plt.xticks(my_x_ticks)
plt.plot(times,list_x0_2,label='右髋关节pitch',color='g',linewidth=2,linestyle='--')
plt.plot(times,list_x0_3,label='左髋关节pitch',color='b',linewidth=2,linestyle='--')
plt.grid()#添加网格
plt.legend(prop=font)#loc='lower right'loc='lower left', 
plt.savefig("2d_hip.jpeg",dpi = 600)
plt.show()

fig=plt.figure(figsize=(20, 12))#
times = np.linspace(0, 37, 38)	
plt.title('驱动关节完整步态位置曲线(左右膝关节knee)', fontproperties=font, fontsize=18) 
plt.xlabel("时间(timestep)",fontproperties=font,fontsize=15)
plt.ylabel("位置(rad)",fontproperties=font,fontsize=15)
my_x_ticks = np.arange(0, 40, 2)#1.0hz
plt.xticks(my_x_ticks)
plt.plot(times,list_x0_4,label='右膝关节knee',color='g',linewidth=2,linestyle='--')
plt.plot(times,list_x0_5,label='左膝关节knee',color='b',linewidth=2,linestyle='--')
plt.grid()#添加网格
plt.legend(prop=font)#loc='lower right'loc='lower left', 
plt.savefig("2d_knee.jpeg",dpi = 600)
plt.show()

fig=plt.figure(figsize=(20, 12))#
times = np.linspace(0, 37, 38)	
plt.title('基座_x', fontproperties=font, fontsize=18) 
plt.xlabel("时间(timestep)",fontproperties=font,fontsize=15)
plt.ylabel("位置(m)",fontproperties=font,fontsize=15)
my_x_ticks = np.arange(0, 40, 2)#1.0hz
plt.xticks(my_x_ticks)
plt.plot(times,list_x0_0,label='pelvis_x',color='g',linewidth=2,linestyle='--')
plt.grid()#添加网格
plt.legend(prop=font)#loc='lower right'loc='lower left', 
plt.savefig("2d_pevel_x.jpeg",dpi = 600)
plt.show()

fig=plt.figure(figsize=(20, 12))#
times = np.linspace(0, 37, 38)	
plt.title('基座_z', fontproperties=font, fontsize=18) 
plt.xlabel("时间(timestep)",fontproperties=font,fontsize=15)
plt.ylabel("位置(m)",fontproperties=font,fontsize=15)
my_x_ticks = np.arange(0, 40, 2)#1.0hz
plt.xticks(my_x_ticks)
plt.plot(times,list_x0_1,label='pelvis_z',color='g',linewidth=2,linestyle='--')
plt.grid()#添加网格
plt.legend(prop=font)#loc='lower right'loc='lower left', 
plt.savefig("2d_pevel_z.jpeg",dpi = 600)
plt.show()


#关节速度
fig=plt.figure(figsize=(20, 12))#
times = np.linspace(0, 37, 38)	
plt.title('驱动关节完整步态速度曲线(左右髋关节hip::pitchdot)', fontproperties=font, fontsize=18) 
plt.xlabel("时间(timestep)",fontproperties=font,fontsize=15)
plt.ylabel("速度(rad/s)",fontproperties=font,fontsize=15)
my_x_ticks = np.arange(0, 40, 2)#1.0hz
plt.xticks(my_x_ticks)
plt.plot(times,list_x0_8,label='右髋关节pitchdot',color='g',linewidth=2,linestyle='--')
plt.plot(times,list_x0_9,label='左髋关节pitchdot',color='b',linewidth=2,linestyle='--')
plt.grid()#添加网格
plt.legend(prop=font)#loc='lower right'loc='lower left', 
plt.savefig("2d_hipdot.jpeg",dpi = 600)
plt.show()

fig=plt.figure(figsize=(20, 12))#
times = np.linspace(0, 37, 38)	
plt.title('驱动关节完整步态速度曲线(左右膝关节kneedot)', fontproperties=font, fontsize=18) 
plt.xlabel("时间(timestep)",fontproperties=font,fontsize=15)
plt.ylabel("速度(rad/s)",fontproperties=font,fontsize=15)
my_x_ticks = np.arange(0, 40, 2)#1.0hz
plt.xticks(my_x_ticks)
plt.plot(times,list_x0_10,label='右髋关节kneedot',color='g',linewidth=2,linestyle='--')
plt.plot(times,list_x0_11,label='左髋关节kneedot',color='b',linewidth=2,linestyle='--')
plt.grid()#添加网格
plt.legend(prop=font)#loc='lower right'loc='lower left', 
plt.savefig("2d_kneedot.jpeg",dpi = 600)
plt.show()

#基座速度
fig=plt.figure(figsize=(20, 12))#
times = np.linspace(0, 37, 38)	
plt.title('基座_xdot', fontproperties=font, fontsize=18) 
plt.xlabel("时间(timestep)",fontproperties=font,fontsize=15)
plt.ylabel("速度(m/s)",fontproperties=font,fontsize=15)
my_x_ticks = np.arange(0, 40, 2)#1.0hz
plt.xticks(my_x_ticks)
plt.plot(times,list_x0_6,label='pelvis_xdot',color='g',linewidth=2,linestyle='--')
plt.grid()#添加网格
plt.legend(prop=font)#loc='lower right'loc='lower left', 
plt.savefig("2d_pevel_xdot.jpeg",dpi = 600)
plt.show()

fig=plt.figure(figsize=(20, 12))#
times = np.linspace(0, 37, 38)	
plt.title('基座_zdot', fontproperties=font, fontsize=18) 
plt.xlabel("时间(timestep)",fontproperties=font,fontsize=15)
plt.ylabel("速度(m/s)",fontproperties=font,fontsize=15)
my_x_ticks = np.arange(0, 40, 2)#1.0hz
plt.xticks(my_x_ticks)
plt.plot(times,list_x0_7,label='pelvis_zdot',color='g',linewidth=2,linestyle='--')
plt.grid()#添加网格
plt.legend(prop=font)#loc='lower right'loc='lower left', 
plt.savefig("2d_pevel_zdot.jpeg",dpi = 600)
plt.show()

#基座速度


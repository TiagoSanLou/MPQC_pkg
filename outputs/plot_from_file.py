import matplotlib.pyplot as plt
import numpy as np

#x, y = np.loadtxt('time_vec_c.txt', delimiter=',', unpack=True)
#dt = np.loadtxt('time_vec_c.txt')
#roll = np.loadtxt('roll.txt')

with open('sim_params.txt') as f:
	lines = f.readlines()
	N = [line.split()[0] for line in lines]

with open('time_vec_c.txt') as f:
    lines = f.readlines()
    i  = [line.split()[0] for line in lines]
    dt = [line.split()[1] for line in lines]
    t  = [line.split()[2] for line in lines]

    
with open('attitude.txt') as f:
    lines = f.readlines()
    roll       = [line.split()[0] for line in lines]
    roll_rate  = [line.split()[1] for line in lines]
    pitch      = [line.split()[2] for line in lines]
    pitch_rate = [line.split()[3] for line in lines]
    yaw        = [line.split()[4] for line in lines]
    yaw_rate   = [line.split()[5] for line in lines]
    
with open('attitude_sp.txt') as f:
    lines = f.readlines()
    roll_sp       = [line.split()[0] for line in lines]
    roll_rate_sp  = [line.split()[1] for line in lines]
    pitch_sp      = [line.split()[2] for line in lines]
    pitch_rate_sp = [line.split()[3] for line in lines]
    yaw_sp        = [line.split()[4] for line in lines]
    yaw_rate_sp   = [line.split()[5] for line in lines]
    
with open('commands.txt') as f:
    lines = f.readlines()
    roll_torqueCmd   = [line.split()[0] for line in lines]
    pitch_torqueCmd  = [line.split()[1] for line in lines]
    yaw_torqueCmd    = [line.split()[2] for line in lines]
    
with open('prediction.txt') as f:
    lines = f.readlines()
    roll_pred       = [line.split()[0] for line in lines]
    roll_rate_pred  = [line.split()[1] for line in lines]
    pitch_pred      = [line.split()[2] for line in lines]
    pitch_rate_pred = [line.split()[3] for line in lines]
    yaw_pred        = [line.split()[4] for line in lines]
    yaw_rate_pred   = [line.split()[5] for line in lines]

## Plot Sampling time 
plt.plot(t,dt, label='Sampling time')
plt.xlabel('t [s]')
plt.ylabel('[ms]')
title = 'Sampling time \n' + 'N = ' + str(N)	
plt.title(title)
plt.legend()
filename = 'exps/dt_exp_N_' + str(N) + '.png'
#plt.savefig(filename)
plt.show()

## Plot Commands
plt.plot(t, roll_torqueCmd,  label='Roll')
plt.plot(t, pitch_torqueCmd, label='Pitch')
plt.plot(t, yaw_torqueCmd,   label='Yaw')

plt.xlabel('t [s]')
plt.ylabel('torques')
title = 'Torque Commands (Roll, pitch, yaw) \n' + 'N = ' + str(N)	
plt.title(title)
plt.legend()
#filename = 'exps/att_exp_N_' + str(N) + '.png'
#plt.savefig(filename)
plt.show()

## Plot attitude
plt.plot(t, roll,  label='Roll')
plt.plot(t, pitch, label='Pitch')
plt.plot(t, yaw,   label='Yaw')

plt.xlabel('t [s]')
plt.ylabel('degrees')
title = 'Attitude (Roll, pitch, yaw) \n' + 'N = ' + str(N)	
plt.title(title)
plt.legend()
filename = 'exps/att_exp_N_' + str(N) + '.png'
#plt.savefig(filename)
plt.show()

## Plot roll 
plt.plot(t, roll,  label='Roll')
plt.plot(t, roll_sp,   label='Roll Setpoint')

plt.xlabel('t [s]')
plt.ylabel('degrees')
title = 'Roll Attitude \n' + 'N = ' + str(N)	
plt.title(title)
plt.legend()
#filename = 'exps/att_exp_N_' + str(N) + '.png'
#plt.savefig(filename)
plt.show()

plt.plot(t, roll_rate,  label='Roll velocity')
plt.plot(t, roll_rate_sp,   label='Roll velocity Setpoint')

plt.xlabel('t [s]')
plt.ylabel('rad/s')
title = 'Roll Velocity \n' + 'N = ' + str(N)	
plt.title(title)
plt.legend()
#filename = 'exps/att_exp_N_' + str(N) + '.png'
#plt.savefig(filename)
plt.show()

## Plot pitch prediction
plt.plot(t, pitch,  label='Pitch')
plt.plot(t, pitch_sp,   label='Pitch Setpoin')

plt.xlabel('t [s]')
plt.ylabel('degrees')
title = 'pitch Attitude \n' + 'N = ' + str(N)	
plt.title(title)
plt.legend()
#filename = 'exps/att_exp_N_' + str(N) + '.png'
#plt.savefig(filename)
plt.show()

plt.plot(t, pitch_rate,  label='Pitch velocity')
plt.plot(t, pitch_rate_sp,   label='Pitch velocity Setpoin')

plt.xlabel('t [s]')
plt.ylabel('rad/s')
title = 'Pitch Velocity \n' + 'N = ' + str(N)	
plt.title(title)
plt.legend()
#filename = 'exps/att_exp_N_' + str(N) + '.png'
#plt.savefig(filename)
plt.show()

## Plot yaw prediction
plt.plot(t, yaw,  label='yaw')
plt.plot(t, yaw_sp,   label='yaw Setpoin')

plt.xlabel('t [s]')
plt.ylabel('degrees')
title = 'yaw Attitude \n' + 'N = ' + str(N)	
plt.title(title)
plt.legend()
#filename = 'exps/att_exp_N_' + str(N) + '.png'
#plt.savefig(filename)
plt.show()

plt.plot(t, yaw_rate,  label='yaw velocity')
plt.plot(t, yaw_rate_sp,   label='yaw velocity Setpoin')

plt.xlabel('t [s]')
plt.ylabel('rad/s')
title = 'yaw Velocity \n' + 'N = ' + str(N)	
plt.title(title)
plt.legend()
#filename = 'exps/att_exp_N_' + str(N) + '.png'
#plt.savefig(filename)
plt.show()

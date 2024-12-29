import matplotlib.pyplot as plt
import numpy as np
import rospkg
import rospy

# Steer PID Parameter
P_GAIN_STEER = 0
I_GAIN_STEER = 0
D_GAIN_STEER = 0

# stanley Parameter

k_s = 1.3

# global_path_name
global_path_name = '09.21'
add_name = 'paldal_index+5_steervel_10_0.9*pi_t + np.arctan'

# File Name
LATERAL_OFFSET_FILE_NAME = 'lateral_offset/' + '_k_s_' + str(k_s) + '_Kp_' + str(P_GAIN_STEER) + '_' + add_name +'.csv'
HEADING_OFFSET_FILE_NAME = 'heading_offset/' + '_k_s_' + str(k_s) + '_Kp_' + str(P_GAIN_STEER) + '_' + add_name +'.csv'
POSITION_DATA_FILE_NAME = 'position_data/' + '_k_s_' + str(k_s) + '_Kp_' + str(P_GAIN_STEER) + '_' + add_name +'.csv'

# frameRate
frameRate = 30



lateral_offset_data = []
cal_steer = []
cur_steer = []
position_data_x = []
position_data_y = []
global_x_data = []
global_y_data = []

def read_file():
    with open(LATERAL_OFFSET_FILE_NAME, 'r') as later_f:
        for line in later_f:
            lateral_offset_data.append(float(line.strip()))
    
    with open(HEADING_OFFSET_FILE_NAME, 'r') as heading_f:
        for line in heading_f:
            x, y = line.strip().split(',')
            cal_steer.append(float(x))
            cur_steer.append(float(y))
    
    with open(POSITION_DATA_FILE_NAME, 'r') as position_f:
        for line in position_f:
            x, y = line.strip().split(',')
            position_data_x.append(float(x))
            position_data_y.append(float(y))

    pkg_name = 'gpsimu'
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path(pkg_name)
    file_path = pkg_path + '/path' + '/' + global_path_name + '.txt'
    # file_path = global_path_name + '.txt'

    with open(file_path, 'r') as f:
        lines = f.readlines()
        for line in lines:
            x, y, _ = line.strip().split('\t')
            global_x_data.append(float(x))
            global_y_data.append(float(y))

    #====================

def plot_data():
    if not lateral_offset_data or not cal_steer or not cur_steer or not position_data_x or not position_data_y:
        print("No data to plot.")
        return

    ti = 0
    tf = len(lateral_offset_data)
    t = np.arange(ti, tf) / frameRate

    plt.plot(t, lateral_offset_data, 'b--')
    plt.xlabel('time[s]')
    plt.ylabel('lateral_offset[m]')
    plt.title(f'K_s_{k_s}_Kp_{P_GAIN_STEER}')
    plt.axhline(y=0, linestyle='--')
    plt.grid(True)
    plt.savefig(f'LATERAL_OFFSET_Ks_{k_s}_Kp_{P_GAIN_STEER}_{add_name}.png')
    plt.show()

    plt.plot(t, cal_steer, 'b--', label='Cal Steer', linewidth=2)
    plt.plot(t, cur_steer, 'k--', label='Cur Steer', linewidth=2)
    plt.xlabel('time[s]')
    plt.ylabel('heading_offset[deg]')
    plt.title(f'K_s_{k_s}_Kp_{P_GAIN_STEER}')
    plt.axhline(y=0, linestyle='--')
    plt.grid(True)
    plt.legend()
    plt.savefig(f'HEADING_OFFSET_Ks_{k_s}_Kp_{P_GAIN_STEER}_{add_name}.png')
    plt.show()



    plt.plot(position_data_x, position_data_y, 'bo-', label='Position Data', markersize=1.5)
    plt.plot(global_x_data, global_y_data, 'k-', label='Global Path')
    plt.grid(True)
    plt.xlabel('Global_path_x')
    plt.ylabel('Global_path_y')
    plt.title(f'K_s_{k_s}_Kp_{P_GAIN_STEER}')
    plt.legend()
    plt.savefig(f'POSITION_DATA_Ks_{k_s}_Kp_{P_GAIN_STEER}_{add_name}.png')
    plt.show()

if __name__ == "__main__":
    read_file()
    plot_data()

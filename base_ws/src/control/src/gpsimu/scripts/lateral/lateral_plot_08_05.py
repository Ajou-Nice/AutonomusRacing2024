import matplotlib.pyplot as plt
import numpy as np
import rospkg
import rospy

# Steer PID Parameter
P_GAIN_STEER = 0.8
I_GAIN_STEER = 0.6
D_GAIN_STEER = 0.001

# Look distance Parameter
fixed_Lp = 2
kpp = 0.1
add_name = 'parking_1'

# File Name
LATERAL_OFFSET_FILE_NAME = 'lateral_offset/' + 'Lp_' + str(fixed_Lp) + '_Kpp_' + str(kpp) + '_Kp_' + str(P_GAIN_STEER) + '_' + add_name +'.csv'
HEADING_OFFSET_FILE_NAME = 'heading_offset/' + 'Lp_' + str(fixed_Lp) + '_Kpp_' + str(kpp) + '_Kp_' + str(P_GAIN_STEER) + '_' + add_name +'.csv'
POSITION_DATA_FILE_NAME = 'position_data/' + 'Lp_' + str(fixed_Lp) + '_Kpp_' + str(kpp) + '_Kp_' + str(P_GAIN_STEER) + '_' + add_name +'.csv'
LP_DATA_FILE_NAME = 'Lp_data/' + 'Lp_' + str(fixed_Lp) + '_Kpp_' + str(kpp) + '_Kp_' + str(P_GAIN_STEER) + '_' + add_name +'.csv'
WAY_DATA_FILE_NAME = 'Way_data/' + 'Lp_' + str(fixed_Lp) + '_Kpp_' + str(kpp) + '_Kp_' + str(P_GAIN_STEER) + '_' + add_name +'.csv'

# frameRate
frameRate = 30

# global_path_name
global_path_name = 'parking_test1'

lateral_offset_data = []
cal_steer = []
cur_steer = []
position_data_x = []
position_data_y = []
global_x_data = []
global_y_data = []
lp_data_x = []
lp_data_y = []
way_data_x = []
way_data_y = []

def read_file():
    with open(LATERAL_OFFSET_FILE_NAME, 'r') as later_f:
        for line in later_f:
            lateral_offset_data.append(float(line.strip()))
    
    with open(HEADING_OFFSET_FILE_NAME, 'r') as heading_f:

        lines = heading_f.readlines()
        for line in lines:
            pure, cur = line.strip().split(',')
            cal_steer.append(float(pure))
            cur_steer.append(float(cur))


    with open(POSITION_DATA_FILE_NAME, 'r') as position_f:
        for line in position_f:
            x, y = line.strip().split(',')
            position_data_x.append(float(x))
            position_data_y.append(float(y))

    with open(LP_DATA_FILE_NAME, 'r') as lp_f:
        for line in lp_f:
            x, y = line.strip().split(',')
            lp_data_x.append(float(x))
            lp_data_y.append(float(y))
    
    with open(WAY_DATA_FILE_NAME, 'r') as way_f:
        for line in way_f:
            x, y = line.strip().split(',')
            way_data_x.append(float(x))
            way_data_y.append(float(y))


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
    plt.title(f'Lp_{fixed_Lp}_Kpp_{kpp}_Kp_{P_GAIN_STEER}')
    plt.axhline(y=0, linestyle='--')
    plt.grid(True)
    plt.savefig(f'LATERAL_OFFSET_Name_{add_name}_Lp_{fixed_Lp}_Kpp_{kpp}_Kp_{P_GAIN_STEER}.png')
    plt.show()

    plt.plot(t, cal_steer, 'b--', label='pure_pursuit',linewidth = 3.0)
    plt.plot(t, cur_steer, 'k--', label='current',linewidth = 3.0)
    plt.xlabel('time[s]')
    plt.ylabel('heading_offset[deg]')
    plt.title(f'Lp_{fixed_Lp}_Kpp_{kpp}_Kp_{P_GAIN_STEER}')
    plt.axhline(y=0)
    plt.grid(True)
    plt.legend()
    plt.savefig(f'HEADING_OFFSET_Name_{add_name}_Lp_{fixed_Lp}_Kpp_{kpp}_Kp_{P_GAIN_STEER}.png')
    plt.show()

    plt.plot(position_data_x, position_data_y, 'bo-', label='Position Data', markersize=1.5)
    plt.plot(global_x_data, global_y_data, 'k-', label='Global Path')
    plt.plot(lp_data_x, lp_data_y, 'ro-', label='Lp Data', markersize=1.5)
    plt.plot(lp_data_x[0], lp_data_y[0],'bo', markersize=2)
    plt.plot(lp_data_x[-1], lp_data_y[-1], 'go', markersize=2)
    # plt.plot(way_data_x[0], way_data_y[0],'ro', markersize=4)
    # plt.plot(way_data_x[-1], way_data_y[-1], 'bo', markersize=7)
    plt.plot(way_data_x, way_data_y, 'go', label='Way Data', markersize=1.5)
    plt.grid(True)
    plt.xlabel('Global_path_x')
    plt.ylabel('Global_path_y')
    plt.title(f'Lp_{fixed_Lp}_Kpp_{kpp}_Kp_{P_GAIN_STEER}')
    plt.legend()
    plt.savefig(f'POSITION_DATA_Name_{add_name}_Lp_{fixed_Lp}_Kpp_{kpp}_Kp_{P_GAIN_STEER}.png')
    plt.show()

if __name__ == "__main__":
    read_file()
    plot_data()

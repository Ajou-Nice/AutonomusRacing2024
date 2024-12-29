import matplotlib.pyplot as plt
import rospkg
import rospy

def read_and_plot_path(file_path):
    
    # 데이터 읽기
    x_data = []
    y_data = []
    with open(file_path, 'r') as f:
        lines = f.readlines()
        for line in lines:
            x, y, _ = line.strip().split('\t')
            x_data.append(float(x))
            y_data.append(float(y))
    
    plt.figure(figsize=(10, 6))
    plt.plot(x_data, y_data, 'k-')
    plt.title('Path Plot')
    plt.xlabel('Global X Position')
    plt.ylabel('Global Y Position')
    plt.grid(True)
    plt.show()

# 파일 경로 지정
pkg_name = 'gpsimu'
path_name = '2024_07_17_1'
rospack = rospkg.RosPack()
pkg_path = rospack.get_path(pkg_name)
file_path = pkg_path + '/path' + '/' + path_name+'.txt'

read_and_plot_path(file_path)
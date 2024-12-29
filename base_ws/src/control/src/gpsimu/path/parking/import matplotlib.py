import matplotlib.pyplot as plt
import pandas as pd

# 각 파일을 읽어와서 DataFrame으로 저장합니다.
file_paths = [
    "kcity_final_parking_lot1_forward_fixed.txt",
    "kcity_final_parking_lot1_return_fixed.txt",
    "kcity_final_parking_lot1_reverse_fixed.txt",
    "kcity_final_parking_lot2_forward_fixed.txt",
    "kcity_final_parking_lot2_return_fixed.txt",
    "kcity_final_parking_lot2_reverse_fixed.txt",
    "kcity_final_parking_lot3_forward_fixed.txt",
    "kcity_final_parking_lot3_return_fixed.txt",
    "kcity_final_parking_lot3_reverse_fixed.txt"
]

# 각 파일의 경로를 읽어서 plot에 추가하는 함수
def plot_paths(file_paths):
    plt.figure(figsize=(10, 10))
    
    for path in file_paths:
        # 파일을 읽고 각 열에 이름을 부여합니다.
        df = pd.read_csv(path, sep="\t", header=None, names=["Easting", "Northing", "Elevation"])
        
        # Easting과 Northing 컬럼을 이용하여 경로를 그립니다.
        plt.plot(df["Easting"], df["Northing"], label=path.split("/")[-1].replace(".txt", ""))
    
    # 그래프 속성 설정
    plt.xlabel("Easting (m)")
    plt.ylabel("Northing (m)")
    plt.title("UTM Coordinate Paths")
    plt.legend(loc="best")
    plt.grid(True)
    plt.show()

# 경로 시각화 함수 호출
plot_paths(file_paths)
import numpy as np

def data_log_to_matrix(simultion_path,robot_num):
    swarm_state = []
    for robot in range(1,robot_num+1):
        log_file = f"{simultion_path}/{robot}/monitor.log"
        # 读取日志文件
        with open(log_file, 'r') as file:
            log_lines = file.readlines()

        # 提取 x, y, direction 数据
        robot_state = []
        for index,line in enumerate(log_lines):
            if index > 10:  # 确保行包含 x, y, direction 数据
                parts = line.split()
                robot_state.append([float(parts[3]), float(parts[4]), float(parts[5])])
        swarm_state.append(robot_state)
        # 转换为 numpy 矩阵
    return np.array(swarm_state)


  
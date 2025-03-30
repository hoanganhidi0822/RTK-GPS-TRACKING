import matplotlib.pyplot as plt
import csv
import numpy as np

def read_csv(filename):
    """ Đọc dữ liệu từ file CSV và trả về danh sách các điểm """
    optimal_x, optimal_y, history_x, history_y = [], [], [], []
    with open(filename, mode='r') as file:
        reader = csv.reader(file)
        next(reader)  # Bỏ qua dòng tiêu đề
        for row in reader:
            optimal_x.append(float(row[0]))
            optimal_y.append(float(row[1]))
            history_x.append(float(row[2]))
            history_y.append(float(row[3]))
    return optimal_x, optimal_y, history_x, history_y

def compute_travel_distance(x, y):
    """ Tính toán quãng đường đã di chuyển dựa trên tích lũy khoảng cách Euclidean """
    distances = [0.0]
    for i in range(1, len(x)):
        dx = x[i] - x[i-1]
        dy = y[i] - y[i-1]
        distances.append(distances[-1] + np.hypot(dx, dy))
    return distances

def plot_paths(optimal_x, optimal_y, history_x, history_y):
    """ Hiển thị đồ thị so sánh lateral displacement theo quãng đường đã di chuyển """
    fig, axs = plt.subplots(2, 1, figsize=(10, 10))
    
    # Vẽ quỹ đạo XY
    axs[0].plot(optimal_x, optimal_y, 'g-', label='Optimal Path (Reference)')
    axs[0].plot(history_x, history_y, 'r-', label='Actual Path (Driven)')
    axs[0].scatter(optimal_x[0], optimal_y[0], color='blue', marker='o', label='Start Point')
    axs[0].scatter(optimal_x[-1], optimal_y[-1], color='black', marker='x', label='End Point')
    axs[0].set_xlabel('X Position (m)')
    axs[0].set_ylabel('Y Position (m)')
    axs[0].set_title('Comparison of Optimal Path vs Actual Path')
    axs[0].legend()
    axs[0].grid()
    
    # Tính toán quãng đường đã di chuyển
    optimal_s = compute_travel_distance(optimal_x, optimal_y)
    history_s = compute_travel_distance(history_x, history_y)
    
    # Tính lateral displacement (sai số ngang)
    lateral_displacement = [hy - oy for oy, hy in zip(optimal_y, history_y)]
    
    axs[1].plot(history_s, lateral_displacement, 'b-', label='Lateral Displacement')
    axs[1].set_xlabel('Traveled Distance (m)')
    axs[1].set_ylabel('Lateral Displacement (m)')
    axs[1].set_title('Lateral Displacement vs Traveled Distance')
    axs[1].legend()
    axs[1].grid()
    
    plt.show()

if __name__ == "__main__":
    filename = "trajectory_data.csv"
    optimal_x, optimal_y, history_x, history_y = read_csv(filename)
    plot_paths(optimal_x, optimal_y, history_x, history_y)

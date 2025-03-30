"""

Frenet optimal trajectory generator

author: Atsushi Sakai (@Atsushi_twi)

Ref:

- [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame](https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)

- [Optimal trajectory generation for dynamic street scenarios in a Frenet Frame](https://www.youtube.com/watch?v=Cj6tAQe7UCY)

"""

import numpy as np
import matplotlib.pyplot as plt
import copy
import math
import cubic_spline_planner
import matplotlib.patches as patches
import time
import keyboard
import csv
# Parameter
MAX_SPEED = 20.0 / 3.6  # maximum speed [m/s]
MAX_ACCEL = 2  # maximum acceleration [m/ss]
MAX_CURVATURE = 10  # maximum curvature (độ cong) [1/m]
MAX_ROAD_WIDTH = 6 # maximum road width [m]
D_ROAD_W = 0.4 # road width sampling length [m]
DT = 0.3  # time tick [s]
MAXT = 5.0  # max prediction time [m]
MINT = 4.0  # min prediction time [m]
TARGET_SPEED = 16 / 3.6  # target speed [m/s]
D_T_S = 0.1 / 3.6  # target speed sampling length [m/s]
N_S_SAMPLE = 1  # sampling number of target speed
ROBOT_RADIUS = 2.0 # robot radius [m]

# cost weights
KJ = 0.1
KT = 0.1
KD = 1.0
KLAT = 2.0
KLON = 1.0

# Hàm lưu dữ liệu vào file CSV
def save_to_csv(filename, optimal_x, optimal_y, hist_x, hist_y):
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Optimal_X", "Optimal_Y", "History_X", "History_Y"])  # Tiêu đề cột
        for ox, oy, hx, hy in zip(optimal_x, optimal_y, hist_x, hist_y):
            writer.writerow([ox, oy, hx, hy])
    print(f"Data saved to {filename}")

show_animation = True

class quintic_polynomial:

    def __init__(self, xs, vxs, axs, xe, vxe, axe, T):

        # calc coefficient of quintic polynomial
        self.xs = xs
        self.vxs = vxs
        self.axs = axs
        self.xe = xe
        self.vxe = vxe
        self.axe = axe

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[T**3, T**4, T**5],
                      [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
                      [6 * T, 12 * T ** 2, 20 * T ** 3]])
        b = np.array([xe - self.a0 - self.a1 * T - self.a2 * T**2,
                      vxe - self.a1 - 2 * self.a2 * T,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t**2 + \
            self.a3 * t**3 + self.a4 * t**4 + self.a5 * t**5

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
            3 * self.a3 * t**2 + 4 * self.a4 * t**3 + 5 * self.a5 * t**4

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t**2 + 20 * self.a5 * t**3

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t**2

        return xt


class quartic_polynomial:

    def __init__(self, xs, vxs, axs, vxe, axe, T):

        # calc coefficient of quintic polynomial
        self.xs = xs   # Vị trí ban đầu.
        self.vxs = vxs # Vận tốc ban đầu.
        self.axs = axs # Gia tốc ban đầu.
        self.vxe = vxe # Vận tốc tại thời điểm kết thúc.
        self.axe = axe # Gia tốc tại thời điểm kết thúc.
                       # T: Thời gian di chuyển từ điểm đầu đến điểm cuối.

        self.a0 = xs        # Giá trị tại thời điểm ban đầu.
        self.a1 = vxs       # Hệ số của vận tốc ban đầu.
        self.a2 = axs / 2.0 # Hệ số gia tốc ban đầu (gia tốc chia 2 để phù hợp với công thức đa thức).

        A = np.array([[3 * T ** 2,  4 * T ** 3],
                      [  6 * T   , 12 * T ** 2]])       # Ma trận A là ma trận hệ số của phương trình
        
        b = np.array([vxe - self.a1 - 2 * self.a2 * T,
                             axe - 2 * self.a2        ]) # Vector b chứa các giá trị đích
        
        x = np.linalg.solve(A, b) # np.linalg.solve(A, b) giải hệ phương trình để tìm giá trị của a3 và a4.

        self.a3 = x[0]
        self.a4 = x[1]

    def calc_point(self, t):
        """Tính giá trị vị trí x(t) của đa thức tại thời điểm t."""
        
        xt = self.a0 + self.a1 * t + self.a2 * t**2 + \
            self.a3 * t**3 + self.a4 * t**4

        return xt

    def calc_first_derivative(self, t):
        """Tính vận tốc v(t), là đạo hàm bậc nhất của x(t)"""
        xt = self.a1 + 2 * self.a2 * t + \
            3 * self.a3 * t**2 + 4 * self.a4 * t**3

        return xt

    def calc_second_derivative(self, t):
        """Tính gia tốc a(t), là đạo hàm bậc hai của x(t)"""
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t**2

        return xt

    def calc_third_derivative(self, t):
        """Tính giật j(t) (jerk), là đạo hàm bậc ba của x(t)"""
        xt = 6 * self.a3 + 24 * self.a4 * t

        return xt


class Frenet_path:

    def __init__(self):
        self.t = []     # Danh sách thời gian tương ứng với các điểm trên quỹ đạo.
        self.d = []     # Danh sách giá trị độ lệch ngang (lateral offset) tại mỗi thời điểm t. 
                        # Giá trị này đại diện cho khoảng cách từ quỹ đạo tham chiếu đến quỹ đạo thực tế.
        self.d_d = []   # Vận tốc lệch ngang (lateral velocity) tại mỗi điểm trên quỹ đạo.
        self.d_dd = []  # Gia tốc lệch ngang (lateral acceleration).
        self.d_ddd = [] # Giật lệch ngang (lateral jerk), tức là đạo hàm bậc ba của d theo thời gian.
        
        self.s = []     # Danh sách giá trị dọc theo quỹ đạo tham chiếu tại mỗi thời điểm t. Đây là khoảng cách tích lũy từ điểm đầu quỹ đạo.
        self.s_d = []   # Vận tốc dọc theo quỹ đạo (longitudinal velocity)
        self.s_dd = []  # Gia tốc dọc (longitudinal acceleration)
        self.s_ddd = [] # Giật dọc (longitudinal jerk)
        
        #Cost
        self.cd = 0.0 # Cost liên quan đến chuyển động ngang (lateral motion cost)
        self.cv = 0.0 # Cost liên quan đến chuyển động dọc (longitudinal motion cost)
        self.cf = 0.0 # Tổng chi phí (total cost), được tính dựa trên cd và cv. 
                      # Giá trị này thường được sử dụng để so sánh và chọn quỹ đạo tối ưu

        # Các thông số trong không gian Cartesian (kết quả chuyển đổi từ Frenet)
        self.x = []   # Danh sách tọa độ x trong không gian Cartesian
        self.y = []   # Danh sách tọa độ y trong không gian Cartesian
        self.yaw = [] # Góc phương vị (yaw angle) tại mỗi điểm trong không gian Cartesian
        self.ds = []  # Khoảng cách giữa các điểm liên tiếp trên quỹ đạo trong không gian Cartesian
        self.c = []   # Độ cong (curvature) tại mỗi điểm trên quỹ đạo trong không gian Cartesian


def calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0):
    frenet_paths = []

    # Create an array of lateral offsets and time durations
    di_values = np.arange(-MAX_ROAD_WIDTH/4, MAX_ROAD_WIDTH*0.75, D_ROAD_W) # Do xe chạy bên làn phải
    Ti_values = np.arange(MINT, MAXT, DT)
    
    # Loop through each lateral offset (di) and time duration (Ti)
    for di in di_values:
        for Ti in Ti_values:
            # Create lateral trajectory using quintic polynomial (for all time steps)
            lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)
            t_values = np.arange(0.0, Ti, DT)

            # Calculate lateral parameters (fp.d, fp.d_d, fp.d_dd, fp.d_ddd) using vectorized operations
            fp_d = lat_qp.calc_point(t_values)
            fp_d_d = lat_qp.calc_first_derivative(t_values)
            fp_d_dd = lat_qp.calc_second_derivative(t_values)
            fp_d_ddd = lat_qp.calc_third_derivative(t_values)

            # Create longitudinal trajectories for each target speed tv
            tv_values = np.arange(TARGET_SPEED - D_T_S * N_S_SAMPLE, TARGET_SPEED + D_T_S * N_S_SAMPLE, D_T_S)

            # Use broadcasting to calculate the longitudinal trajectories in parallel
            for tv in tv_values:
                tfp = Frenet_path()

                # Generate longitudinal trajectory using quartic polynomial
                lon_qp = quartic_polynomial(s0, c_speed, 0.0, tv, 0.0, Ti)

                # Calculate longitudinal parameters (tfp.s, tfp.s_d, tfp.s_dd, tfp.s_ddd) using vectorized operations
                tfp_s = lon_qp.calc_point(t_values)
                tfp_s_d = lon_qp.calc_first_derivative(t_values)
                tfp_s_dd = lon_qp.calc_second_derivative(t_values)
                tfp_s_ddd = lon_qp.calc_third_derivative(t_values)

                # Compute costs (Jp, Js, ds)
                Jp = np.sum(np.power(fp_d_ddd, 2))  # Use fp_d_ddd for lateral jerk
                Js = np.sum(np.power(tfp_s_ddd, 2))  # Use tfp_s_ddd for longitudinal jerk
                ds = np.square(TARGET_SPEED - tfp_s_d[-1])

                # Calculate total costs (tfp.cd, tfp.cv, tfp.cf) using vectorized operations
                tfp_cd = KJ * Jp + KT * Ti + KD * np.square(fp_d[-1])  # Cost do độ lệch ngang cuối cùng.
                tfp_cv = KJ * Js + KT * Ti + KD * ds
                tfp_cf = KLAT * tfp_cd + KLON * tfp_cv

                # Assign the calculated values to tfp
                tfp.t = t_values
                tfp.d = fp_d
                tfp.d_d = fp_d_d
                tfp.d_dd = fp_d_dd
                tfp.d_ddd = fp_d_ddd
                tfp.s = tfp_s
                tfp.s_d = tfp_s_d
                tfp.s_dd = tfp_s_dd
                tfp.s_ddd = tfp_s_ddd
                tfp.cd = tfp_cd
                tfp.cv = tfp_cv
                tfp.cf = tfp_cf

                # Append the trajectory to the list
                frenet_paths.append(tfp)

    return frenet_paths


def calc_global_paths(fplist, csp):

    
    for fp in fplist:

        # calc global positions
        # Xử lý từng quỹ đạo Frenet để chuyển đổi sang hệ tọa độ toàn cục.
        
        for i in range(len(fp.s)):
            
            # Tính vị trí toàn cục (ix,iy) tại khoảng cách s[i] dọc theo đường tham chiếu.
            
            ix, iy = csp.calc_position(fp.s[i])
            
            if ix is None:
                # Nếu không tính được vị trí (ví dụ: ngoài phạm vi đường tham chiếu), thoát khỏi vòng lặp.
                break
            
            # Tính góc hướng (yaw) tại khoảng cách s[i]
            iyaw = csp.calc_yaw(fp.s[i])
            
            # Sử dụng độ lệch ngang di để tính vị trí toàn cục (fx,fy)
            di = fp.d[i]
            fx = ix + di * math.cos(iyaw + math.pi / 2.0)
            fy = iy + di * math.sin(iyaw + math.pi / 2.0)
            
            # Lưu kết quả vào
            fp.x.append(fx)
            fp.y.append(fy)

        # calc yaw and ds
        # Tính góc hướng và khoảng cách
        for i in range(len(fp.x) - 1):
            dx = fp.x[i + 1] - fp.x[i]
            dy = fp.y[i + 1] - fp.y[i]
            
            # Sử dụng atan2 để tính góc hướng giữa hai điểm liên tiếp
            # Sử dụng công thức Pythagoras để tính độ dài đoạn thẳng ds giữa hai điểm liên tiếp.
            fp.yaw.append(math.atan2(dy, dx))
            fp.ds.append(math.sqrt(dx**2 + dy**2))
        
        # Lưu kết quả vào
        fp.yaw.append(fp.yaw[-1])
        fp.ds.append(fp.ds[-1])

        # calc curvature
        # Tính độ cong (curvature)
        for i in range(len(fp.yaw) - 1):
            
            # Độ cong: Tính độ thay đổi của góc hướng (yaw) giữa hai điểm liên tiếp chia cho khoảng cách ds
            # Độ cong này biểu diễn mức độ uốn cong của quỹ đạo tại mỗi điểm
            fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])
    # - Danh sách các quỹ đạo với thông tin toàn cục đầy đủ bao gồm:
    #    + Vị trí toàn cục (x,y).
    #    + Góc hướng yaw.
    #    + Khoảng cách ds.
    #    + Độ cong c.
    return fplist



# def check_collision(fp, ob):
#     # fp:Một đối tượng thuộc lớp Frenet_path chứa thông tin quỹ đạo, bao gồm danh sách các điểm tọa độ toàn cục (x,y).
#     # ob:Mảng NumPy ob chứa tọa độ của các chướng ngại vật: Mỗi hàng đại diện cho một chướng ngại vật với (x,y).
    
#     # Duyệt qua từng chướng ngại vật
#     # Lặp qua tất cả các chướng ngại vật trong danh sách ob
#     for i in range(len(ob[:, 0])):
#         # Tính khoảng cách từ từng điểm trên quỹ đạo đến chướng ngại vật
#         # Với mỗi chướng ngại vật ob[i]=(xob,yob), tính khoảng cách bình phương từ nó đến từng điểm (x,y) trên quỹ đạo fp:
#         d = [((ix - ob[i, 0])**2 + (iy - ob[i, 1])**2)
#              for (ix, iy) in zip(fp.x, fp.y)]
#         # print(d)
#         # So sánh mỗi khoảng cách với bán kính va chạm
#         collision = any([di <= ROBOT_RADIUS**2 for di in d])

#         if collision:
#             return False

#     return True
def check_collision(fp, ob):
    """
    Kiểm tra va chạm nhưng chỉ xét 3/4 quỹ đạo phía xa nhất của xe.

    Parameters:
        fp: Đối tượng Frenet_path chứa danh sách tọa độ (x, y).
        ob: Mảng NumPy chứa danh sách tọa độ (x, y) của các chướng ngại vật.

    Returns:
        True nếu 3/4 quỹ đạo phía xa an toàn, False nếu có va chạm.
    """
    path_length = len(fp.x)
    three_fourths_index = path_length // 4  # Lấy 1/4 đầu bỏ đi, lấy 3/4 sau
    
    # Chỉ lấy 3/4 quỹ đạo phía xa
    x_far = fp.x[three_fourths_index:]  
    y_far = fp.y[three_fourths_index:]

    for i in range(ob.shape[0]):  # Duyệt qua từng chướng ngại vật
        x_ob, y_ob = ob[i]  # Tọa độ của vật cản

        # Kiểm tra va chạm chỉ với 3/4 quỹ đạo phía xa
        collision = any((ix - x_ob) ** 2 + (iy - y_ob) ** 2 <= ROBOT_RADIUS**2 
                        for ix, iy in zip(x_far, y_far))

        if collision:
            return False  # Có va chạm, quỹ đạo không an toàn

    return True  # Không có va chạm, quỹ đạo an toàn



def check_paths(fplist, ob):
    """Hàm check_paths được sử dụng để kiểm tra các quỹ đạo (paths)
    trong danh sách fplist dựa trên các ràng buộc động học và tránh va chạm. 
    Chỉ những quỹ đạo hợp lệ (thỏa mãn các điều kiện) mới được giữ lại."""

    # Danh sách các đối tượng thuộc lớp Frenet_path, 
    # mỗi đối tượng đại diện cho một quỹ đạo với các thuộc tính như vận tốc, gia tốc, độ cong, v.v.
    
    # ob: Mảng NumPy chứa tọa độ của các chướng ngại vật trong môi trường.
    
    # Mảng này lưu trữ chỉ số của các quỹ đạo hợp lệ trong fplist
    okind = []
    
    # Kiểm tra từng quỹ đạo
    for i in range(len(fplist)):
        
        # Kiểm tra điều kiện động học
        
        if any([v > MAX_SPEED for v in fplist[i].s_d]):  # Max speed check
            # Giới hạn vận tốc tối đa
            # print("1")
            continue
        # elif any([abs(a) > MAX_ACCEL for a in fplist[i].s_dd]):  # Max accel check
        #     # Giới hạn độ cong tối đa
        #     # print("2")
        #     continue
        # elif any([abs(c) > MAX_CURVATURE for c in fplist[i].c]):  # Max curvature check
        #     # Giới hạn độ cong tối đa
        #     # print("3")
        #     continue
        elif not check_collision(fplist[i], ob):
            # Nếu quỹ đạo va chạm với bất kỳ chướng ngại vật nào, bỏ qua quỹ đạo.
            # print("4")
            continue
        # Nếu bất kì điều kiện nào không thỏa thì bỏ qua path đó
        # Lưu các quỹ đạo hợp lệ thỏa mãn tất cả điều kiện trên, thêm chỉ số i vào danh sách okind.
        okind.append(i)
    # Trích xuất các quỹ đạo hợp lệ từ danh sách fplist dựa trên chỉ số trong okind
    return [fplist[i] for i in okind]


def frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob):
    """ Hàm frenet_optimal_planning thực hiện quá trình lập kế hoạch chuyển động tối ưu trong hệ tọa độ Frenet, 
        dựa trên các thông tin đầu vào về trạng thái xe, đường cong tham chiếu, và các chướng ngại vật. 
        Hàm trả về quỹ đạo tối ưu dựa trên tiêu chí chi phí tối thiểu. """

    # - Các tham số đầu vào
    #     + csp: Đường cong tham chiếu (Cubic Spline Path) trong hệ tọa độ toàn cục.
    #     + Dùng để tính toán vị trí và hướng toàn cục từ tọa độ Frenet.

    #     + s0:Vị trí dọc (longitudinal) ban đầu của xe trên đường tham chiếu.
    #     + c_speed:Vận tốc dọc ban đầu của xe.

    #     + c_d, c_d_d, c_d_dd:
    #     + Các trạng thái ngang (lateral state):
    #     + c_d: vị trí ngang ban đầu.
    #     + c_d_d: vận tốc ngang ban đầu.
    #     + c_d_dd: gia tốc ngang ban đầu.

    #     ob:Mảng NumPy chứa tọa độ của các chướng ngại vật.

    # Tính toán các quỹ đạo trong hệ tọa độ Frenet
    # Hàm calc_frenet_paths tạo ra một danh sách các quỹ đạo ứng viên trong hệ tọa độ Frenet, 
    # bao gồm cả chuyển động ngang (lateral) và dọc (longitudinal).
    fplist = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0)
    
    # Chuyển đổi quỹ đạo sang hệ tọa độ toàn cục
    # Sử dụng đường cong tham chiếu csp, hàm calc_global_paths tính toán vị trí toàn cục (x,y), góc lái (yaw), 
    # và độ cong cho từng quỹ đạo trong danh sách.
    fplist = calc_global_paths(fplist, csp)
    
    # Kiểm tra tính hợp lệ của quỹ đạo
    # Loại bỏ các quỹ đạo không hợp lệ (vi phạm giới hạn động học hoặc va chạm với chướng ngại vật) 
    # bằng cách sử dụng hàm check_paths.
    fplist = check_paths(fplist, ob)

    # find minimum cost path
    mincost = float("inf")
    bestpath = None
    for fp in fplist:
        if mincost >= fp.cf:
            mincost = fp.cf
            bestpath = fp
    # print(f"num path: {len(fplist)}")
    return bestpath, fplist




def generate_target_course(x, y):
    
    """Hàm generate_target_course tạo ra đường dẫn tham chiếu (target course) từ một tập hợp các điểm (x,y). 
        Đường dẫn này được biểu diễn dưới dạng spline 2D và bao gồm:
        các thuộc tính như tọa độ (x,y), góc định hướng (yaw), và độ cong (curvature)."""
    
    # cubic_spline_planner.Spline2D tạo spline hai chiều dựa trên các điểm x và y
    # Spline 2D cung cấp các hàm tính toán vị trí, góc yaw, và độ cong cho bất kỳ giá trị dọc s.
    csp = cubic_spline_planner.Spline2D(x, y)
    
    # Tạo danh sách các giá trị dọc s
    # Mỗi giá trị s đại diện cho một vị trí dọc (longitudinal position) dọc theo spline
    # Khoảng cách giữa các giá trị s là 0.1, đảm bảo độ phân giải cao.
    s = np.arange(0, csp.s[-1], 0.1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        # Tính tọa độ (x,y) tại vị trí s.
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        # Tính góc định hướng (yaw) tại s
        ryaw.append(csp.calc_yaw(i_s))
        # Tính độ cong (curvature) tại s
        rk.append(csp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, csp


def update_state(state, v, delta, dt, L):
    """
    Update the state of the vehicle using Ackerman kinematics.
    """
    state["x"] += v * math.cos(state["yaw"]) * dt
    state["y"] += v * math.sin(state["yaw"]) * dt
    state["yaw"] += v / L * math.tan(delta) * dt
    state["v"] = v
    return state





def generate_all_trajectories(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob):
    """
    Generates all possible trajectories for visualization purposes.

    :param csp: Cubic spline path
    :param s0: Current position along the spline
    :param c_speed: Current speed [m/s]
    :param c_d: Current lateral position [m]
    :param c_d_d: Current lateral speed [m/s]
    :param c_d_dd: Current lateral acceleration [m/s^2]
    :param ob: Obstacles
    :return: List of all candidate trajectories
    """
    trajectories = []  # To store all possible trajectories

    for speed in np.arange(5.0 / 3.6, 20.0 / 3.6, 2.0 / 3.6):  # Varying speeds
        for lateral_offset in np.arange(-2.0, 2.0, 0.5):  # Varying lateral offsets
            for lateral_speed in np.arange(-0.5, 0.5, 0.2):  # Varying lateral speeds
                traj = frenet_optimal_planning( csp, s0, c_speed + speed,
                                                c_d + lateral_offset,
                                                c_d_d + lateral_speed,
                                                c_d_dd, ob)
                
                if traj is not None:  # Add trajectory only if it is valid
                    trajectories.append(traj)

    return trajectories

# Hàm động học của xe
def update_vehicle_state(car_x, car_y, car_yaw, car_steer, c_speed, delta_t, L):
    """ Cập nhật vị trí và hướng của xe dựa trên động học """
    car_x += c_speed * np.cos(car_yaw) * delta_t
    car_y += c_speed * np.sin(car_yaw) * delta_t
    car_yaw += (c_speed / L) * np.tan(car_steer) * delta_t
    return car_x, car_y, car_yaw

def cartesian_to_frenet(x, y, yaw, csp):
    """
    Chuyển đổi tọa độ Cartesian sang Frenet dựa trên đường spline tham chiếu.

    Args:
        x (float): Tọa độ x của xe.
        y (float): Tọa độ y của xe.
        yaw (float): Góc phương vị của xe (radians).
        csp (CubicSpline2D): Đường spline tham chiếu.

    Returns:
        s (float): Vị trí dọc theo spline.
        d (float): Khoảng cách vuông góc đến spline.
        d_d (float): Tốc độ vuông góc.
        d_dd (float): Gia tốc vuông góc.
    """
    # Tạo danh sách giá trị s
    s_min = 0.0
    s_max = csp.s[-1]
    ds = 0.1  # Bước tìm kiếm
    s_values = np.arange(s_min, s_max, ds)

    # Tính toán vị trí (x, y) trên spline cho tất cả giá trị s
    positions = np.array([csp.calc_position(s) for s in s_values])
    x_spline, y_spline = positions[:, 0], positions[:, 1]

    # Tính khoảng cách từ điểm (x, y) đến tất cả các điểm trên spline
    distances = np.hypot(x_spline - x, y_spline - y)

    # Tìm giá trị s tương ứng với khoảng cách nhỏ nhất
    min_idx = np.argmin(distances)
    s_best = s_values[min_idx]

    # Tính toán các giá trị dựa trên s_best
    s = s_best
    x_ref, y_ref = csp.calc_position(s)
    yaw_ref = csp.calc_yaw(s)
    d = np.hypot(x - x_ref, y - y_ref)

    # Xác định hướng của d (trái hay phải spline)
    cross_product = (x - x_ref) * -np.sin(yaw_ref) + (y - y_ref) * np.cos(yaw_ref)
    if cross_product < 0:
        d *= -1

    # Tính tốc độ và gia tốc vuông góc
    dx = x - x_ref
    dy = y - y_ref
    v_ref = np.array([np.cos(yaw_ref), np.sin(yaw_ref)])
    v_cartesian = np.array([dx, dy])

    # Tốc độ vuông góc
    d_d = np.dot(v_cartesian, [-v_ref[1], v_ref[0]])

    # Gia tốc vuông góc (giả sử không có dữ liệu gia tốc thêm)
    d_dd = 0.0

    return s, d, d_d, d_dd


# Vehicle parameters
LENGTH = 2.0  # total vehicle length
WIDTH = 1.0   # total vehicle width
BACKTOWHEEL = 0.2  # distance from rear to vehicle center
WHEEL_LEN = 0.3  # wheel length
WHEEL_WIDTH = 0.2  # wheel width
TREAD = 0.6  # width between left and right wheels
WB = 1.8  # wheel base: distance between front and rear axles

def plot_car(x, y, yaw, steer=0.0, cabcolor="-r", truckcolor="-k"):  # pragma: no cover
    """
    Plots a car at the given position and orientation.
    :param x: X-coordinate of the car's rear axle center.
    :param y: Y-coordinate of the car's rear axle center.
    :param yaw: Orientation of the car in radians.
    :param steer: Steering angle in radians.
    :param cabcolor: Color of the car body.
    :param truckcolor: Color of the wheels.
    """

    # Define the car outline and wheel shapes
    outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                        [WIDTH / 2, WIDTH / 2, -WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

    fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                         [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    # Rotation matrices
    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                     [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                     [-math.sin(steer), math.cos(steer)]])

    # Transform front wheels based on steering angle
    fr_wheel = (fr_wheel.T.dot(Rot2)).T
    fl_wheel = (fl_wheel.T.dot(Rot2)).T
    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB

    # Apply vehicle yaw rotation
    fr_wheel = (fr_wheel.T.dot(Rot1)).T
    fl_wheel = (fl_wheel.T.dot(Rot1)).T
    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    # Translate to vehicle position
    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    # Plot car body and wheels
    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), cabcolor)
    plt.plot(np.array(fr_wheel[0, :]).flatten(),
             np.array(fr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rr_wheel[0, :]).flatten(),
             np.array(rr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fl_wheel[0, :]).flatten(),
             np.array(fl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rl_wheel[0, :]).flatten(),
             np.array(rl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(x, y, "*")  # Mark the vehicle's position


def project_onto_path(x, y, tx, ty):
    """
    Project the point (x, y) onto the closest segment of the target course (tx, ty).
    Returns the projected point (px, py).
    """
    min_dist = float('inf')
    closest_point = None
    
    # Loop over each segment of the target course
    for i in range(len(tx) - 1):
        # Calculate vector for the current segment
        segment_start = np.array([tx[i], ty[i]])
        segment_end = np.array([tx[i+1], ty[i+1]])
        segment_vec = segment_end - segment_start
        
        # Vector from the segment start to the current point
        point_vec = np.array([x, y]) - segment_start
        
        # Project the point onto the segment
        segment_length = np.linalg.norm(segment_vec)
        if segment_length == 0:
            continue
            
        projection = np.dot(point_vec, segment_vec) / segment_length
        projection = np.clip(projection, 0, segment_length)
        
        projected_point = segment_start + (projection / segment_length) * segment_vec
        dist_to_projected = np.linalg.norm(projected_point - np.array([x, y]))
        
        if dist_to_projected < min_dist:
            min_dist = dist_to_projected
            closest_point = projected_point

    return closest_point

def pure_pursuit_control_frenet(optimal_path, current_idx, x, y,yaw, lookahead_distance, WB):
    """
    Pure Pursuit controller for following the optimal trajectory by finding the
    lookahead point as the point on the path at the lookahead distance.
    """
    try:
        path_x = optimal_path.x
        path_y = optimal_path.y
    except:
        path_x = optimal_path.x
        path_y = optimal_path.y
    # Circle center is the car's current position
    car_pos = np.array([x, y])
    lookahead_point = None

    # Find the closest point on the path (we'll use a simple Euclidean distance)
    min_distance = float('inf')
    closest_idx = current_idx

    for idx in range(current_idx, len(path_x)):
        path_point = np.array([path_x[idx], path_y[idx]])
        distance_to_point = np.linalg.norm(car_pos - path_point)

        if distance_to_point < min_distance:
            min_distance = distance_to_point
            closest_idx = idx

    # Now find the lookahead point from the closest point
    lookahead_point = np.array([path_x[closest_idx], path_y[closest_idx]])

    # Calculate the vector from the car position to the lookahead point
    lookahead_vector = lookahead_point - car_pos
    lookahead_distance_actual = np.linalg.norm(lookahead_vector)

    if lookahead_distance_actual < lookahead_distance:
        # If the closest point is less than the lookahead distance, find the next point
        for idx in range(closest_idx + 1, len(path_x)):
            path_point = np.array([path_x[idx], path_y[idx]])
            distance_to_point = np.linalg.norm(car_pos - path_point)

            if distance_to_point >= lookahead_distance:
                lookahead_point = path_point
                break
 
    # Calculate the steering angle towards the lookahead point
    alpha = np.arctan2(lookahead_point[1] - y, lookahead_point[0] - x) - yaw
    steering_angle = np.arctan2(2.0 * WB * np.sin(alpha), lookahead_distance)

    return steering_angle, closest_idx, lookahead_point

def calculate_speed_at_projected_point(projected_point, nearest_idx, csp):
    """
    Calculate the speed at the projected point on the target course.
    """
    # Speed at the nearest point in the course (csp is the speed profile)
    speed_at_point = csp[nearest_idx]
    return speed_at_point

def plot_obstacle(x, y, yaw, width=1.0, length=2.0, color='blue'):
    """
    Vẽ obstacle hình chữ nhật với vị trí (x, y), góc yaw (radian),
    kích thước width x length.
    """
    yaw = yaw 
    # Tọa độ góc của hình chữ nhật trước khi xoay (tọa độ tương đối)
    corners = np.array([
        [-length / 2, -width / 2],
        [ length / 2, -width / 2],
        [ length / 2,  width / 2],
        [-length / 2,  width / 2]
    ])

    # Ma trận xoay
    rotation_matrix = np.array([
        [np.cos(yaw), -np.sin(yaw)],
        [np.sin(yaw), np.cos(yaw)]
    ])

    # Xoay và tịnh tiến hình chữ nhật
    rotated_corners = np.dot(corners, rotation_matrix.T) + np.array([x, y])

    # Vẽ hình chữ nhật sử dụng Polygon
    rect = patches.Polygon(rotated_corners, closed=True, color=color, alpha=0.5, edgecolor='black')
    plt.gca().add_patch(rect)
    
def slow_down_speed(distance, max_speed):
    return int(max_speed * (1 / (1 + math.exp(5 - 1 * distance))))  # 5 và 1 là các hệ số điều chỉnh

# Biến toàn cục cho figure, axes và lookahead_circle
fig, ax, lookahead_circle = None, None, None  

def main():
    print(__file__ + " start!!")

    # Waypoints
    wx = [ 16.993362287481073, 17.187923598713887, 17.49352551415194, 17.937076377633296, 18.317844743686386, 18.724997266552673, 19.07281820089718, 19.460870603078643, 19.899332470290926, 20.26753123284231, 20.53436503672848, 20.918944691528594, 21.30990197246108, 21.60512932270481, 21.22818316463616, 19.348563210588786, 16.13681638281528, 12.319708244035736, 8.344426216952932, 4.701861939487953, 1.1455703567508342, -3.0998107343616064, -7.6857287205863685, -12.02466654410695, -16.305168268312414, -20.626633139273324, -24.97120599077035, -29.15798717227951, -33.221169065319145, -36.91961402395132, -41.73395808545473, -46.69008422532714, -51.25835453257045, -55.404358212983375, -60.67700686209172, -66.46091406119442, -71.5797570570356, -75.85426337342408, -81.17714675648574, -87.60955161473437, -94.45877372215186, -100.89445475822947, -106.75244726997752, -112.23949994915941, -117.36089836470045, -122.2231073923242, -126.34090317824639, -128.85482401981284, -129.82747768492615, -130.38188350229782, -130.94465449687135, -131.52799988920412, -132.1734070223879, -132.51940421595108, -132.8699552979691, -133.5071719834766, -134.20117576014735, -134.99728709519107, -135.73971278892694, -136.51052110667908, -136.98065269652596, -137.44660170009982 ]
    wy = [ 152.51032455731195, 153.0803875877001, 154.65341773695621, 157.12138913379545, 160.34678701251474, 163.59515776300438, 166.44045802412867, 169.39452916221285, 172.32507145404503, 175.21966442588385, 178.6153684543191, 181.99774021093415, 185.3511901671062, 189.0469425855375, 193.1750542372629, 197.13766336010585, 200.59490266070458, 203.1434903795131, 204.88721586056306, 206.20932353852896, 207.1811671974217, 208.325173961064, 209.60855244603547, 210.85708244087843, 212.10450048636298, 213.3115214152105, 214.51483955289189, 215.74150862512548, 216.86791323200364, 217.73412171053477, 218.26340956130647, 218.8186391886356, 219.38665623240172, 219.9409629417633, 220.39816312155244, 221.0236345839291, 221.69655292148033, 222.31664254935714, 222.76531407835708, 223.31388312962162, 224.10484600118966, 224.83391777679958, 225.64581865320702, 226.37247749870903, 226.95607295180193, 227.09042978172207, 225.99293585559573, 223.05572186840294, 218.36588680568767, 212.91363260906544, 208.15040889501321, 203.41888685440506, 198.15213915384612, 192.8612955126648, 189.29749811373694, 185.7972708542985, 180.6450539281787, 174.43499518709206, 167.86708893290023, 161.40461770818555, 155.38990621419438, 150.10647927464134, 145.23447356374058, 139.7634]


    # Obstacles
    ob = np.array([
                   [20.0,176.0],
                   
                   [102.0,0.0]])
    print(ob)

    # Initial state
    c_speed = 0.5  # current speed [m/s]
    c_d = 0.0  # current lateral position [m]
    c_d_d = 0.0  # current lateral speed [m/s]
    c_d_dd = 0.0  # current lateral acceleration [m/s]
    s0 = 0.0  # current course position

    # Generate target course (assuming `generate_target_course` is defined elsewhere)
    tx, ty, tyaw, tc, csp = generate_target_course(wx, wy)
    # print(tyaw)
    # Initial state
    x, y, yaw = 16.993362287481073,152.51032455731195, np.deg2rad(90.0)  # Convert yaw to radians
    dyaw = np.deg2rad(-80.0)
    car_speed = 2.8  # Car speed [m/s]
    lookahead_distance = 4  # Lookahead distance [m]
    dt = 0.3  # Time step [s]
    L = 1.8  # Wheelbase of the vehicle [m]
    current_idx = 0  # Initial target point index
    area = 35.0  # Animation area length [m]
    show_animation = 1  # Enable animation
    
    # time = 0.0  # Initialize simulation time
    states_time = []  # Store states for plotting later
    car_steer = 0
    speed = 10
    time_start = 0
    fps = 0
    alpha = 0.1
    history_x = []
    history_y = []
    actual_path = []  # Lưu lại đường xe chạy thực tế
    optimal_path_x = []
    optimal_path_y = []

    while 1:
        # time += dt  # Update simulation time

        optimal_path, paths = frenet_optimal_planning(csp, s0, c_speed, c_d, c_d_d, c_d_dd, ob)
        if optimal_path is None:
            print("Error: Optimal path not found. Skipping iteration.")
            continue  # Skip to the next iteration if no path is found
        # Pure Pursuit control: use the optimal path from Frenet

        # Update vehicle state using kinematic equations
        dx = car_speed * np.cos(yaw) * dt
        dy = car_speed * np.sin(yaw) * dt
        dyaw = (car_speed / L) * np.tan(car_steer) * dt

        x += dx
        y += dy
        yaw += dyaw
        history_x.append(x)
        history_y.append(y)

        # print(yaw)
        # yaw = (yaw + np.pi) % (2 * np.pi) - np.pi
        # yaw = yaw % (2 * np.pi)

        car_steer, current_idx, lookahead_point = pure_pursuit_control_frenet(
            optimal_path, current_idx, x, y,yaw, lookahead_distance, L)

        # Project the current position onto the target course
        projected_point = project_onto_path(x, y, tx, ty)
        # s0 , c_d, c_d_d, c_d_dd = cartesian_to_frenet(projected_point[0], projected_point[1], yaw, csp)
        s0 , c_d, c_d_d, c_d_dd = cartesian_to_frenet(x, y, yaw, csp)
        
        # c_d = optimal_path.d[1]
        c_d_d = optimal_path.d_d[1]
        c_d_dd = optimal_path.d_dd[1]
        c_speed = car_speed    

        distance_to_goal = np.hypot(tx[-1] - x, ty[-1] - y)
  
        if distance_to_goal < 10:
            speed = slow_down_speed(distance_to_goal, 10)

        
        # print(f"Speed: {speed}")
        
        # Check if the goal is reached
        if np.isclose(x, tx[-1], atol=1.0) and np.isclose(y, ty[-1], atol=1.0):
            
            
            
            print("Goal reached!")
            break
        
        # # Visualization
        alpha = 0.5
        delta_t = time.time() - time_start
        if delta_t > 0:
            fps = (1 - alpha) * fps + alpha * (1 / delta_t)
        time_start = time.time()
        print(f"fps: {fps}")
        # time.sleep(0.01)
        # Visualization
        actual_path.append([x, y, yaw])  # Ghi lại vị trí xe thực tế
        optimal_path_x.append(lookahead_point[0])
        optimal_path_y.append(lookahead_point[1])
        
         # Nhấn phím 'S' để lưu dữ liệu
        if keyboard.is_pressed('s'):
            save_to_csv("trajectory_data.csv", optimal_path_x, optimal_path_y, history_x, history_y)
            
        if show_animation:
            global fig, ax, lookahead_circle  

            if fig is None or ax is None:  # Chỉ tạo figure một lần
                fig, ax = plt.subplots()
                lookahead_circle = plt.Circle((0, 0), lookahead_distance, color='b', fill=False, linestyle='--')
                ax.add_artist(lookahead_circle)

            ax.clear()  # Xóa nội dung cũ thay vì tạo mới figure

            # Vẽ đường mục tiêu
            ax.plot(tx, ty, "--y", label="Target Course")

            # Chuyển đổi tyaw nếu cần
            if not isinstance(tyaw, np.ndarray):
                tyaw = np.array(tyaw)

            # Tính toán đường ranh giới
            left_x = tx + (MAX_ROAD_WIDTH * 0.75 + 0.5) * np.cos(tyaw + np.pi / 2)
            left_y = ty + (MAX_ROAD_WIDTH * 0.75 + 0.5) * np.sin(tyaw + np.pi / 2)
            right_x = tx + (MAX_ROAD_WIDTH / 4 + 0.5) * np.cos(tyaw - np.pi / 2)
            right_y = ty + (MAX_ROAD_WIDTH / 4 + 0.5) * np.sin(tyaw - np.pi / 2)

            ax.plot(left_x, left_y, "black", label="Left Boundary")
            ax.plot(right_x, right_y, "black", label="Right Boundary")

            # Vẽ obstacles
            for o in ob:
                # print(ob)
                plot_obstacle(o[0], o[1], yaw=(yaw))

            # Chuẩn hóa danh sách x, y để có cùng độ dài
            max_len = max(len(traj.x) for traj in paths)
            all_x = [traj.x + [np.nan] * (max_len - len(traj.x)) for traj in paths]
            all_y = [traj.y + [np.nan] * (max_len - len(traj.y)) for traj in paths]

            ax.plot(np.array(all_x).T, np.array(all_y).T, "black", alpha=0.05)

            # Vẽ đường đi tối ưu
            ax.plot(optimal_path.x[1:], optimal_path.y[1:], "-g", alpha=0.95, label="Optimal Trajectory")

            # Cập nhật hình tròn lookahead
            lookahead_circle.set_center((x, y))

            # Vẽ các điểm quan trọng
            scatter_x = [x, lookahead_point[0]]
            scatter_y = [y, lookahead_point[1]]
            scatter_colors = ['blue', 'red']

            if projected_point is not None:
                scatter_x.append(projected_point[0])
                scatter_y.append(projected_point[1])
                scatter_colors.append('green')

            ax.scatter(scatter_x, scatter_y, c=scatter_colors, s=20, label=["Current Position", "Lookahead Point", "Projected Point"][:len(scatter_x)])

            # Vẽ xe
            plot_car(x, y, yaw, np.deg2rad(car_steer), cabcolor="-r")
            ax.plot(optimal_path_x, optimal_path_y, 'b-', label="Opti Path")  # Vẽ đường xe đã chạy
            ax.plot(history_x, history_y, 'r-', label="Path traveled")  # Vẽ đường xe đã chạy
            
            ax.set_xlim(x - area, x + area)
            ax.set_ylim(y - area, y + area)
            ax.set_title(f"Speed: {car_speed * 3.6:.2f} km/h | Steering Angle: {car_steer:.2f}°")
            ax.grid(True)
            ax.legend()

            plt.pause(1 / 60)  # Cập nhật hình ảnh nhanh


    
    print("Simulation finished.")
    if show_animation:
        plt.show()

if __name__ == '__main__':
    main()

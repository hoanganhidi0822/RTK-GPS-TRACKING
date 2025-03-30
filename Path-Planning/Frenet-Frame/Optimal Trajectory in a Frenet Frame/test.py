import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Hàm tính các vectơ Frenet
def frenet_frame(t):
    # Định nghĩa đường cong tham số (helix)
    x = np.cos(t)
    y = np.sin(t)
    z = t
    r = np.array([x, y, z])

    # Đạo hàm bậc 1
    dx = -np.sin(t)
    dy = np.cos(t)
    dz = 1
    dr = np.array([dx, dy, dz])

    # Đạo hàm bậc 2
    ddx = -np.cos(t)
    ddy = -np.sin(t)
    ddz = 0
    ddr = np.array([ddx, ddy, ddz])

    # Vectơ tiếp tuyến
    T = dr / np.linalg.norm(dr)

    # Vectơ pháp tuyến
    dT = ddr / np.linalg.norm(dr) - (np.dot(dr, ddr) / np.linalg.norm(dr)**3) * dr
    N = dT / np.linalg.norm(dT)

    # Vectơ nhị diện
    B = np.cross(T, N)

    return r, T, N, B

# Tạo dữ liệu
t_values = np.linspace(0, 4 * np.pi, 100)  # Tham số t
points = []
tangents = []
normals = []
binormals = []

for t in t_values:
    r, T, N, B = frenet_frame(t)
    points.append(r)
    tangents.append(T)
    normals.append(N)
    binormals.append(B)

points = np.array(points)
tangents = np.array(tangents)
normals = np.array(normals)
binormals = np.array(binormals)

# Vẽ 3D
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Vẽ đường cong
ax.plot(points[:, 0], points[:, 1], points[:, 2], 'b-', label="Helix")

# Vẽ các vectơ Frenet tại một số điểm
for i in range(0, len(t_values), 10):
    ax.quiver(
        points[i, 0], points[i, 1], points[i, 2], 
        tangents[i, 0], tangents[i, 1], tangents[i, 2], 
        color='r', length=0.5, normalize=True, label="Tangent (T)" if i == 0 else ""
    )
    ax.quiver(
        points[i, 0], points[i, 1], points[i, 2], 
        normals[i, 0], normals[i, 1], normals[i, 2], 
        color='g', length=0.5, normalize=True, label="Normal (N)" if i == 0 else ""
    )
    ax.quiver(
        points[i, 0], points[i, 1], points[i, 2], 
        binormals[i, 0], binormals[i, 1], binormals[i, 2], 
        color='k', length=0.5, normalize=True, label="Binormal (B)" if i == 0 else ""
    )

# Thiết lập trục
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.legend()
plt.title("Hệ tọa độ Frenet trên đường cong xoắn ốc")
plt.show()

import matplotlib.pyplot as plt
import networkx as nx
import math
import numpy as np
from queue import PriorityQueue
import glob
import os
from math import radians, sin, cos, sqrt, atan2
from Assistance_Astar.overlay import *
from Assistance_Astar.location_finder  import *

class Graph:
    def __init__(self):
        # Define GPS coordinates for each node (latitude, longitude)
        self.coordinates = {
            "A":  (10.8532733433, 106.7715069217), # Khu C
            "BB": (10.852302    , 106.771424    ), # Khu D
            "B":  (10.8514838933, 106.7713101400),
            "TT": (10.8512819350, 106.7719588833), # Toa trung Tam
            "C":  (10.851238    ,     106.772669), # Toa Viet Duc
            "CC": (10.851554    ,     106.772746),
            "D":  (10.851198    ,     106.773302),
            "DD": (10.851641    ,     106.773369), # Maker Space
            "E":  (10.852292    ,     106.773450),
            "F":  (10.852364    ,     106.772835),
            "G":  (10.853240    ,     106.772932), # Go
            "H":  (10.853319    ,     106.772592),
            "I":  (10.853541    ,     106.772572),
            "K":  (10.853686    ,     106.771636),
        }

        self.segments = [
            ("A", "BB"),("BB", "A"), ("BB", "B"), ("B", "TT"),("TT", "C"), ("C", "D"), ("C", "CC"),("CC", "C"),("C", "TT"),("TT", "B"),("B", "BB"),
            ("CC", "F"), ("D", "DD"), ("DD", "E"),("E", "F"), ("F", "G"), ("G", "F"),("F", "CC"),
            ("G", "H"),("H", "G"), ("H", "I"),("H", "I"), ("I", "K"), ("K", "I"),("K", "A"),("A", "K")
        ]
        self.threshold = 10  # Ngưỡng khoảng cách (mét)
        self.graph = {}
        self.weights = {}

    def haversine_distance(self, point1, point2):
        """Calculate Haversine distance between two GPS coordinates."""
        R = 6371.0  # Radius of the Earth in kilometers
        lat1, lon1 = point1
        lat2, lon2 = point2

        # Convert latitude and longitude from degrees to radians
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        # Haversine formula
        a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return R * c * 1000  # Distance in meters

    def project_point_to_line(self, A, B, I):
        """
        Project point I onto line segment AB and calculate the projected point H.
        """
        A = np.array(A)
        B = np.array(B)
        I = np.array(I)

        AB = B - A
        AI = I - A

        AB_squared = np.dot(AB, AB)
        if AB_squared == 0:
            return None, float('inf')  # A and B are the same point

        t = np.dot(AI, AB) / AB_squared
        t = max(0, min(1, t))  # Clamp t to [0, 1]

        H = A + t * AB
        distance = self.haversine_distance(I, H)
        return H, distance

    def update_graph_with_projection(self, current_pos):
        """
        Update graph segments by adding projection points if within the threshold.
        """
        added_points = []  # Store added projection points
        updated_segments = []

        for seg in self.segments:
            A, B = self.coordinates[seg[0]], self.coordinates[seg[1]]
            H, distance = self.project_point_to_line(A, B, current_pos)

            if distance < self.threshold:
                H_name = "H1"
                added_points.append((H_name, H))
                updated_segments.append((seg[0], H_name))
                updated_segments.append((H_name, seg[1]))
            else:
                updated_segments.append(seg)

        for H_name, H_coord in added_points:
            self.coordinates[H_name] = tuple(H_coord)

        self.segments = updated_segments
        self.build_graph()

    def build_graph(self):
        """Build the graph structure and calculate weights."""
        self.graph = {}
        for start, end in self.segments:
            if start not in self.graph:
                self.graph[start] = []
            self.graph[start].append(end)
        self.calculate_weights()

    def calculate_weights(self):
        """Calculate weights for all edges."""
        self.weights = {}
        for from_node, neighbors in self.graph.items():
            for to_node in neighbors:
                self.weights[(from_node, to_node)] = self.haversine_distance(
                    self.coordinates[from_node], self.coordinates[to_node]
                )

    def neighbors(self, node):
        return self.graph.get(node, [])

    def get_cost(self, from_node, to_node):
        return self.weights.get((from_node, to_node), float('inf'))

    def heuristic(self, node, goal):
        return self.haversine_distance(self.coordinates[node], self.coordinates[goal])

def astar(graph, start, goal):
    queue = PriorityQueue()
    queue.put((0, start))
    came_from = {}
    cost_so_far = {start: 0}

    while not queue.empty():
        _, current = queue.get()

        if current == goal:
            break

        for neighbor in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.get_cost(current, neighbor)
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + graph.heuristic(neighbor, goal)
                queue.put((priority, neighbor))
                came_from[neighbor] = current

    path = []
    node = goal
    while node != start:
        path.append((came_from[node], node))
        node = came_from[node]
    path.reverse()

    return path

# Cấu hình file waypoint cho từng đoạn đường
waypoint_files = {
    ('H1' , 'BB' )  : "MAP/THUAN/waypoint_1.txt" ,
    ('H1' , 'DD' )  : "MAP/THUAN/space.txt"      ,
    ('H1' , 'B'  )  : "MAP/THUAN/waypoint_2.txt" ,
    ('H1' , 'TT' )  : "MAP/THUAN/waypoint_3.txt" ,
    ('H1' , 'C'  )  : "MAP/NGHICH/waypoint_3.txt" ,
    ('H1' , 'CC' )  : "MAP/THUAN/waypoint_5.txt",
    ('H1' , 'F'  )  : "MAP/THUAN/space.txt" ,
    ('BB' , 'B'  )  : "MAP/THUAN/waypoint_2.txt" ,
    ('B'  , 'TT' )  : "MAP/THUAN/waypoint_3.txt" ,
    ('TT' , 'C'  )  : "MAP/THUAN/waypoint_4.txt" ,
    ('C'  , 'CC' )  : "MAP/THUAN/waypoint_5.txt" ,
    ('CC' , 'F'  )  : "MAP/THUAN/waypoints_6.txt",
    ('C'  , 'D'  )  : "MAP/THUAN/space.txt"      ,
    ('D'  , 'DD' )  : "MAP/THUAN/waypoints_8.txt",
    ('DD' , 'E'  )  : "MAP/THUAN/space.txt"      ,
    ('E'  , 'F'  )  : "MAP/THUAN/space.txt"      ,
    ('F'  , 'G'  )  : "MAP/THUAN/space.txt"      ,
    ('G'  , 'H'  )  : "MAP/THUAN/waypoints_7.txt",
    ('H'  , 'I'  )  : "MAP/THUAN/space.txt"      ,
    ('I'  , 'K'  )  : "MAP/THUAN/space.txt"      ,
    ('I'  , 'K'  )  : "MAP/THUAN/space.txt"      ,
    ('K'  , 'A'  )  : "MAP/THUAN/space.txt"      ,
    
    ('A'  , 'K'  )  : "MAP/NGHICH/waypoint_1.txt",
    ('K'  , 'I'  )  : "MAP/NGHICH/space.txt"     ,
    ('I'  , 'H'  )  : "MAP/NGHICH/space.txt"     ,
    ('I'  , 'H'  )  : "MAP/NGHICH/space.txt"     ,
    ('H'  , 'G'  )  : "MAP/NGHICH/space.txt"     ,
    ('G'  , 'F'  )  : "MAP/NGHICH/waypoint_2.txt",
    ('F'  , 'CC' )  : "MAP/NGHICH/waypoint_2.txt",
    ('CC' , 'C'  )  : "MAP/NGHICH/waypoint_3.txt",
    ('C'  , 'TT' )  : "MAP/NGHICH/space.txt"     ,
    ('TT' , 'B'  )  : "MAP/NGHICH/waypoint_4.txt",
    ('B'  , 'BB' )  : "MAP/NGHICH/space.txt"     ,
    ('BB' , 'A'  )  : "MAP/NGHICH/waypoint_5.txt",
}

def haversine(lat1, lon1, lat2, lon2):
    R = 6371000  # Bán kính Trái Đất (m)
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    
    a = math.sin(delta_phi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    return R * c

def load_waypoints(file_path):
    """ Đọc nội dung của file waypoint """
    try:
        with open(file_path, 'r') as file:
            waypoints = file.readlines()
        return waypoints
    except FileNotFoundError:
        print(f"⚠️ File {file_path} không tồn tại!")
        return []


def haversine(lat1, lon1, lat2, lon2):
    """Tính khoảng cách Haversine giữa hai tọa độ."""
    R = 6371000  # Bán kính Trái Đất (m)
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return R * c  # Khoảng cách tính bằng mét

def project_point_on_line(p, a, b):
    """ Chiếu điểm P lên đoạn thẳng AB """
    px, py = p
    ax, ay = a
    bx, by = b

    abx, aby = bx - ax, by - ay
    apx, apy = px - ax, py - ay
    ab_length_squared = abx**2 + aby**2

    # Hệ số t cho vị trí chiếu
    t = (apx * abx + apy * aby) / ab_length_squared
    t = max(0, min(1, t))  # Giữ t trong [0,1] để nằm trên đoạn AB

    # Tọa độ chiếu vuông góc
    projected_x = ax + t * abx
    projected_y = ay + t * aby

    return projected_x, projected_y

def find_nearest_waypoints(input_lat, input_lon, base_folder="D:/Documents/Researches/2024_Project/Astar/MAP/"):
    txt_files = glob.glob(f"{base_folder}/THUAN/*.txt") + glob.glob(f"{base_folder}/NGHICH/*.txt")
    
    nearest_file = None
    nearest_waypoint_1 = None
    nearest_waypoint_2 = None
    min_distance_1 = float("inf")
    min_distance_2 = float("inf")
    
    for file in txt_files:
        with open(file, "r") as f:
            waypoints = []
            for line in f:
                line = line.strip()
                if not line:  # Bỏ qua dòng trống
                    continue
                try:
                    lat, lon = map(float, line.split(","))
                    waypoints.append((lat, lon))
                except ValueError:
                    print(f"⚠️ Bỏ qua dòng không hợp lệ trong {file}: {line}")
        
        # Tìm 2 waypoint gần nhất
        for i in range(len(waypoints) - 1):
            lat1, lon1 = waypoints[i]
            lat2, lon2 = waypoints[i + 1]

            d1 = haversine(input_lat, input_lon, lat1, lon1)
            d2 = haversine(input_lat, input_lon, lat2, lon2)

            if d1 < min_distance_1:
                min_distance_2 = min_distance_1
                nearest_waypoint_2 = nearest_waypoint_1
                
                min_distance_1 = d1
                nearest_waypoint_1 = (lat1, lon1)
                nearest_file = os.path.basename(file)
            
            if d2 < min_distance_1:
                min_distance_2 = min_distance_1
                nearest_waypoint_2 = nearest_waypoint_1
                
                min_distance_1 = d2
                nearest_waypoint_1 = (lat2, lon2)
                nearest_file = os.path.basename(file)

    if nearest_waypoint_1 and nearest_waypoint_2:
        # Chiếu tọa độ hiện tại lên đoạn thẳng nối giữa 2 waypoint gần nhất
        projected_point = project_point_on_line((input_lat, input_lon), nearest_waypoint_1, nearest_waypoint_2)

        # Tính khoảng cách từ điểm chiếu đến tọa độ hiện tại
        projected_distance = haversine(input_lat, input_lon, projected_point[0], projected_point[1])

        # Chỉ trả về nếu khoảng cách < 1.5m
        if projected_distance < 1.5:
            return nearest_file, projected_point, projected_distance
        else:
            return None, None, None

    return None, None, None # Không tìm thấy waypoint phù hợp

def compare_coordinates(coord1, coord2, epsilon=1e-7):
    # print(coord1)
    """So sánh hai tọa độ với sai số nhỏ epsilon. Đảm bảo rằng tọa độ là kiểu float."""
    coord1 = tuple(map(float, coord1))  # Chuyển đổi tọa độ thành kiểu float
    coord2 = tuple(map(float, coord2))  # Chuyển đổi tọa độ thành kiểu float
    return (math.isclose(coord1[0], coord2[0], abs_tol=epsilon) and
            math.isclose(coord1[1], coord2[1], abs_tol=epsilon))

def load_waypoints(file_name):
    waypoints = []
    with open(file_name, 'r') as f:
        for line in f:
            # Tách chuỗi, bỏ khoảng trắng, sau đó chuyển thành tuple (float, float)
            coords = line.strip().split(',')
            if len(coords) == 2:
                waypoints.append(tuple(map(float, coords)))
    return waypoints

def compare_coordinates(coord1, coord2, epsilon=1e-6):
    """So sánh hai tọa độ với sai số nhỏ epsilon. Đảm bảo rằng tọa độ là kiểu float."""
    return (math.isclose(coord1[0], coord2[0], abs_tol= epsilon) and
            math.isclose(coord1[1], coord2[1], abs_tol= epsilon))

def find_optimal_path(current_position, target_node):
    # Tìm waypoint gần nhất
    name, nearest_point, distance = find_nearest_waypoints(current_position[0], current_position[1])
    print(f"name: {name}, distance: {distance}, nearest point: {nearest_point}")

    graph = Graph()
    graph.update_graph_with_projection(current_position)

    # Tìm start_node
    projected_nodes = list(set(graph.coordinates.keys()) - set(graph.graph.keys()))
    if projected_nodes:
        start_node = projected_nodes[0]  # Dùng điểm chiếu nếu có
    else:
        closest_node = min(
            graph.coordinates.keys(),
            key=lambda node: graph.haversine_distance(current_position, graph.coordinates[node])
        )
        start_node = closest_node

    # Tìm đường đi tối ưu
    optimal_path = astar(graph, start_node, target_node)

    # Danh sách chứa toàn bộ waypoint từ các file
    all_waypoints = []

    print("\n### Loading Waypoints ###")
    for (from_node, to_node) in optimal_path:
        file_name = waypoint_files.get((from_node, to_node))
        print(name)
        print(file_name)
        if file_name:
            print(f"📥 Loading waypoints from {file_name} for segment: {from_node} -> {to_node}")

            # Đọc dữ liệu từ file
            waypoints = load_waypoints(file_name)

            print(f"file_name: {file_name}, name: {name}")  # Debugging
            if os.path.basename(file_name) == os.path.basename(name):
                # Nếu là file chứa nearest_point, chỉ lấy các waypoint phía sau nó
                try:
                    # Tìm nearest_point trong file waypoint với sai số nhỏ
                    index = next(i for i, wp in enumerate(waypoints) if compare_coordinates(wp, nearest_point))
                    print(f"index: {index}")
                    waypoints = waypoints[index:]  # Lấy từ nearest_point trở đi
                except StopIteration:
                    print(f"⚠️ Nearest point {nearest_point} not found in {file_name}")
            # Lưu vào danh sách tổng hợp
            all_waypoints.extend(waypoints)
        else:
            print(f"⚠️ No waypoint file configured for {from_node} -> {to_node}")

    # Ghi toàn bộ dữ liệu vào file duy nhất
    merged_file = "merged_waypoints.txt"
    with open(merged_file, 'w') as file:
        file.writelines([f"{wp[0]},{wp[1]}\n" for wp in all_waypoints])
    print(f"\n✅ All waypoints merged into {merged_file}")

    # # Visualization
    # G = nx.DiGraph()
    # for (from_node, to_node), weight in graph.weights.items():
    #     G.add_edge(from_node, to_node, weight=round(weight, 2))

    # pos = {node: (coord[1], coord[0]) for node, coord in graph.coordinates.items()}
    # plt.figure(figsize=(15, 30))
    # nx.draw(G, pos, with_labels=True, node_size=200, node_color='lightblue', font_size=10, font_weight='bold')
    # edge_labels = {(u, v): f"{d['weight']:.2f} m" for u, v, d in G.edges(data=True)}
    # nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=8)
    # optimal_edges = [(u, v) for u, v in optimal_path]
    # nx.draw_networkx_edges(G, pos, edgelist=optimal_edges, edge_color='red', width=2)
    # plt.title("Graph Visualization with Optimal Path (Haversine Distance)")
    # plt.grid(1)
   
    return optimal_path




def run_map(name):
    # Example usage 
    # gps_ser = connect_to_serial("COM17", 115200)
    # lat, lon, car_heading, sat_count = get_gps_data(gps_ser)
    
    # import math

    # while True:
    #     lat, lon, car_heading, sat_count = get_gps_data(gps_ser)
        
    #     try:
    #         lat = float(lat)  # Chuyển đổi lat sang số thực
    #         lon = float(lon)
            
    #         if not math.isnan(lat):  # Nếu lat không phải NaN thì thoát vòng lặp
    #             break
    #     except ValueError:
    #         pass  # Nếu không thể chuyển đổi, tiếp tục lấy dữ liệu GPS

    # current_position = (lat, lon)
    current_position = 10.852736,106.77144
    finder = LocationFinder()
    # name = input("Nhập tên khu vực: ")
    
    print(f"Mã khu vực: {finder.get_key(name)}")

    target_node = finder.get_key(name)
    path = find_optimal_path(current_position, target_node)
    print("Optimal Path:", path)

    important_nodes = {
        "Khu C"         : (10.8532423383, 106.7715303633 ),  # Khu C
        "Khu D"         : (10.852302    , 106.771424     ),  # Khu D
        "Toa Trung Tam" : (10.8512819350, 106.7719588833 ),  # Toa Trung Tam
        "Toa Viet Duc"  : (10.8514139833, 106.7726869067 ),  # Toa Viet Duc
        "Maker Space"   : (10.8513325717, 106.7733373500 ),  # Maker Space
        "Xuong Go"      : (10.8528667033, 106.7728553250 )   # Góc
    }

    # filename1 = 'PROCESS_DATA\MAP_THUAN\HD_MAP1.txt'  # First file with waypoints
    # filename2 = 'PROCESS_DATA\MAP_THUAN\HD_MAP2.txt'  # First file with waypoints
    # filename3 = 'PROCESS_DATA\MAP_NGHICH\HD_MAP1.txt' # First file with waypoints
    # filename4 = 'PROCESS_DATA\MAP_NGHICH\HD_MAP2.txt' # First file with waypoints

    filename1 = 'PROCESS_DATA/MAP_THUAN/HD_MAP1.txt'  # First file with waypoints
    filename2 = 'PROCESS_DATA/MAP_THUAN/HD_MAP2.txt'  # First file with waypoints
    filename3 = 'PROCESS_DATA/MAP_NGHICH/HD_MAP1.txt' # First file with waypoints
    filename4 = 'PROCESS_DATA/MAP_NGHICH/HD_MAP2.txt' # First file with waypoints

    filename5 = './merged_waypoints.txt'  # Third file with waypoints
    waypoints1 = read_waypoints(filename1)
    waypoints2 = read_waypoints(filename2)
    waypoints3 = read_waypoints(filename3)
    waypoints4 = read_waypoints(filename4)
    waypoints5 = read_waypoints(filename5)

    # Create the plot
    plt.figure(figsize=(10, 6))

    # Plot the three paths with different colors
    plot_path(waypoints1, 'silver', 'Path 1')  # Blue for path 1
    plot_path(waypoints2, 'silver', 'Path 2')  # Blue for path 1
    plot_path(waypoints3, 'silver', 'Path 3')  # Blue for path 1
    plot_path(waypoints4, 'silver', 'Path 4')  # Blue for path 1
    plot_path(waypoints5, 'r'     , 'Path 5')  # Green for path 5

    # Vẽ các node quan trọng
    for name, (lat, lon) in important_nodes.items():
        plt.scatter(lon, lat, color='gold', edgecolors='black', s=100, marker='*', label=name)
        plt.text(lon, lat, name, fontsize=8, verticalalignment='bottom', horizontalalignment='center')

    # Add labels, title, and legend
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title('HD MAP')
    plt.legend()

    # Add grid and show the plot
    plt.grid(True)
    # plt.draw()
    # plt.pause(0.01)



# run_map(name = "go")
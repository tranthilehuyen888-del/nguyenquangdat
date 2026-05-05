import heapq

# Tạo node cho thuật toán A*
def create_node(position, g=float('inf'), h=0, parent=None):
    return {
        'position': position,
        'g': g,
        'h': h,
        'f': g + h,
        'parent': parent
    }

# Hàm heuristic dùng khoảng cách Manhattan vì robot chỉ đi 4 hướng
def calculate_heuristic(pos1, pos2):
    x1, y1 = pos1
    x2, y2 = pos2
    return abs(x1 - x2) + abs(y1 - y2)

# Lấy chi phí khi đi vào một ô
def get_cell_cost(cell_value):
    if cell_value == 0:
        return 1      # Ô trống
    elif cell_value == 2:
        return 3      # Bùn lầy
    elif cell_value == 3:
        return 5      # Đá
    else:
        return float('inf')  # Vật cản hoặc ô không hợp lệ

# Lấy các ô lân cận hợp lệ: chỉ đi lên, xuống, trái, phải
def get_valid_neighbors(grid, position):
    x, y = position
    rows = len(grid)
    cols = len(grid[0])

    possible_moves = [
        (x - 1, y),  # Lên
        (x + 1, y),  # Xuống
        (x, y - 1),  # Trái
        (x, y + 1)   # Phải
    ]

    neighbors = []

    for nx, ny in possible_moves:
        # Kiểm tra không vượt biên và không phải vật cản
        if 0 <= nx < rows and 0 <= ny < cols and grid[nx][ny] != 1:
            neighbors.append((nx, ny))

    return neighbors

# Dựng lại đường đi từ goal về start
def reconstruct_path(goal_node):
    path = []
    current = goal_node

    while current is not None:
        path.append(current['position'])
        current = current['parent']

    return path[::-1]

# Thuật toán A*
def find_path(grid, start, goal):
    start_node = create_node(
        position=start,
        g=0,
        h=calculate_heuristic(start, goal)
    )

    open_list = []
    heapq.heappush(open_list, (start_node['f'], start))

    open_dict = {start: start_node}
    closed_set = set()

    while open_list:
        # Lấy node có f nhỏ nhất
        _, current_pos = heapq.heappop(open_list)
        current_node = open_dict[current_pos]

        # Nếu đến đích thì trả về đường đi
        if current_pos == goal:
            return reconstruct_path(current_node), current_node['g']

        closed_set.add(current_pos)

        # Duyệt các ô hàng xóm
        for neighbor_pos in get_valid_neighbors(grid, current_pos):
            if neighbor_pos in closed_set:
                continue

            x, y = neighbor_pos
            move_cost = get_cell_cost(grid[x][y])

            tentative_g = current_node['g'] + move_cost

            if neighbor_pos not in open_dict:
                neighbor_node = create_node(
                    position=neighbor_pos,
                    g=tentative_g,
                    h=calculate_heuristic(neighbor_pos, goal),
                    parent=current_node
                )

                open_dict[neighbor_pos] = neighbor_node
                heapq.heappush(open_list, (neighbor_node['f'], neighbor_pos))
            elif tentative_g < open_dict[neighbor_pos]['g']:
                neighbor_node = open_dict[neighbor_pos]
                neighbor_node['g'] = tentative_g
                neighbor_node['f'] = tentative_g + neighbor_node['h']
                neighbor_node['parent'] = current_node

                heapq.heappush(open_list, (neighbor_node['f'], neighbor_pos))

    return [], 0

# Hiển thị bản đồ và đường đi
def visualize_path(grid, path, start, goal):
    display_grid = []

    for row in grid:
        display_grid.append(row.copy())

    for x, y in path:
        if (x, y) != start and (x, y) != goal:
            display_grid[x][y] = '*'

    sx, sy = start
    gx, gy = goal
    display_grid[sx][sy] = 'S'
    display_grid[gx][gy] = 'G'

    print("\nBản đồ sau khi tìm đường:")
    print("S: Start | G: Goal | *: Đường đi | 1: Vật cản | 2: Bùn | 3: Đá")

    for row in display_grid:
        print(' '.join(str(cell) for cell in row))

# Hàm main
def main():
    # 0: ô trống
    # 1: vật cản
    # 2: bùn lầy, chi phí 3
    # 3: đá, chi phí 5

    grid = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0, 2, 2, 0, 1, 0],
        [0, 0, 0, 1, 0, 2, 0, 0, 1, 0],
        [0, 3, 0, 1, 0, 0, 0, 3, 1, 0],
        [0, 3, 0, 0, 0, 1, 1, 0, 0, 0],
        [0, 3, 3, 3, 0, 0, 0, 0, 2, 0],
        [0, 0, 0, 3, 1, 1, 1, 0, 2, 0],
        [1, 1, 0, 0, 0, 2, 0, 0, 2, 0],
        [0, 0, 0, 1, 0, 2, 0, 3, 3, 0],
        [0, 1, 0, 0, 0, 0, 0, 0, 0, 0]
    ]

    start = (0, 0)
    goal = (9, 9)

    path, total_cost = find_path(grid, start, goal)

    if path:
        print("Tìm thấy đường đi tối ưu!")
        print("Đường đi:", path)
        print("Số bước đi:", len(path) - 1)
        print("Tổng chi phí:", total_cost)

        visualize_path(grid, path, start, goal)
    else:
        print("Không tìm thấy đường đi!")

if __name__ == "__main__":
    main()
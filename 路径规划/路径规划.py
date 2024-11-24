import sys
import heapq
import math


def main():
    # 输入
    n_m = sys.stdin.readline().strip().split()
    while len(n_m) < 2:
        n_m += sys.stdin.readline().strip().split()
    n, m = map(int, n_m)

    # 代价地图
    grid = []
    for _ in range(n):
        while True:
            line = sys.stdin.readline()
            if line.strip() == '':
                continue
            nums = list(map(int, line.strip().split()))
            if len(nums) >= m:
                grid.append(nums[:m])
                break

    # 膨胀系数
    t_line = sys.stdin.readline().strip()
    while t_line == '':
        t_line = sys.stdin.readline().strip()
    t = float(t_line)

    # 起始和终止坐标
    coords = []
    while len(coords) < 4:
        coord_line = sys.stdin.readline().strip()
        if coord_line == '':
            continue
        coords += list(map(int, coord_line.strip().split()))
    x1, y1, x2, y2 = coords[0], coords[1], coords[2], coords[3]

    # 计算每个点的邻居grid值之和
    sum_neighbors = [[0 for _ in range(m)] for _ in range(n)]
    for x in range(n):
        for y in range(m):
            s = 0
            if x > 0:
                s += grid[x - 1][y]
            if x < n - 1:
                s += grid[x + 1][y]
            if y > 0:
                s += grid[x][y - 1]
            if y < m - 1:
                s += grid[x][y + 1]
            sum_neighbors[x][y] = s

    # 计算膨胀地图
    expanded_cost = [[0.0 for _ in range(m)] for _ in range(n)]
    for x in range(n):
        for y in range(m):
            expanded_cost[x][y] = grid[x][y] + t * sum_neighbors[x][y]
    # 起点的膨胀代价设为0
    expanded_cost[x1][y1] = 0.0

    # Dijkstra算法
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 上下左右四个方向
    distances = [[math.inf for _ in range(m)] for _ in range(n)]  # 初始化距离为无穷大
    distances[x1][y1] = 0.0  # 起点到起点的距离为0
    heap = []
    heapq.heappush(heap, (0.0, x1, y1))  # 将起点加入堆
    visited = [[False for _ in range(m)] for _ in range(n)]  # 记录是否访问过

    while heap:
        current_dist, x, y = heapq.heappop(heap)  # 取出当前最小距离的点
        if visited[x][y]:
            continue
        visited[x][y] = True
        if x == x2 and y == y2:  # 如果到达终点，跳出循环
            break
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < n and 0 <= ny < m and not visited[nx][ny]:
                new_dist = current_dist + expanded_cost[nx][ny]  # 计算新距离
                if new_dist < distances[nx][ny]:
                    distances[nx][ny] = new_dist
                    heapq.heappush(heap, (new_dist, nx, ny))  # 更新距离并加入堆

    # 输出终点的最小代价
    print("{0:.10f}".format(distances[x2][y2]))


if __name__ == "__main__":
    main()

import copy

from grid_visualization import GridVisualization
from heapq import heappush, heappop


'''
在main.py中修改在初始化之后为每个机器人添加自己的路径，
采用A*算法让机器人从当前位置运动到delivery_pos,采用三维数组存储，
第二三个坐标对应着在棋盘上的坐标，第一个坐标表示时间，
下一刻的坐标为上一刻的坐标上下左右或者保持，
利用A*算法产生从初始地点到达dilivery_pos的最短路径，
启发函数为曼哈顿距离。如果数组不够大那么添加一个新的数组。
每一次搜索后保留在三维数组中的结果是某刻在x,y的位置上有机器人占据。
不同的机器人不能占有相同的格子。A*搜索在添加了新的任务之后需要使用。
对于机器人x,如果添加了新的任务，需要先把货物运送到delivery_pos再到下一个目标地点。
因此一个机器人拥有两种运动方向，一种是去delivery_pos,一种是去任务点。
每当任务完成之后检查任务列表是否为空，如果任务列表为空的话，让机器人保持等待。
在A*搜索的过程中，如果发现前面的机器人是没有任务的状态并且机器人的任务列表为空，
那么就要为它在三维数组中找到一个位置放置它使它不挡路，如果发现它是有任务的，那么就绕开它。
'''
class Node:
    def __init__(self, t, x, y, g, h, parent=None, direction=None):
        self.t = t
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent
        self.direction = direction

    def __lt__(self, other):
        return self.f < other.f

def planrobot(planner, robotid, targetx, targety, begintime, robot_actions):
    # 创建planner副本
    planner_copy = copy.deepcopy(planner)
    
    # 初始化移动方向和反向映射
    movements = {"up":(-1,0), "down":(1,0), "left":(0,-1), "right":(0,1), "stay":(0,0)}
    movements_rev = {(-1,0):"up", (1,0):"down", (0,-1):"left", (0,1):"right", (0,0):"stay"}
    
    # 查找机器人起始位置
    start_t = begintime
    start_x, start_y = -1, -1
    for i in range(len(planner_copy[0])):
        for j in range(len(planner_copy[0][0])):
            if planner_copy[start_t][i][j] == robotid:
                start_x, start_y = i, j
                break
        if start_x != -1:
            break
    
    # 初始化优先队列
    heap = []
    initial_h = abs(targetx - start_x) + abs(targety - start_y)
    heappush(heap, Node(start_t, start_x+1, start_y+1, 0, initial_h))
    
    # 动态扩展三维数组容量
    def expand_planner(the_planner,time_needed,notchange=-1):
        while len(the_planner)<= time_needed:
            new_layer = [[-1 for _ in range(m)] for _ in range(n)]
            the_planner.append(new_layer)
    
    # A*搜索主循环
    while heap:
        current = heappop(heap)
        # 到达目标
        if (current.x, current.y) == (targetx, targety):
            # 回溯并记录路径
            path = []
            while current:
                path.append((current.t, current.x, current.y, current.direction))
                current = current.parent
            path.reverse()
            
            # 更新planner和robot_actions
            if path:
                expand_planner(planner,0)
                planner[path[0][0]][path[0][1]-1][path[0][2]-1] = robotid
                
                for t, x, y, direction in path[1:]:
                    expand_planner(planner,t,robotid)
                    planner[t][x-1][y-1] = robotid
                    if direction:
                        robot_actions[robotid].append(direction)

                # 处理后续时间层冲突
                current_time = path[-1][0]
                while current_time < len(planner)-1:
                    current_time += 1
                    expand_planner(planner, current_time)
                    
                    # 检查未来时间层是否被占用
                    if planner[current_time][targetx-1][targety-1] != -1:
                        # 寻找相邻空闲位置
                        for dx, dy in [(0,1),(0,-1),(1,0),(-1,0),(0,0)]:
                            nx = targetx + dx
                            ny = targety + dy
                            if 0 < nx <= n and 0 < ny <= m \
                                    and planner[current_time][nx-1][ny-1] == -1:
                                planner[current_time][nx-1][ny-1] = robotid
                                robot_actions[robotid].append(movements_rev[(dx, dy)])
                                break
                        else:
                            # 无法找到空闲位置则保持原位
                            planner[current_time][targetx-1][targety-1] = robotid
                            robot_actions[robotid].append('stay')
                    else:
                        planner[current_time][targetx-1][targety-1] = robotid
                        robot_actions[robotid].append(movements_rev[(0, 0)])
            return
        
        # 生成子节点
        for dx, dy in [(0,1), (0,-1), (1,0), (-1,0), (0,0)]:
            next_t = current.t + 1
            next_x = current.x + dx
            next_y = current.y + dy
            
            # 边界检查
            if 0 < next_x <= n and 0 < next_y <= m:
                expand_planner(planner_copy,next_t,robotid)
                
                # 检查位置是否可用
                if planner_copy[next_t][next_x-1][next_y-1] == -1:
                    # 计算启发式值
                    h = abs(targetx - next_x) + abs(targety - next_y)
                    new_node = Node(next_t, next_x, next_y, current.g + 1, h, 
                                   current, movements_rev[(dx, dy)])
                    
                    # 直接检查三维数组标记
                    if planner_copy[next_t][next_x-1][next_y-1] == -1:
                        planner_copy[next_t][next_x-1][next_y-1] = robotid
                        heappush(heap, new_node)
    
    

if __name__ == '__main__':
    n = 6  # 仓库的行数
    m = 6  # 仓库的列数
    delivery_pos = (6, 6)  # 交付站的位置（0-based索引）
    robot_positions = [(4,3), (3,5)]
    robot_actions = [[] for _ in range(len(robot_positions))]
    tasks = [[] for _ in range(len(robot_positions))]
    action_pointers = [0] * len(robot_positions)
    
    # 初始化planner三维数组
    planner = [[[-1 for _ in range(m)] for _ in range(n)]]
    for idx, (x, y) in enumerate(robot_positions):
        planner[0][x-1][y-1] = idx
    
    # 测试路径规划
    planrobot(planner, 0, delivery_pos[0], delivery_pos[1], 0, robot_actions)
    for i in planner:
        for j in i:
            print(j)
        print()
    planrobot(planner, 1, delivery_pos[0], delivery_pos[1], 0, robot_actions)
    action_pointers = [0] * len(robot_positions)
    print(tasks)
    grid=GridVisualization(n,m,delivery_pos,robot_positions,robot_actions,tasks,action_pointers)
    while True:
        grid.next()
    print(robot_actions)
    

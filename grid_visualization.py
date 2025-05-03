import tkinter as tk
from tkinter import simpledialog
'''
    GridVisualization 使用方式如下：
        1.初始化时使用GridVisualization(n, m, delivery_pos, robot_positions,
            robot_actions, tasks, action_pointers)
            n,m为仓库的行数和列数
            delivery_pos为交付站的位置
            robot_positions为机器人的初始位置
            robot_actions为机器人的动作序列,传入的是地址
            tasks是一个三维列表，第一个维度是机器人的编号，第二个维度是任务的队列，第三个维度存储了任务的信息，三个数据分别为任务的横纵坐标以及是否完成是否规划0表示未规划，1表示已经规划，2表示完成
            action_pointers存储了当前每个机器人的动作到了第几个任务
        2.调用next可以让所有的机器人按着序列移动
        3.其他函数均是私有函数

'''
class GridVisualization:
    UP = 'up'
    DOWN = 'down'
    LEFT = 'left'
    RIGHT = 'right'
    STAY = 'stay'
    def __init__(self, n, m, delivery_pos, robot_positions, robot_actions=None, tasks=None, action_pointers=None):
        self.n = n
        self.m = m
        self.delivery_pos = delivery_pos
        self.robots = robot_positions
        self.robot_actions = robot_actions if robot_actions is not None else [[] for _ in range(len(robot_positions))]
        self.tasks = tasks if tasks is not None else [[] for _ in range(len(robot_positions))]
        self.action_pointers = action_pointers if action_pointers is not None else [0] * len(robot_positions)
        self.task_added = False  # 新增任务标记
        self.new_tasks_this_step = []  # 记录当前步骤新增任务的机器人ID
        self.root = tk.Tk()
        self.canvas = tk.Canvas(self.root, width=m*50, height=n*50)
        self.canvas.pack()
        self.draw_grid()
        self.draw_robots()
        self.draw_delivery()
        
        # 初始化按钮
        self.btn_continue = tk.Button(self.root, text='Continue', command=self.on_continue)
        self.btn_continue.pack(side='left')
        self.btn_add = tk.Button(self.root, text='Add Mission', command=self.on_add_mission)
        self.btn_add.pack(side='right')
        
        self.next()
    
    def draw_grid(self):
        for i in range(self.n):
            for j in range(self.m):
                self.canvas.create_rectangle(j*50, i*50, (j+1)*50, (i+1)*50, outline='gray')
    
    def draw_robots(self):
        # 清除所有旧的任务标记
        #self.canvas.delete('mission')
            
        for idx, (x, y) in enumerate(self.robots):
            self.canvas.create_oval((y-1)*50+10, (x-1)*50+10, y*50-10, x*50-10, fill='blue', tags=f'robot_{idx}')
            self.canvas.create_text((y-1)*50+25, (x-1)*50+25, text=str(idx), fill='white')
            if self.robot_actions[idx]:
                action = self.robot_actions[idx][0]
                action_text = {'up':'↑', 'down':'↓', 'left':'←', 'right':'→','stay':'·'}[action]
                self.canvas.create_text((y-1)*50+25, (x-1)*50+40, text=f"{idx}:{action_text}", fill='blue', font=('Arial', 12))
            
            # 只绘制第一个未完成的任务
            for task_idx, (task_x, task_y, status) in enumerate(self.tasks[idx]):
                if status !=2:  # 未完成的任务
                    self.canvas.create_rectangle((task_y-1)*50+5, (task_x-1)*50+5, task_y*50-5, task_x*50-5, 
                                              outline='red', width=2, tags=f'mission')
                    self.canvas.create_text((task_y-1)*50+25, (task_x-1)*50+25, text=f"M:R{idx}", fill='red')
                    break
    
    def draw_delivery(self):
        x, y = self.delivery_pos
        self.canvas.create_rectangle((y-1)*50+5, (x-1)*50+5, y*50-5, x*50-5, fill='green')
    

    
    def update_robots(self, actions):
        for i in range(len(self.robots)):
            self.robot_actions[i].extend(actions[i] if isinstance(actions[i], list) else [actions[i]])
    
    def on_continue(self):
        task_added = False
        for i in range(len(self.robots)):
            if self.robot_actions[i] and self.action_pointers[i] < len(self.robot_actions[i]):
                action = self.robot_actions[i][self.action_pointers[i]]
                x, y = self.robots[i]
                if action == self.UP and x > 1:
                    self.robots[i] = (x-1, y)
                    self.action_pointers[i] += 1
                elif action == self.DOWN and x < self.n:
                    self.robots[i] = (x+1, y)
                    self.action_pointers[i] += 1
                elif action == self.LEFT and y > 1:
                    self.robots[i] = (x, y-1)
                    self.action_pointers[i] += 1
                elif action == self.RIGHT and y < self.m:
                    self.robots[i] = (x, y+1)
                    self.action_pointers[i] += 1
                elif action == self.STAY:
                    self.action_pointers[i] += 1
        self.canvas.delete('all')
        self.draw_grid()
        self.draw_robots()
        self.draw_delivery()
        # 重新绘制所有任务标记
        for robot_idx in range(len(self.tasks)):
            for idx, (x, y, status) in enumerate(self.tasks[robot_idx]):
                if status==0:
                    self.canvas.create_rectangle((y-1)*50+5, (x-1)*50+5, y*50-5, x*50-5, outline='red', width=2, tags=f'mission_{idx}')
                    self.canvas.create_text((y-1)*50+25, (x-1)*50+25, text=f'M:R{robot_idx}', fill='red')
                    break
            self.root.quit()
        return task_added
    
    def on_add_mission(self):
        task_added = False
        robot_idx = simpledialog.askinteger('添加任务', '输入机器人编号:', parent=self.root, minvalue=0, maxvalue=len(self.robots)-1)
        if robot_idx is not None:
            x = simpledialog.askinteger('添加任务', '输入目标x坐标:', parent=self.root, minvalue=1, maxvalue=self.n)
            y = simpledialog.askinteger('添加任务', '输入目标y坐标:', parent=self.root, minvalue=1, maxvalue=self.m)
            if x is not None and y is not None:
                if 0 <= robot_idx < len(self.tasks):
                    self.tasks[robot_idx].append((x, y, 0))
                    self.new_tasks_this_step.append(robot_idx)
                task_added = True
        self.task_added = task_added
        return task_added
    
    def next(self):
        self.root.mainloop()
        current_tasks = self.new_tasks_this_step.copy()
        self.new_tasks_this_step.clear()
        self.task_added = False  # 重置状态
        return current_tasks
    
    def run(self):
        self.root.mainloop()
    
    def complete_mission(self, robot_idx, mission_idx):
        if robot_idx < len(self.tasks) and mission_idx < len(self.tasks[robot_idx]):
            x, y, _ = self.tasks[robot_idx][mission_idx]
            self.tasks[robot_idx][mission_idx] = (x, y, 2)  # 标记为完成
            self.canvas.delete(f'mission_{mission_idx}')
            self.canvas.create_rectangle((y-1)*50+5, (x-1)*50+5, y*50-5, x*50-5, outline='green', width=2, tags=f'mission_{mission_idx}')
            text_id = self.canvas.create_text((y-1)*50+25, (x-1)*50+25, text=f'✓R{robot_idx}', fill='green')
            self.root.after(1000, lambda: self.canvas.delete(text_id))
            return True
        return False

    def __del__(self):
        if hasattr(self, 'root') and self.root:
            self.root.destroy()
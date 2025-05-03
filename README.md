# GridVisualization 可视化工具使用文档

## 类初始化参数
- `n`: 网格行数（Y轴方向）
- `m`: 网格列数（X轴方向）
- `delivery_pos`: 交付台位置元组 (x,y)
- `robot_positions`: 机器人初始位置列表 [(x1,y1), (x2,y2)...]
- `robot_actions`: 可选参数，机器人初始动作队列
- `tasks`: 可选参数，初始任务列表
- `action_pointers`: 可选参数，动作执行指针

## 主要方法
### `on_continue()`
执行机器人队列中的下一个动作，更新机器人位置并刷新画布

### `on_add_mission()`
弹出对话框添加新任务：
1. 选择机器人编号
2. 输入目标坐标(x,y)
3. 任务会显示为红色方框标记

### `complete_mission(robot_idx, mission_idx)`
标记指定机器人的任务为完成状态，对应任务标记变绿

## 界面说明
- **交付台**：绿色方块
- **机器人**：蓝色圆形+编号
- **当前动作**：机器人下方黄色方向箭头
- **未完成任务**：红色方框+M:R{编号}
- **按钮**：
  - Continue：逐步执行动作
  - Add Mission：添加新任务

## 使用示例
```python
# 初始化3x3网格，交付台在(2,2)
viz = GridVisualization(
    n=3,
    m=3,
    delivery_pos=(2,2),
    robot_positions=[(1,1)],
    robot_actions=[["right", "down"]]
)

# 添加任务到0号机器人
viz.tasks[0].append((3,3,0))

# 运行可视化界面
viz.run()
```
> 注意：坐标系统以左上角为(1,1)，x向下增长，y向右增长
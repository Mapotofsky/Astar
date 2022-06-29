import numpy as np
import time


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.f = 0

    def setF(self, f):
        self.f = f

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    # g为到起点最短路径代价的估计值
    def getG(self, start):
        dist = abs(self.x - start.x) + abs(self.y - start.y)
        self.g = dist

    # h为到终点最短路径代价的估计值，使用曼哈顿距离或欧式距离计算
    def getH(self, end, flag=False):
        if flag:  # 曼哈顿距离
            dist = abs(self.x - end.x) + abs(self.y - end.y)
            dist *= 10
        else:     # 欧式距离
            dist = pow(self.x - end.x, 2) + pow(self.y - end.y, 2)

        self.h = dist

    # 启发函数 f = g + h
    def getFunctionValue(self, start, end):
        self.getG(start)
        self.getH(end)
        self.f = self.g + self.h

        return self.f


class State:
    def __init__(self, state, current_point=Point(0, 0), start_point=Point(0, 0), end_point=Point(0, 0)):
        self.state = state
        self.cP = current_point
        self.sP = start_point
        self.eP = end_point

    def __eq__(self, other):
        return self.cP == other.cP

    def setF(self, f):
        self.f = f

    def setCurrentPoint(self, x, y):
        self.cP.x = x
        self.cP.y = y

    def getCurPoint(self):
        return self.cP.x, self.cP.y


# 确定下一步的方法
def nextStep(map, openTable, closeTable, wrongTable, generate):
    subPoints = []
    boarder = len(map.state) - 1
    # 获取当前所在的点
    x, y = map.getCurPoint()
    # 往左走
    if y > 0 and map.state[x][y - 1] == 0:
        p = Point(x, y - 1)
        if p not in closeTable and p not in wrongTable:
            # 添加到可以走的list
            openTable.append(p)
            # new point
            # 获取F函数值
            p.setF(p.getFunctionValue(map.sP, map.eP))
            subPoints.append(p)
            generate[0] += 1
    # 往上走
    if x > 0 and map.state[x - 1][y] == 0:
        p = Point(x - 1, y)
        if p not in closeTable and p not in wrongTable:
            openTable.append(p)
            p.setF(p.getFunctionValue(map.sP, map.eP))
            subPoints.append(p)
            generate[0] += 1
    # 往下走
    if x < boarder and map.state[x + 1][y] == 0:
        p = Point(x + 1, y)
        if p not in closeTable and p not in wrongTable:
            openTable.append(p)
            p.setF(p.getFunctionValue(map.sP, map.eP))
            subPoints.append(p)
            generate[0] += 1
    # 往右走
    if y < boarder and map.state[x][y + 1] == 0:
        p = Point(x, y + 1)
        if p not in closeTable and p not in wrongTable:
            openTable.append(p)
            p.setF(p.getFunctionValue(map.sP, map.eP))
            subPoints.append(p)
            generate[0] += 1
    # 根据F值排序，获取F值最近的
    subPoints.sort(key=compareF)
    if len(subPoints) < 1:
        # 防止走到死路无法回头情况
        wrongTable.append(Point(map.cP.x, map.cP.y))
        closeTable.remove(map.cP)
        next_point = closeTable[-1]
        map.cP.x, map.cP.y = next_point.x, next_point.y
    else:
        next_point = subPoints[0]
        map.cP.x, map.cP.y = next_point.x, next_point.y
        closeTable.append(next_point)
        openTable.remove(next_point)


# 迭代走下一步
def solve(map, openTable, closeTable, wrongTable, generate):
    # start the loop
    while not map.cP == map.eP:
        nextStep(map, openTable, closeTable, wrongTable, generate)


def compareF(p):
    return p.f


# 展示最后结果
def showInfo(map, path):
    for i in range(len(map.state)):
        for j in range(len(map.state)):
            if Point(i, j) in path:
                # 正确路径用‘*’表示
                print('*', end='  ')
            else:
                print(map.state[i, j], end='  ')
        print("\n")
    return


if __name__ == '__main__':
    start = time.clock()

    # openList
    openTable = []
    # closeList
    closeTable = []
    # 走错路返回用的
    wrongTable = []
    # 0表示无障碍，1表示有障碍
    state = np.array([[0, 0, 0, 0, 0],
                      [1, 0, 1, 0, 1],
                      [0, 0, 0, 0, 1],
                      [0, 1, 0, 0, 0],
                      [0, 0, 0, 1, 0]])
    '''
    state = np.array([[0, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1],
                      [0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 1],
                      [0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 1, 1],
                      [0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 1],
                      [1, 1, 1, 1, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1],
                      [0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0, 0, 1],
                      [0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1],
                      [1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1],
                      [0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1],
                      [0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1, 1, 0, 1, 0],
                      [0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1],
                      [0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1],
                      [0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1],
                      [1, 1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                      [0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                      [0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                      [0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                      [1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0],
                      [0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0]])'''
    # 起点终点
    start_point = Point(0, 0)
    end_point = Point(4, 4)
    # 最终路径
    generate = [0]
    path = [start_point]
    Map = State(state, Point(0, 0), start_point, end_point)
    solve(Map, openTable, closeTable, wrongTable, generate)
    print('Best Way:')
    path = path + closeTable
    showInfo(Map, path)
    print("Total steps is %d" % (len(path) - 1))
    print(f'generate node: {generate[0]}')

    end = time.clock()
    print(f"time is {end - start}")

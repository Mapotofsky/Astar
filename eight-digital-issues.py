import numpy as np
import time


class State:
    def __init__(self, state, answer, directionFlag=None, parent=None):
        self.state = state
        self.answer = answer
        self.direction = ['up', 'down', 'right', 'left']
        if directionFlag:
            self.direction.remove(directionFlag)
        self.parent = parent
        self.getFunctionValue()

    def getDirection(self):
        return self.direction

    def setF(self, f):
        self.f = f
        return

    # 打印结果
    def showInfo(self):
        for i in range(len(self.state)):
            for j in range(len(self.state)):
                print(self.state[i, j], end='  ')
            print("\n")
        print('->')
        return

    # 获取0点
    def getZeroPos(self):
        postion = np.where(self.state == 0)
        return postion

    # g为深度
    def getG(self):
        if (self.parent is not None):
            self.g = self.parent.g + 1
        else:
            self.g = 0

    # h为到目标状态的距离，使用曼哈顿距离之和或不在位数计算
    def getH(self, flag=True):
        cur_node = self.state.copy()
        fin_node = self.answer.copy()
        dist = 0
        N = len(cur_node)

        if flag:  # 曼哈顿距离
            for i in range(N):
                for j in range(N):
                    if cur_node[i][j] != fin_node[i][j]:
                        index = np.argwhere(fin_node == cur_node[i][j])
                        x = index[0][0]  # 最终x距离
                        y = index[0][1]  # 最终y距离
                        dist += (abs(x - i) + abs(y - j))
        else:     # 不在位数
            for i in range(N):
                for j in range(N):
                    if cur_node[i][j] != fin_node[i][j]:
                        dist += 1

        self.h = dist

    # 启发函数 f = g + h
    def getFunctionValue(self):
        self.getG()
        self.getH()
        self.f = self.g + self.h

        return self.f

    def nextStep(self, generate):
        if not self.direction:
            return []  # direction为空时返回空列表
        subStates = []
        boarder = len(self.state) - 1
        # 获取0点位置
        x, y = self.getZeroPos()
        # 向左
        if 'left' in self.direction and y > 0:
            s = self.state.copy()
            a = self.answer.copy()
            tmp = s[x, y - 1]
            s[x, y - 1] = s[x, y]
            s[x, y] = tmp
            news = State(s, a, directionFlag='right', parent=self)
            news.setF(news.getFunctionValue())
            subStates.append(news)
            generate[0] += 1
        # 向上
        if 'up' in self.direction and x > 0:
            # it can move to upper place
            s = self.state.copy()
            a = self.answer.copy()
            tmp = s[x - 1, y]
            s[x - 1, y] = s[x, y]
            s[x, y] = tmp
            news = State(s, a, directionFlag='down', parent=self)
            news.setF(news.getFunctionValue())
            subStates.append(news)
            generate[0] += 1
        # 向下
        if 'down' in self.direction and x < boarder:
            # it can move to down place
            s = self.state.copy()
            a = self.answer.copy()
            tmp = s[x + 1, y]
            s[x + 1, y] = s[x, y]
            s[x, y] = tmp
            news = State(s, a, directionFlag='up', parent=self)
            news.setF(news.getFunctionValue())
            subStates.append(news)
            generate[0] += 1
        # 向右
        if self.direction.count('right') and y < boarder:
            # it can move to right place
            s = self.state.copy()
            a = self.answer.copy()
            tmp = s[x, y + 1]
            s[x, y + 1] = s[x, y]
            s[x, y] = tmp
            news = State(s, a, directionFlag='left', parent=self)
            news.setF(news.getFunctionValue())
            subStates.append(news)
            generate[0] += 1
        # 返回F值最小的下一个点
        subStates.sort(key=lambda state: state.f)
        return subStates

    # A* 迭代
    def solve(self, generate):
        # openList
        openTable = []
        # closeList
        closeTable = []

        openTable.append(self)
        while openTable:
            # 对openTable表按f值升序排列
            openTable.sort(key=lambda S: S.f)
            # 下一步的点移除open
            n = openTable.pop(0)
            # 加入close
            closeTable.append(n)
            path = []
            # 判断是否和最终结果相同
            if (n.state == n.answer).all():
                while n.parent and n.parent != originState:
                    path.append(n.parent)
                    n = n.parent
                path.reverse()
                return path

            # 扩展结点
            subStates = n.nextStep(generate)
            for stat in subStates:
                # 查看扩展出的状态是否已经存在于openTable或closeTable中
                findstat = isin(stat, openTable)
                findstat2 = isin(stat, closeTable)
                # 在closeTable中,判断是否更新
                if (findstat2[0] and stat.f < closeTable[findstat2[1]].f):
                    closeTable[findstat2[1]] = stat
                    openTable.append(stat)
                # 在openTable中，判断是否更新
                if (findstat[0] and stat.f < openTable[findstat[1]].f):
                    openTable[findstat[1]] = stat
                # stat状态不在openTable中，也不在closeTable中
                if (not findstat[0] and not findstat2[0]):
                    openTable.append(stat)
        else:
            return None, None  # 循环最后输出空值


# 判断状态s是否在状态集合中，s是对象，sList是对象列表
# 返回的结果是一个列表，第一个值是真假，如果是真则第二个值是g在gList中的位置索引
def isin(s, sList):
    state = s.state.copy()
    state = state.tolist()
    statList = []
    for i in sList:
        sta = i.state.copy()
        sta = sta.tolist()
        statList.append(sta)
    if(state in statList):
        res = [True, statList.index(state)]
    else:
        res = [False, 0]
    return res


if __name__ == '__main__':
    start = time.clock()

    # 前面是初始状态，后面是结束状态
    originState = State(np.array([[2, 0, 8], [1, 6, 3], [7, 5, 4]]),
                        np.array([[1, 2, 3], [8, 0, 4], [7, 6, 5]]))

    s1 = State(state=originState.state,
               answer=originState.answer)
    generate = [0]
    path = s1.solve(generate)
    if path:
        for node in path:
            node.showInfo()
        print(s1.answer)
        print("Total steps is %d" % len(path))
        print(f'generate node: {generate[0]}')

    end = time.clock()
    print(f"time is {end - start}")

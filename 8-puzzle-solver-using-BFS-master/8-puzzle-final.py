from operator import truediv
import matplotlib.pyplot as plt
import numpy as np
import os

class Node:
    def __init__(self, mat, stateId, parentId = 0, parentVal=None):
        self.headVal = None
        self.mat = mat
        self.parentId = parentId
        self.stateId = stateId
        self.parentVal = parentVal
    
class BFS:
    def __init__(self, start, goal, f, g, h):
        self.start = start
        self.goal = goal
        self.queue = []
        self.visited = []
        self.node_id = 1
        self.f = f
        self.g = g
        self.h = h
        
    def isValid(self, i, j, mat):
        if i >= 0 and i < len(mat) and j >= 0 and j < len(mat[0]):
            return True
        return False

    def swapMat(self, mat,  x, y, i, j, parentId_, parentVal_):
        mat_ = np.copy(mat).tolist()
        if not self.isValid(x, y, mat_):
            return
        mat_[i][j], mat_[x][y] = mat_[x][y], mat_[i][j]
        if self.visited.count(mat_) == 0:
            self.node_id += 1
            return Node(np.asarray(mat_), self.node_id, parentId_, parentVal_)
        return None

    def ActionMoveLeft(self, mat, x, y, i, j, parentId_, parentVal_, possib_list):
        node_address = self.swapMat(mat,  x, y, i, j, parentId_, parentVal_)
        if node_address != None:
            possib_list.append(node_address)
        return possib_list
            
    def ActionMoveRight(self, mat, x, y, i, j, parentId_, parentVal_, possib_list):
        node_address = self.swapMat(mat,  x, y, i, j, parentId_, parentVal_)
        if node_address != None:
            possib_list.append(node_address)
        return possib_list
        
    def ActionMoveUp(self, mat, x, y, i, j, parentId_, parentVal_, possib_list):
        node_address = self.swapMat(mat,  x, y, i, j, parentId_, parentVal_)
        if node_address != None:
            possib_list.append(node_address)
        return possib_list

    def ActionMoveDown(self, mat, x, y, i, j, parentId_, parentVal_, possib_list):
        node_address = self.swapMat(mat,  x, y, i, j, parentId_, parentVal_)
        if node_address != None:
            possib_list.append(node_address)
        return possib_list
        
    def possibilities(self, mat, parentId_, parentVal_):
        i = np.where(mat==0)[0][0]
        j = np.where(mat==0)[1][0]
        
        possib_list = []
        coordinate_list = [[i+1, j], [i-1, j], [i, j+1], [i, j-1]]
        
        possib_list = self.ActionMoveLeft(mat, coordinate_list[0][0], coordinate_list[0][1], i, j, parentId_, parentVal_, possib_list)
        possib_list = self.ActionMoveRight(mat, coordinate_list[1][0], coordinate_list[1][1], i, j, parentId_, parentVal_, possib_list)
        possib_list = self.ActionMoveUp(mat, coordinate_list[2][0], coordinate_list[2][1], i, j, parentId_, parentVal_, possib_list)
        possib_list = self.ActionMoveDown(mat, coordinate_list[3][0], coordinate_list[3][1], i, j, parentId_, parentVal_, possib_list)
            
        return possib_list
    
    def printNodePath(self, list_):
        print(list_)
        while len(list_) > 0:
            data = list_.pop()
            for i in range (3):
                for j in range (3):
                    self.f.write("%d " % data[i][j])
            self.f.write("\n") 

    def generate_path(self,node):
        idlist = []
        parent_mat_list = []
        while node.parentId != 0:
            idlist.append(node.parentId)
            parent_mat_list.append(node.mat.T.tolist())
            node = node.parentVal
        parent_mat_list.append(self.start.T.tolist())
        self.printNodePath(parent_mat_list)
        return idlist

    def bfs(self):
        root = Node(self.start, 1)
        self.queue.append(root)
        self.visited.append(root.mat.tolist())
        while len(self.queue) != 0:
            possib_list = self.possibilities(self.queue[0].mat, self.queue[0].stateId, self.queue[0])
            self.visited.append(self.queue[0].mat.tolist())
            self.g.write("%d\t\t\t" % self.queue[0].stateId)
            self.g.write("%d\n" % self.queue[0].parentId)
            print(np.reshape(self.queue[0].mat,(1,9))[0])
            self.h.writelines(str(np.reshape(self.queue[0].mat,(1,9))[0])[1:18])
            self.h.write("\n") 
            self.queue.pop(0)
            for data in possib_list:
                if data.mat.tolist() == self.goal.tolist():
                    print(data.mat.tolist())
                    print("goal_reached")
                    idlist_ = self.generate_path(data)
                    print(idlist_)
                    return
                if self.visited.count(data.mat.tolist()) == 0:
                   self.queue.append(data)    
            

f1= open("nodePath1.txt","w+")
g1= open("NodesInfo1.txt","w+")
h1= open("Nodes1.txt","w+")

g1.write("Node_index\t")
g1.write("Parent_Node_index\n")

start1 = np.array([1, 4, 7, 0, 2, 8, 3, 5, 6])
goal1 = np.array([1, 4, 7, 2, 5, 8, 3, 6, 0])
start1 = np.reshape(start1, (3, 3)).T 
goal1 = np.reshape(goal1, (3, 3)).T

a = BFS(start1, goal1, f1, g1, h1)
a.bfs()
f1.close()
g1.close()
# start = np.array([1, 0, 3, 4, 2, 5, 7, 8, 6])
# goal = np.array([1, 2, 3, 4, 5, 6, 7, 8, 0])

# start = np.array([1, 4, 7, 8, 2, 6, 0, 3, 5])
# goal = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8])
f2= open("nodePath2.txt","w+")
g2= open("NodesInfo2.txt","w+")
h2= open("Nodes2.txt","w+")

g2.write("Node_index\t")
g2.write("Parent_Node_index\n")

start2 = np.array([1, 0, 3, 4, 2, 5, 7, 8, 6])
goal2 = np.array([1, 2, 3, 4, 5, 6, 7, 8, 0])
start2 = np.reshape(start2, (3, 3)).T 
goal2 = np.reshape(goal2, (3, 3)).T
b = BFS(start2, goal2,f2, g2, h2)
b.bfs()

f2.close()
g2.close()

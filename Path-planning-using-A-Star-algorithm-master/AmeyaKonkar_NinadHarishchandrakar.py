from multiprocessing import parent_process
import numpy as np
import math
import cv2

class Node:
    def __init__(self, f_, g_, theta_,  parent_):
        
        self.theta = theta_
        self.f = f_
        self.g = g_
        self.parent = parent_


def free_space(c, r):
    space = np.ndarray((800,500),dtype=Node)
 
    for x in range(800):
        for y in range(500):
            parent = np.array([-1, -1])
            space[x, y] = Node(np.inf, 0, 0,  parent)
            if ((x - 300*2)**2 + (y - 185*2)**2 < (((40+((c+r)*2))*2)**2)) or (((0.5773502691896258 * x) + y < (255.88457268119896*2 + ((((c+r)*2))*(math.sqrt(0.5773502691896258**2 + 1))))) and (x < (235*2 + (((c+r)*2)))) and ((-0.5773502691896256 * x) + y > (-55.88457268119893*2 - ((((c+r)*2))*(math.sqrt(0.5773502691896256**2 + 1))))) and ((0.5773502691896256 * x) + y > (175.05553499465134*2 - ((((c+r)*2))*(math.sqrt(0.5773502691896256**2 + 1))))) and (x > (165*2 - (((c+r)*2)))) and ((-0.5773502691896258 * x) + y < (24.944465005348647*2 + ((((c+r)*2))*(math.sqrt(0.5773502691896258**2 + 1)))))) or (((1.2318840579710144 * x) + y > (229.3478260869565*2 - ((((c+r)*2))*(math.sqrt(1.2318840579710144**2 + 1))))) and ((-0.31645569620253167 * x) + y < (173.60759493670886*2 + ((((c+r)*2))*(math.sqrt(0.31645569620253167**2 + 1))))) and (((-0.8571428571428571 * x) + y > (111.42857142857143*2 - ((((c+r)*2))*(math.sqrt(0.8571428571428571**2 + 1))))) or ((3.2 * x) + y < (436.0*2 + ((((c+r)*2))*(math.sqrt(3.2**2 + 1))))))) or (x <= (((c+r)*2))) or (x >= (800 - (((c+r)*2)))) or (y <= (((c+r)*2))) or (y >= (500 - (((c+r)*2)))):
                space[x, y] = Node(-1, 0, 0, parent)
                
    return space


def visualize(c):

    vis = np.zeros((800, 500, 3), np.uint8)

    for x in range(800):
        for y in range(500):

            if ((x - 300*2)**2 + (y - 185*2)**2 < ((80+(c*2))**2)) or (((0.5773502691896258 * x) + y < (255.88457268119896*2 + (((c*2))*(math.sqrt(0.5773502691896258**2 + 1))))) and (x < (235*2 + ((c*2)))) and ((-0.5773502691896256 * x) + y > (-55.88457268119893*2 - (((c*2))*(math.sqrt(0.5773502691896256**2 + 1))))) and ((0.5773502691896256 * x) + y > (175.05553499465134*2 - (((c*2))*(math.sqrt(0.5773502691896256**2 + 1))))) and (x > (165*2 - ((c*2)))) and ((-0.5773502691896258 * x) + y < (24.944465005348647*2 + (((c*2))*(math.sqrt(0.5773502691896258**2 + 1)))))) or (((1.2318840579710144 * x) + y > (229.3478260869565*2 - (((c*2))*(math.sqrt(1.2318840579710144**2 + 1))))) and ((-0.31645569620253167 * x) + y < (173.60759493670886*2 + (((c*2))*(math.sqrt(0.31645569620253167**2 + 1))))) and (((-0.8571428571428571 * x) + y > (111.42857142857143*2 - (((c*2))*(math.sqrt(0.8571428571428571**2 + 1))))) or ((3.2 * x) + y < (436.0*2 + (((c*2))*(math.sqrt(3.2**2 + 1))))))) or (x <= ((c*2))) or (x >= (800 - ((c*2)))) or (y <= ((c*2))) or (y >= (500 - ((c*2)))):

                vis[x][y] = (0, 255, 255)


            if ((x - 300*2)**2 + (y - 185*2)**2 < (40*2)**2) or (((0.5773502691896258 * x) + y < 255.88457268119896*2) and (x < 235*2) and ((-0.5773502691896256 * x) + y > -55.88457268119893*2) and ((0.5773502691896256 * x) + y > 175.05553499465134*2) and (x > 165*2) and ((-0.5773502691896258 * x) + y < 24.944465005348647*2) or ((1.2318840579710144 * x) + y > 229.3478260869565*2) and ((-0.31645569620253167 * x) + y < 173.60759493670886*2) and (((-0.8571428571428571 * x) + y > 111.42857142857143*2) or ((3.2 * x) + y < 436.0*2))):

                vis[x][y] = (0, 0, 255)

    return vis


def isValid(point, space):
    if (not (0 < point[0] < 800)) or (not (0 < point[1] < 500)) or (space[point[0], point[1]].f == -1):
        return False
    return True


def isDestination(point, goal):
    return np.array_equal(point, goal)


def childnodes(x, y, theta, L, src, goal, space):

    childnodes = []
    parent_g = space[x, y].g

    for i in range(-60, 61, 30):
        childx = round(x + L*(math.cos(math.radians(theta + i))))
        childy = round(y + L*(math.sin(math.radians(theta + i))))
        childtheta = theta + i
        childcost, g = heuristic(childx, childy, goal, parent_g, x, y)
        childnodes.append([childx, childy, childtheta, childcost, g])

    return childnodes

def heuristic(x, y, goal, parent_g, p_x, p_y):

    h = (math.sqrt(((x - goal[0])**2) + ((y - goal[1])**2)))
    g = parent_g + (math.sqrt(((x - p_x)**2) + ((y - p_y)**2)))

    return (h + g), g

def checkThreshold(point, goal):
    threshold = 4*2
    return (point[0] - goal[0])*(point[0] - goal[0]) + (point[1] - goal[1])*(point[1] - goal[1]) <= threshold*threshold

def A_star(src, goal, length, clearance, radius):

    vis = visualize(clearance)
    cv2.circle(vis, (src[1], src[0]), 3, (203, 192, 255), -1)
    cv2.circle(vis, (goal[1], goal[0]), 3, (203, 192, 255), -1)
   
    src_x = src[0]
    src_y = src[1]
    
    space = free_space(clearance, radius)
    space[src_x, src_y].f = 0
    space[src_x, src_y].g = 0
    space[src_x, src_y].theta = src[2]
    space[src_x, src_y].parent = np.array([src_x, src_y])

    openList = {}
    openList[(src_x, src_y)] = space[src_x, src_y].f
    closedList = np.zeros((800, 500))

    while(not len(openList) == 0):

        point = min(openList, key=openList.get)
        openList.pop(point, None)

        i = point[0]
        j = point[1]
        theta = space[i, j].theta
        successorList = childnodes(i, j, theta, length, src, goal, space)
        closedList[i, j] = 1
        
        for data in successorList:
            new_i = data[0]
            new_j = data[1]
            theta_new = data[2]
            f_new = data[3]
            g_new = data[4]

            cur_point = np.array([new_i, new_j])
            
            if(isValid(cur_point, space)):
                if(checkThreshold(cur_point, goal[0:2])):
                    space[new_i, new_j].parent = np.array([i, j])
                    traverseList = backtrack(cur_point, src, space, goal[0:2])
                    print()
                    print("-------------------------------------")
                    print("Optimal Path Found, Displaying Output")
                    print("-------------------------------------")
                    visualize_path(traverseList, vis, radius)
                    print()
                    print("-------------------------------------")
                    print("Destination Reached")
                    print("-------------------------------------")
                    return 

                elif closedList[new_i, new_j] == 0:
                    if space[new_i, new_j].f == np.inf or space[new_i, new_j].f > f_new:
                        openList[new_i, new_j] = f_new
                        space[new_i, new_j].f = f_new
                        space[new_i, new_j].g = g_new
                        space[new_i, new_j].theta = theta_new
                        space[new_i, new_j].parent = np.array([i, j])
                        cv2.line(vis, (new_j, new_i) , (j, i), (255, 255, 255), 1)
        vis = cv2.rotate(vis, cv2.ROTATE_90_COUNTERCLOCKWISE)

        cv2.imshow('A*', vis)
        if cv2.waitKey(1) & 0xFF == ord('q'):
                return 0

        vis = cv2.rotate(vis, cv2.ROTATE_90_CLOCKWISE)


def backtrack(cur_point, src, space, goal):
    space_ = space.copy()
    x = cur_point[0].copy()
    y = cur_point[1].copy()

    traverseList = []
    traverseList.append((goal[0], goal[1]))
    while(not (x == src[0] and y == src[1])):
        temp = space_[x,y].parent[0].copy()
        y = space_[x,y].parent[1].copy()
        x = temp
        traverseList.append((x,y))

    traverseList.append((src[0], src[1]))
    traverseList.reverse()

    return traverseList


def visualize_path(traverseList, vis, radius):

    while(len(traverseList) != 0):
        vis_ = vis.copy()
        y = traverseList.pop(0)[0]
        x = traverseList.pop(0)[1]
        cv2.circle(vis_, (x, y) , radius, (0, 255, 0), -1)
        vis_ = cv2.rotate(vis_, cv2.ROTATE_90_COUNTERCLOCKWISE)
        cv2.imshow('A*', vis_)
        if cv2.waitKey(100) & 0xFF == ord('q'):
                return 0
        vis_ = cv2.rotate(vis_, cv2.ROTATE_90_CLOCKWISE)


def user_input():

    src_x = 2*int((input("Enter x coordinaate of source point: ")))
    src_y = 2*int((input("Enter y coordinaate of source point: ")))
    src_theta = int((input("Enter orientation at source point: ")))

    goal_x = 2*int((input("Enter x coordinaate of goal point: ")))
    goal_y = 2*int((input("Enter y coordinaate of goal point: ")))
    goal_theta = int((input("Enter orientation at goal point: ")))

    length = 2*int((input("Enter length of step between 0 and 10: ")))
    clearance = 2*int((input("Enter clearance space width: ")))
    radius = int((input("Enter radius of robot: ")))


    space = free_space(clearance, radius)
    if not (isValid([src_x, src_y], space) and isValid([goal_x, goal_y], space)):
        return None, None, None, None, None 

    return np.array([src_x, src_y, src_theta]), np.array([goal_x, goal_y, goal_theta]), length, clearance, radius


def main():

    src, goal, length, clearance, radius = user_input()

    if(length == None):
        print("Invalid input, try again")
        return

    A_star(src, goal, length, clearance, radius)

    
main()
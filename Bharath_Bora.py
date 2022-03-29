import math
import heapq as hq
from matplotlib import pyplot as plt
import numpy as np


#Action set 
Action_set = [60,30,0,-30,-60]
          
class Node:
    def __init__(self,x,y,cost,angle,parent_co):
        self.x = x
        self.y = y
        self.cost = cost
        self.angle = angle
        self.parent_co = parent_co
        self.co = (x,y)
    def __lt__(self,other):
        return self.cost < other.cost


def back_track():
     
    path_list.append(current_node.co)
    parent = info_dict.get(current_node.co)

    while parent != None:  

      path_list.append(parent)

      parent = info_dict.get(parent)

#inputs
p = int(input("enter the initial node x co-ordinate (0 to 400) "))
q = int(input("enter the initial node y co-ordinate (0 to 250) "))
theta_s = int(input("enter the angle of initial node (0 to 360) "))
r = int(input("enter the goal node x co-ordinate (0 to 400) " ))
s = int(input("enter the goal node y co-ordinate (0 to 250) "))
theta_g = int(input("enter the angle of goal node (0 to 360) "))
k = int(input("enter step size (1 to 10) "))

if( p<0 or p>400 or q<0 or q>250 or r<0 or r>400 or s<0 or s>250 or theta_s > 360 or theta_g >360 or k >10 ):
    print("Inputs are out of bounds")
    quit()

#defining lists and dictionaries
path_list = []    
v_map = np.zeros([500,800,12], dtype = int)
info_dict = {}   #child-->parent
dupe_dict = {}   #co-->node
open_list =[]


cost = math.sqrt( (r-p)**2 + (s-q)**2 )
initial_node = Node(p,q,cost,theta_s,None)
goal_node = Node(r,s,None,theta_g,None)

hq.heappush(open_list, [initial_node.cost, initial_node])
dupe_dict[initial_node.co] = initial_node

#map with obstacles 
map = np.zeros([250,400], dtype =int)
for y in range(250):
    for x in range(400):
        #circle obstale
        c = ((x-300)**2) + ((y-185)**2) - 1600

        #polygon obstacle
        s1 = y + (3.2 * x) - 436
        s2 = y + (1.23 * x) - 229.34
        s3 = y + (0.1136 * x) - 189.09
        s4 = y - (0.316 * x) - 173.608 
        s5 = y - 0.857 * x - 111.42  

        #hexagon obstacle
        h1 = y - (0.577 * x) - 24.97 
        h2 = y + (0.577 * x) - 255.82
        h3 = x - 235 
        h6 = x - 165 
        h5 = y + (0.577 * x) - 175 
        h4 = y - (0.577 * x) + 55.82 

        if( c<=0 or (s1<=0 and s2>=0 and s3<=0)  or (s4<=0 and s5>=0 and s3>=0) or (h1<=0 and h2<=0 and h3<=0 and h4>=0 and h5>=0 and h6>=0) ):
            map[y,x] = 1

#checking whether the co-ordinates are in obstacle space 
def is_in_obstacle_space(x,y):
    #circle obstale
        c = ((x-300)**2) + ((y-185)**2) - 1600

        #polygon obstacle
        s1 = y + (3.2 * x) - 436
        s2 = y + (1.23 * x) - 229.34
        s3 = y + (0.1136 * x) - 189.09
        s4 = y - (0.316 * x) - 173.608 
        s5 = y - 0.857 * x - 111.42  

        #hexagon obstacle
        h1 = y - (0.577 * x) - 24.97 
        h2 = y + (0.577 * x) - 255.82
        h3 = x - 235 
        h6 = x - 165 
        h5 = y + (0.577 * x) - 175 
        h4 = y - (0.577 * x) + 55.82 

        if( c<=0 or (s1<=0 and s2>=0 and s3<=0)  or (s4<=0 and s5>=0 and s3>=0) or (h1<=0 and h2<=0 and h3<=0 and h4>=0 and h5>=0 and h6>=0) ):
            return True
        else:
            return False

#exits if start node or goal node is in obstacle space 
if(map[initial_node.y,initial_node.x] == 1  or map[goal_node.y,goal_node.x]==1):
    print("Failed!!! start node or goal node is in obstacle space")
    quit()


while( open_list != [] ):

    current_node = (hq.heappop(open_list))[1]
    
    v_map[int(current_node.y * 2)][int(current_node.x * 2)][int(current_node.angle/30)] = 1
    info_dict[current_node.co] = current_node.parent_co

    reach = math.sqrt( ((goal_node.x - current_node.x)**2) + ((goal_node.y - current_node.y)**2) )
    if( reach <= k*1.5 ):
        print("Found Goal Node!!")
        print("Backtracking...")
        back_track()
        break

    for an in Action_set:

        theta = an + current_node.angle
        x_co = round(  k*(math.cos(math.radians(theta))) ) 
        y_co = round( k*(math.sin(math.radians(theta))) ) 

        c2c = math.sqrt( ((x_co - current_node.x)**2) + ((y_co - current_node.y)**2) )
        c2g = math.sqrt( ((goal_node.x - x_co)**2)+ ((goal_node.y - y_co)**2) )
        t_cost = round(c2c + c2g)

        check = is_in_obstacle_space(x_co,y_co)
        if(x_co > 0 and x_co < 400 and y_co > 0 and y_co < 250 ): in_range = True
        else: in_range = False

        if(check == False and in_range == True and v_map[int(y_co *2)][int(x_co*2)][int(theta/30)] != 1):
            
            if (x_co,y_co) not in dupe_dict.keys():
                
                new_node = Node(x_co,y_co,t_cost,theta,current_node.co)
                dupe_dict[new_node.co] = new_node
                hq.heappush(open_list,[cost,new_node])

            else:
                get_node = dupe_dict.get((x_co,y_co))
                if(get_node.cost > t_cost):
                    get_node.cost = t_cost
                    get_node.parent_co = current_node.co
                    get_node.angle = theta
                    dupe_dict[(x_co,y_co)] = get_node





# -*- coding: utf-8 -*-
"""
Created on Sun Mar 20 13:00:00 2022

@author: 91834
"""

import numpy as np
import math
import heapq
import time
import cv2
import pygame

#Getting the start time to measure the time taken for solving
    
 ####################User Inputs######################   
x_start= int(input("Enter the x coordinate of the start:  "))
y_start= int(input("Enter the y coordinate of the start:  "))

start_orientation = int(input("Enter the Orientation at start (enter in multiples of 30 degreees and less that 360 degrees), :  "))
x_goal= int(input("Enter the x coordinate of the goal:  "))
y_goal= int(input("Enter the y coordinate of the goal:  "))

step_size= int(input("Enter the step (1-10): "))
start = (x_start,y_start)
goal = (x_goal,y_goal)
#######################################################


#function to round the point 5

def Round2Point5(num):
    return (round(num * 2) / 2)


#for solving the path using eucledian heuristic
    
def EucledianDistance(a,b):  
    x1 = a[0]
    x2 = b[0]
    y1 = a[1]
    y2 = b[1]
    dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)  
    return dist  


#eucledian heuristic as for caclulation of the value of cost to go to the goal.

#####

#function to move any direction based on the degree
def ActionMove(curr_node,degree,step_size):
    x = curr_node[0]
    y = curr_node[1]
    x_new = (step_size)*np.cos(np.deg2rad(degree)) + x# - (y-0)*np.sin(np.deg2rad(degree)) + length
    y_new = (step_size)*np.sin(np.deg2rad(degree)) + y# + (y-0)*np.cos(np.deg2rad(degree)) + length
    new_node = (round(x_new,2),round(y_new,2))
    if new_node[0]>=0.00 and new_node[0]<=400.00 and new_node[1]>=0.00 and new_node[1]<=250.00:
        return(new_node,True)
    else:
        return(curr_node,False)
    
        
        
######################################     POINTS FOR REMOVING VALUES FROM THE GRAPH
        
#calculating all the points within the entire canvas
        
all_possible_points = []
for i in range(0,401): 
    for j in range(0,251):
        all_possible_points.append((Round2Point5(i),Round2Point5(j))) #appending
        
#all possible points in the obstacle space
list_of_all_points=[]
clearance_points =[]

for pt in all_possible_points:
    
      x = pt[0]
      y = pt[1]
    
#circle shaped obstacle with 5mm clearance and 10mm radius
      if((x-300)**2 + (y-185)**2 <= (55)**2):
          list_of_all_points.append((x,y))
      
      #polygon shaped obstacle with 5mm clearance and 10mm radius
      if y - (0.316)*x <=188.607:
          if y+(1.232)*x >= 214.348:
              if y+(3.2)*x <=451 or y-(0.857)*x >=96.429:
                  list_of_all_points.append((x,y))
     
      #hexagon obstacle with 5mm clearance and 10mm radius
      if x>=150 and x<=250 and y+(0.577)*x<=270.883 and y+(0.577)*x >=160.057 and y-(0.577)*x <=39.945 and y-(0.577)*x >= -70.885:
          list_of_all_points.append((x,y))
        
      #highlighting_clearance
      if ((x-300)**2 + (y-185)**2 <= (40)**2):
          clearance_points.append((x,y))
          
      if x>=165 and x<=235 and y+(0.577)*x<=255.883 and y+(0.577)*x >=175.057 and y-(0.577)*x <=24.945 and y-(0.577)*x >= -55.885:
          clearance_points.append((x,y))
              
      if y - (0.316)*x <=173.607:
          if y+(1.232)*x >= 229.348:
              if y+(3.2)*x <=436 or y-(0.857)*x >=111.429:
                  clearance_points.append((x,y))

##################function to check if any point is within the obstacle space#################
#including the region covered by the robot radius and its clearance

def checkObstaclespace(point):
    
    global list_of_all_points    
    if point[0] in list_of_all_points[:][0] and  point[1] in list_of_all_points[:][1]:
        return False
    else:
        return True


#showing what every child node of each parent nodes are connected to
list_of_points_for_graph = []

def generateGraph(point,size_x,size_y,orientation,step_size): #remember that this size_x and size_y are the sizes of the matrix, so not the end coordinates
    

    global list_of_points_for_graph
    
    i = point[0] #x coordinate
    j = point[1] #y coordinate
    
    if i <=size_x and j<=size_y and i>=0 and j>=0:
        
        cost_values = {}
        
        pos0 = ActionMove(point,orientation+0,step_size)[0]#0
        if pos0[0]>=0 and pos0[1]>=0 and pos0[0]<=size_x and pos0[1]<=size_y:
            cost_values[pos0] = (1.0,orientation)
            
        pos30 = ActionMove(point,orientation+30,step_size)[0]#30
        if pos30[0]>=0 and pos30[1]>=0 and pos30[0]<=size_x and pos30[1]<=size_y: 
            cost_values[pos30] = (1.3,orientation+30)
            
        pos60 = ActionMove(point,orientation+60,step_size)[0]#60
        if pos60[0]>=0 and pos60[1]>=0 and pos60[0]<=size_x and pos60[1]<=size_y:
            cost_values[pos60] = (1.8,orientation+60)
            
        pos_minus60 = ActionMove(point,orientation-60,step_size)[0]#-60
        if pos_minus60[0]>=0 and pos_minus60[1]>=0 and pos_minus60[0]<=size_x and pos_minus60[1]<=size_y:
            cost_values[pos_minus60] = (1.8,orientation-60)
            
        pos_minus30 = ActionMove(point,orientation-30,step_size)[0]#-30
        if pos_minus30[0]>=0 and pos_minus30[1]>=0 and pos_minus30[0]<=size_x and pos_minus30[1]<=size_y:
            cost_values[pos_minus30] = (1.3,orientation-30)
            
        cost_values_copy = cost_values.copy()
        
        for k,v in cost_values_copy.items():
            if k==point:
                del cost_values[k]
        return(cost_values)
    
    else:
        
        pass

# function to backtrack so that the path from the goal to the end is obtained
        
def BackTrack(backtrack_dict,goal,start):#goal is the starting point now and start is the goal point now
    
    #initializing the backtracked list
    back_track_list = []
    #appending the start variable to the back_track_list list
    back_track_list.append(start)
    #while the goal is not found
    
    while goal!=[]:
        #for key and values in the backtracking dictionary 
        for k,v in backtracking_dict.items():
            
            #for the key and values in the values, v
            for k2,v2 in v.items():
                
                #checking if the first key is the start
                if k==start:
                    
                    #checking if not in the backtrackedlist
                    
                    if v2 not in back_track_list:
                        back_track_list.append(start)
                    #updating the start variable
                    start=v2
                    
                    #checking if it is the goal
                    if v2==goal:
                        goal=[]
                        break      
    #returns the backtracked list
    return(back_track_list)

#empty dictionary for backtracking from child to parent upto the start
backtracking = {}
#list of all the visited nodes
visited = set()
global orientation_to_layer
#Mapping of Angles to layers
orientation_to_layer={0:0,30:1,60:2,90:3,120:4,150:5,180:6,210:7,240:8,270:9,300:10,330:11, 360:0}
#array to store cost from

cost_from=np.array(np.ones((800,500,12)) * np.inf)
#Initializing visited nodes as empty array
V=np.zeros((800,500,12))
# array to store Heuristic distance
heur=np.array(np.ones((800,500)) * np.inf)
#array to store total cost f
f=np.array(np.ones((800,500,12)) * np.inf)
#list for Explored nodes
priority_queue=[]
# append start point,start orientation of the bot and initialize it's cost to zero
heapq.heappush(priority_queue,(0,start,start_orientation))
#initialize cost  for start node to zero
cost_from[int(2*start[0])][int(2*start[1])][orientation_to_layer[start_orientation]]=0
f[int(2*start[0])][int(2*start[1])][orientation_to_layer[start_orientation]]=0


######### < < < < A* algorithm code > > > > #########

def a_star_Algorithm(start,goal):
    
    global step_size
    global list_of_all_points
    global V
    break_while=0
  
    #create a dictionary for backtracked parents
    backtracking = {}
    
    #check if goal/start in obstacle space
  
    if goal in list_of_all_points or start in list_of_all_points:
        
        print('!!!!!!!!!!GOAL/START IS INSIDE OBSTACLE SPACE!!!!!!!!!!')
        backtracking=0
        rounded_neighbour=0
        
    else:
        while True:
            if break_while==1:
                break
          
           
            _,curr_vert,curr_orient=heapq.heappop(priority_queue)
            print("current_parent=",curr_vert)
            #append visited nodes
            visited.add(curr_vert)
            #checking if the neighbour is the goal. If goal found reached
            if ((curr_vert[0]-goal[0])**2 + (curr_vert[1]-goal[1])**2 <= (1.5)**2):
                
                print('^^^^^^^^^^^^^^')
                print('GOAL REACHED')
                print('^^^^^^^^^^^^^^')
               
                print(curr_vert)
                break
            #check whether node is in the obstacle space
            if checkObstaclespace(curr_vert)==True:
                #generate neighbours
                graph = generateGraph(curr_vert,400,250,curr_orient,step_size)
                graph_list = []
                #put neighbours in a list for easy access
                for key,cost_value in graph.items():
                    graph_list.append((key,cost_value))  
                for neighbour,cost in graph_list:
                    this_cost = graph[neighbour][0]
                    breakflag = 0
                    orientation = graph[neighbour][1]
                    orientation = orientation % 360
                    #Round the node
                    rounded_neighbour = (Round2Point5(neighbour[0]),Round2Point5(neighbour[1]))
                    if rounded_neighbour in visited:
                        breakflag=1 
                    #exit if found
                    if breakflag==1:
                        continue
                    #check if this neighbour is goal
                    if ((rounded_neighbour[0]-goal[0])**2 + (rounded_neighbour[1]-goal[1])**2 <= (1.5)**2):
                      
                        print('^^^^^^^^^^^^^^')
                        print('GOAL REACHED')
                        print('^^^^^^^^^^^^^^')
                      
                        break_while = 1
                        break
                    ##check whether current neighbour node is in the obstacle space
                    if checkObstaclespace(rounded_neighbour)==True:
                        #check if visited
                        if V[int(2*rounded_neighbour[0])][int(2*rounded_neighbour[1])][orientation_to_layer[orientation]]==0:
                            #if not, make it visited
                            V[int(2*rounded_neighbour[0])][int(2*rounded_neighbour[1])][orientation_to_layer[orientation]]=1
                            #calculate cost from
                            cost_from[int(2*rounded_neighbour[0])][int(2*rounded_neighbour[1])][orientation_to_layer[int(orientation)]]= (step_size +cost_from[int(2*curr_vert[0])][int(2*curr_vert[1])][orientation_to_layer[curr_orient]])
                            #calculate cost to go values
                            heur[int(2*rounded_neighbour[0])][int(2*rounded_neighbour[1])] = EucledianDistance(rounded_neighbour,goal)
                            #calculate f = g+h
                            f[int(2*rounded_neighbour[0])][int(2*rounded_neighbour[1])][orientation_to_layer[orientation]]=cost_from[int(2*rounded_neighbour[0])][int(2*rounded_neighbour[1])][orientation_to_layer[orientation]] + heur[int(2*rounded_neighbour[0])][int(2*rounded_neighbour[1])]
                            #push to the explored node queue
                            heapq.heappush(priority_queue,(f[int(2*rounded_neighbour[0])][int(2*rounded_neighbour[1])][orientation_to_layer[orientation]] ,rounded_neighbour,orientation))
                            backtracking[rounded_neighbour]={}
                            #adding to the backtracking dictionary
                            backtracking[rounded_neighbour][f[int(2*rounded_neighbour[0])][int(2*rounded_neighbour[1])][orientation_to_layer[orientation]]]=curr_vert
                        else:
                            #if visited, check cost. if newly genrated neighbour has a lower cost, update it
                            if(f[int(2*rounded_neighbour[0])][int(2*rounded_neighbour[1])][orientation_to_layer[orientation]])>(f[int(2*curr_vert[0])][int(2*curr_vert[1])][orientation_to_layer[curr_orient]]+step_size):
                                f[int(2*rounded_neighbour[0])][int(2*rounded_neighbour[1])][orientation_to_layer[orientation]]=(f[int(2*curr_vert[0])][int(2*curr_vert[1])][orientation_to_layer[curr_orient]]+step_size)
                                backtracking[rounded_neighbour][f[int(2*rounded_neighbour[0])][int(2*rounded_neighbour[1])][orientation_to_layer[orientation]]]=curr_vert
                    
    return(curr_vert,backtracking)                             
    #return the last parent node and backtracked dictionary
        
start_time = time.time()
#calling the function to solve the obstacle space using A* Algorithm
new_goal_rounded,backtracking_dict= a_star_Algorithm(start,goal)

print("SOLVED in =",time.time()-start_time)
#Backtracking back to the goal
backtracked_final = BackTrack(backtracking_dict,start,new_goal_rounded)
#printing the nodes from initial to the end
print(backtracked_final)


#Scale the visited nodes and backtracked nodes for easy visualization
new_list_visited = []
for visited_node in list(visited):
    new_x = visited_node[0]
    new_y = visited_node[1]
    new_list_visited.append((new_x,new_y))
    
new_backtracked = []
for back in backtracked_final:
    new_x_b = back[0]
    new_y_b = back[1]
    new_backtracked.append((new_x_b,new_y_b))
    
# #defining a blank canvas
#defining a blank canvas
new_canvas = np.zeros((251,401,3),np.uint8) 
#for every point that belongs within the obstacle

for l in list_of_all_points: #change the name of the variable l
    
    x = int(l[1])
    y = int(l[0])
    new_canvas[(x,y)]=[0,0,255]                                                 #assigning a red coloured pixel
for d in clearance_points:
    x =int(d[1])
    y =int(d[0])
    new_canvas[(x,y)] = [0,255,255]                                              #assigning a yellow coloured pixel
#flipping the image for correct orientation
new_canvas = np.flipud(new_canvas)
#making a copy for backtracking purpose
new_canvas_copy_backtrack = new_canvas.copy()
#making a copy for showing the covered nodes on the obstacle space
#can be used for the animation
new_canvas_copy_covered = new_canvas.copy()

#showing the obstacle map
cv2.imshow('new_canvas',new_canvas)
cv2.waitKey(0)
cv2.destroyAllWindows()


#visited path
for visit_path in new_list_visited:
    
    #print(path)
    x = int(visit_path[0])
    y = int(visit_path[1])
    new_canvas_copy_covered[(250-y,x)]=[255,255,255] #setting every backtracked pixel to white
    
#showing the final backtracked path
#new_visited = cv2.resize(new_canvas_copy_visited,(600,400))
#showing the image
cv2.imshow('visited',new_canvas_copy_covered)
#saving the image
#cv2.imwrite('visited_img.jpg',new_visited)
cv2.waitKey(0)
cv2.destroyAllWindows()

#backtracked path
for path in new_backtracked:
    
    #print(path)
    x = int(path[0])
    y = int(path[1])
    new_canvas_copy_backtrack[(250-y,x)]=[255,255,255] #setting every backtracked pixel to white
#showing the final backtracked path
#new_backtracked = cv2.resize(new_canvas_copy_backtrack,(600,400))

#showing the image
cv2.imshow('backtracked',new_canvas_copy_backtrack)
#saving the image
#cv2.imwrite('backtracked_img.jpg',new_backtracked)
cv2.waitKey(0)
cv2.destroyAllWindows()

#showing animation
pygame.init()

display_width = 400
display_height = 250

gameDisplay = pygame.display.set_mode((display_width,display_height),pygame.FULLSCREEN)
pygame.display.set_caption('Covered Nodes- Animation')

black = (0,0,0)
white = (255,255,255)
#new = np.array(new_canvas_copy_visited)
surf = pygame.surfarray.make_surface(new_canvas)

clock = pygame.time.Clock()

done = False
while not done:
    
    for event in pygame.event.get(): 
        
        if event.type == pygame.QUIT:  
            done = True   

    gameDisplay.fill(black)
    for path in new_list_visited:
        if path not in new_canvas_copy_covered:
           
            x = path[0]
            y = abs(250-path[1])
            pygame.draw.rect(gameDisplay, white, [x,y,1,1])
            pygame.display.flip()
            
    for path in new_backtracked:
        
        pygame.time.wait(5)
        x = path[0]
        y = abs(250-path[1])
        pygame.draw.rect(gameDisplay, (0,0,255), [x,y,1,1])
        pygame.display.flip()

    done = True
pygame.quit()



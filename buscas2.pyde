# The Nature of Code
# Daniel Shiffman
# http://natureofcode.com
#
# Modified by sslp23

# Draws a "vehicle" on the screen

from vehicle import Vehicle
from food import Food
import random
import time
from queue import Queue, PriorityQueue
import sys
sys.setrecursionlimit(1000000)

################## GLOBALS
video_scale = 8
vis = []
path = []
walked = 1
comeu = 0
waiter=0
usr_input = 0

############ BFS ############
def bfs(g, start, goal):
    frontier = Queue()
    frontier.put(start)
    reached = {}
    reached[start] = True
    visited = []
    came_from = {}
    came_from[start] = None
    
    while not frontier.empty():
        current = frontier.get()
        if current == goal:
            break
        for next in g[current]:
            if next not in reached:
                frontier.put(next)
                reached[next] = True
                visited.append(next)
                came_from[next] = current
                
    print('Found')
    return visited, came_from

############### DFS #############
def dfs(graph, start, goal):
    visited = []
    stack = []
    came_from = dict()
    stack.append(start)
    
    while stack:
        current = stack.pop()
        if current == goal:
            print('Found')
            return visited, came_from
        if current not in visited:
            visited.append(current)
            for neighbor in graph[current]:
                if neighbor not in visited:
                    came_from[neighbor] = current
                    stack.append(neighbor)
            
    
    return visited, came_from

def cost(s):
    if s in water:
        return 10
    elif s in sand:
        return 5
    else:
        return 1

############# dijkstra
def dijkstra(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    visited = []

    while not frontier.empty():
        current = frontier.get()
        visited.append(current)
        
        if current == goal:
            print('Found')
            break
        
        for next in graph[current]:
            new_cost = cost_so_far[current] + cost(next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost
                frontier.put(next, priority)
                came_from[next] = current
    
    return visited, came_from

def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)

########### greedy
def greedy(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    came_from[start] = None
    visited = []

    while not frontier.empty():
        current = frontier.get()
        visited.append(current)
        
        if current == goal:
            print('Found')
            break
        
        for next in graph[current]:
            if next not in came_from:                
                priority = heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current
    
    return visited, came_from

##########a star
def astar(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    visited = []

    while not frontier.empty():
        current = frontier.get()
        visited.append(current)
        
        if current == goal:
            print('Found')
            break
        
        for next in graph[current]:
            new_cost = cost_so_far[current] + cost(next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current
    
    return visited, came_from

    
# função para achar o cami33nho a partir da busca
def reconstruct_path(came_from, start, goal):
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    return path

#criação da areia
def create_sand():    
    global sand
    obs_x = [random.randint(0, cols-1) for i in range(0, 50, 1)]
    obs_x = [i*video_scale for i in obs_x]
    obs_y = [random.randint(0, rows-1) for i in range(0, 50, 1)]
    obs_y = [j*video_scale for j in obs_y] 
    
    for (i, j) in zip(obs_x, obs_y):
        fill(255, 255, 0)
        stroke(0)
        rect(i, j, video_scale, video_scale)
    
    sand = [tuple((x, y)) for (x, y) in zip(obs_x, obs_y)] #posições na qual há areia
    
#criação da água
def create_water():
    global water
    obs_x = [random.randint(0, cols-1) for i in range(0, 50, 1)]
    obs_y = [random.randint(0, rows-1) for i in range(0, 50, 1)]
    
    for (i, j) in zip(obs_x, obs_y):
        fill(0, 0, 255)
        stroke(0)
        rect(i*video_scale, j*video_scale, video_scale, video_scale)
    
    water = [tuple((x, y)) for (x, y) in zip(obs_x, obs_y)] #posições na qual há areia
        
def neighbours(i, j, obs):
    n = [(i+1,j+1), (i-1,j-1), (i-1,j+1), (i+1,j-1), (i, j-1), (i, j+1), (i+1, j), (i-1, j)]
    takeout = []    
    for v in n:
        if (v[0]< 0) or (v[1]<0) or (v[0]>79) or (v[1]>59):
            takeout.append(v)
        if (v in obs):
            takeout.append(v)
    takeout = list(set(takeout))
    for t in takeout:
        n.remove(t)
    return n
    

def setup():
    ## MAP
    size(640, 520)
    global cols
    cols = int(width/video_scale)

    global rows
    rows = int(480/video_scale)
    
    background(255)
    
    rand = random.randint(5, 10)
    for i in range(0, cols, 1):
        for j in range(0, rows, 1):
            x = i*video_scale
            y = j*video_scale
            fill(255)
            stroke(0)
            rect(x, y, video_scale, video_scale)
    
    #Criando areia
    global sand
    create_sand()
    #Criando água
    global water
    create_water()
    
    #Criando os obstaculos
    obs_x = [random.randint(0, cols-1) for i in range(0, 50, 1)]
    obs_y = [random.randint(0, rows-1) for i in range(0, 50, 1)]
    
    global obs
    obs = [tuple((x, y)) for (x, y) in zip(obs_x, obs_y)]
    
    for (i, j) in zip(obs_x, obs_y):
        fill(255,0,255)
        stroke(0)
        rect(i*video_scale, j*video_scale, video_scale, video_scale)
    
    
    
    global vehicle
    
    velocity = PVector(0, 0)
    x = random.randint(0, cols-1)
    y = random.randint(0, rows-1)
    
    global start_pos
    while (x,y) in obs:
        x = random.randint(0, cols-1)
        y = random.randint(0, rows-1)
    vehicle = Vehicle(x*video_scale, y*video_scale, velocity)
    start_pos = (x, y)
    
    print(start_pos)
    global food
    velocity = PVector(0, 0)
    x = random.randint(0, cols-1)
    y = random.randint(0, rows-1)
    
    global final_pos
    while (x,y) in obs:
        x = random.randint(0, cols-1)
        y = random.randint(0, rows-1)
    final_pos = (x, y)
    print(final_pos)
    food = Food(x*video_scale, y*video_scale, velocity)

    
    global g
    g = {}
    for i in range(0, cols, 1):
        for j in range(0, rows, 1):
            if (x,y) not in obs:
                g[(i,j)] = neighbours(i, j, obs)



    #time.sleep(10)

    
def draw():
    global waiter
    global usr_input
    global vehicle
    global comeu
    last_alg = ""
    vehicle.update()
    vehicle.display()
    
    food.update()
    food.display()

    if waiter == 1:
        global vis
        global path
        global walked

        
        if len(vis)<1 and walked == 1:

            if int(usr_input) == 1: #bfs
                vis, paths = bfs(g, start_pos, final_pos)
                last_alg = "BFS" 
            elif int(usr_input) == 2: #dfs
                vis, paths = dfs(g, start_pos, final_pos)
                last_alg = "DFS"
            elif int(usr_input) == 3: #dijkstra
                vis, paths = dijkstra(g, start_pos, final_pos)
                last_alg = "DIJKSTRA"
            elif int(usr_input) == 4: #greedy
                vis, paths = greedy(g, start_pos, final_pos)
                last_alg = "GREEDY"
            else: #astar
                vis, paths = astar(g, start_pos, final_pos)
                last_alg = "A*"
            
            path = reconstruct_path(paths, start_pos, final_pos)
            vis.reverse()
            walked = 0
        
        if len(vis)>0:
            last = vis.pop()
            fill(127,127,127,220)
            rect(last[0]*video_scale, last[1]*video_scale, video_scale, video_scale)
            
        else:
            
            v_pos = path.pop()
            '''
            if len(path) > 1:
                next = path[-1]
                line(v_pos[0]*video_scale, v_pos[1]*video_scale, next[0]*video_scale, next[1]*video_scale)
                stroke(255,0,0)
            '''
            
            vel = PVector(0,0)
            vehicle = Vehicle(v_pos[0]*video_scale, v_pos[1]*video_scale, vel)
            vehicle.update()
            vehicle.display()
            if v_pos in water:
                print('water')
                time.sleep(1) #movimento atrasa 10 vezes na água
            elif v_pos in sand:
                print('sand')
                time.sleep(0.5) #movimento atrasa 5 vezes na areia
            else:
                time.sleep(0.1)
            if len(path)<1:
                walked = 1
        
        
        if walked == 1:
            comeu+=1
            print('comeu %d comidas'%(comeu))
            waiter = 0
            setup()
        #fill(127,127,127,10)
        #rect(last[0], last[1], video_scale, video_scale)
    else:
        text("Escolha o algoritmo: 1 - BFS / 2 - DFS / 3 - DIJKSTRA / 4 - GREEDY / 5 - A*", x= 0, y=495)
    
    text("Algoritmo usado: %s"%(last_alg),x=0, y=510)
    text("Placar: %d"%(comeu), x=560, y=495)

def keyPressed():
    global usr_input
    usr_input = int(key)

    global waiter
    waiter=1
                 

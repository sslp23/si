# The Nature of Code
# Daniel Shiffman
# http://natureofcode.com
#
# Modified by Filipe Calegario

# Draws a "vehicle" on the screen

from vehicle import Vehicle
from food import Food
import random
import time
from queue import Queue
import sys
sys.setrecursionlimit(1000000)


video_scale = 8
vis = []
path = []
walked = 1

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
    
def reconstruct_path(came_from, start, goal):
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    return path


def create_sand():
    obs_x = [random.randint(0, cols-1) for i in range(0, 50, 1)]
    obs_x = [i*video_scale for i in obs_x]
    obs_y = [random.randint(0, rows-1) for i in range(0, 50, 1)]
    obs_y = [j*video_scale for j in obs_y] 
    
    for (i, j) in zip(obs_x, obs_y):
        fill(255, 255, 0)
        stroke(0)
        rect(i, j, video_scale, video_scale)
    
def create_water():
    obs_x = [random.randint(0, cols-1) for i in range(0, 50, 1)]
    obs_y = [random.randint(0, rows-1) for i in range(0, 50, 1)]
    
    for (i, j) in zip(obs_x, obs_y):
        fill(0, 0, 255)
        stroke(0)
        rect(i*video_scale, j*video_scale, video_scale, video_scale)
        
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
    size(640, 480)
    global cols
    cols = int(width/video_scale)
    global rows
    rows = int(height/video_scale)
    
    background(255)
    
    rand = random.randint(5, 10)
    for i in range(0, cols, 1):
        for j in range(0, rows, 1):
            x = i*video_scale
            y = j*video_scale
            fill(255)
            stroke(0)
            rect(x, y, video_scale, video_scale)
            
    create_sand()
    create_water()
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
    global vis
    global path
    global walked
    global vehicle
    if len(vis)<1 and walked == 1:
        vehicle.update()
        vehicle.display()
        
        food.update()
        food.display()
        vis, paths = bfs(g, start_pos, final_pos)
        path = reconstruct_path(paths, start_pos, final_pos)
        vis.reverse()
        walked = 0
    
    if len(vis)>0:
        last = vis.pop()
        fill(127,127,127,220)
        rect(last[0]*video_scale, last[1]*video_scale, video_scale, video_scale)
        
    else:
        #vehicle.seek(PVector(path[-1][0]*video_scale, path[-1][1]*video_scale))
        v_pos = path.pop()
        if len(path) > 1:
            next = path[-1]
            line(v_pos[0]*video_scale, v_pos[1]*video_scale, next[0]*video_scale, next[1]*video_scale)
            stroke(255,0,0)
        
        vel = PVector(0,0)
        vehicle = Vehicle(v_pos[0]*video_scale, v_pos[1]*video_scale, vel)
        vehicle.update()
        vehicle.display()
        time.sleep(0.1)
        if len(path)<1:
            walked = 1
    
    
    if walked == 1:
        setup()
    #fill(127,127,127,10)
    #rect(last[0], last[1], video_scale, video_scale)

import cv2
import numpy as np
import heapq

img=cv2.imread('map2.png')
img=cv2.resize(img,(1080,620))
img1=img.copy()

gray_scale=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
ret,b_img=cv2.threshold(gray_scale,243,255,cv2.THRESH_BINARY)
obstacles=np.argwhere(b_img==0)
obstacles=np.array(obstacles)
obstacles=np.fliplr(obstacles)

source=np.array([586,136])
goal=np.array([250,300])
# goal=np.array([790,149])
# goal=np.array([680,549])

img=cv2.circle(img,source,3,(255,0,0),-1)
img=cv2.circle(img,goal,3,(255,0,0),-1)
img1=cv2.circle(img1,source,3,(255,0,0),-1)
img1=cv2.circle(img1,goal,3,(255,0,0),-1)   

def sampling(x):
    scale=130-x
    x=int(np.random.normal(goal[0],scale))
    y=int(np.random.normal(goal[1],scale))
    # x=int(np.random.uniform(0,1080))
    # y=int(np.random.uniform(0,620))
    return np.array([x, y])

def nearest_neighbor(nodes,point):
    dists=np.linalg.norm(nodes - point, axis=1)
    nearest_index=np.argmin(dists)
    nearest_node=nodes[nearest_index]
    return nearest_node

def steer(from_point,to_point):
    direction=np.array(to_point)-np.array(from_point)
    magnitude=np.linalg.norm(direction)
    if magnitude<4:
        return to_point
    else:
        tmp=from_point+(direction/magnitude)*4
        return tmp.astype(int)

def is_collision_free(from_point,to_point):
    num=abs(int(np.linalg.norm(to_point-from_point)-1))
    if num>10:
        return False
    line=np.linspace(from_point,to_point,num)
    for point in line:
        if len(np.argwhere(np.all(obstacles==point,axis=1)))>0:
            return False
    return True

def rrt(img):
    nodes=[tuple(source)]
    i=0
    while True:
        i=i+1
        rand_point=sampling((i/67))
        nearest_node=nearest_neighbor(nodes,rand_point)
        point=steer(nearest_node,rand_point)
        if is_collision_free(nearest_node, point):
            nodes.append(tuple(point))
            cv2.line(img,tuple(nearest_node),tuple(point),(0,0,255),1)
            cv2.imshow('Map',img)
            k=cv2.waitKey(1) & 0xff
            if k==27:
                break
            if np.linalg.norm(point-goal)<15:
                print("Found in %d iterations" %(i))
                cv2.destroyAllWindows()
                nodes=np.array(nodes)
                return nodes
    print("Not Found")
    return None

def dijkstra(graph,source,goal):
    pq=[(0,source)]
    heapq.heapify(pq)
    visited=set()
    parent={}
    dist={node:float('inf') for node in graph}
    dist[source]=0
    while pq:
        current_dist,current_node=heapq.heappop(pq)
        if current_node==goal:
            break
        if current_node in visited:
            continue
        visited.add(current_node)
        for neighbor,weight in graph[current_node]:
            if neighbor in visited:
                continue
            new_dist=current_dist+weight
            if new_dist<dist[neighbor]:
                dist[neighbor]=new_dist
                parent[neighbor]=current_node
                heapq.heappush(pq,(new_dist,neighbor))
    path=[]
    while goal!=source:
        path.append(goal)
        goal=parent[goal]
    path.append(source)
    return path[::-1]

def build_graph(nodes):
    graph={}
    print("Searching...")
    for node in nodes:
        graph[tuple(node)]=[]
    for i,node in enumerate(nodes):
        for other_node in nodes[i+1:]:
            if is_collision_free(node,other_node):
                weight=int(np.linalg.norm(np.array(node)-np.array(other_node)))
                graph[tuple(node)].append((tuple(other_node),weight))
                graph[tuple(other_node)].append((tuple(node),weight))
    return graph

tree=rrt(img)

if tree is not None:
    graph=build_graph([np.array(node) for node in tree])
    path=dijkstra(graph,tuple(source),tuple(tree[-1]))  
    for i in range(len(path)-1):
        cv2.line(img1,tuple(path[i]),tuple(path[i+1]),(0,255,0),1)
    cv2.imshow('Map', img1)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    pass
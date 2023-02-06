#full name: Nguyễn Thái Hiệp
#id: 20127496

import numpy as np
from queue import PriorityQueue
from queue import Queue
import math

INF = 999999

def DFS(matrix, start, end):
    """
    BFS algorithm:
    Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer
        starting node
    end: integer
        ending node
    
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes, each key is a visited node,
        each value is the adjacent node visited before it.
    path: list
        Founded path
    """
    # TODO: 
    
    path=[]
    visited={}

    frontier = []
    n_vertices = matrix.shape[0]
    checked = [False for i in range(n_vertices)]

    frontier.append((-1,start))
    while frontier:
        print(frontier)
        buffer, node = frontier.pop()

        if checked[node] == True : continue
        visited.update({node: buffer})
        checked[node] = True
        if node == end:
            while node != start:
                path.append(node)
                node = visited[node]
            path.append(start)
            path.reverse()
            break

        for i in range(n_vertices - 1, -1, -1):
            if matrix[node][i] != 0 and checked[i] == False:
                frontier.append((node, i))

    return visited, path

def BFS(matrix, start, end):
    """
    DFS algorithm
     Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node
    
    Returns
    ---------------------
    visited 
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """

    # TODO: 
    
    path=[]
    visited={}

    frontier = Queue()
    n_vertices = matrix.shape[0]
    checked = [False for i in range(n_vertices)]
 
    frontier.put((-1, start))
    while not frontier.empty():
        node_buffer, node = frontier.get()
        
        if checked[node] == True : continue
            
        visited[node] = node_buffer
        checked[node] = True

        if node == end:
            while node != start:
                path.append(node)
                node = visited[node]
            path.append(start)
            path.reverse()
            break

        for i in range(n_vertices):
            if matrix[node][i] != 0 and checked[i] == False:
                frontier.put((node, i))
    
    return visited, path

def UCS(matrix, start, end):
    """
    Uniform Cost Search algorithm
     Parameters:visited
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node
    
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO:  
    
    path=[]

    visited={}
    global INF

    frontier = PriorityQueue()

    n_vertices = matrix.shape[0]

    checked = [False for i in range(n_vertices)]
    explored = [INF for i in range(n_vertices)]
    frontier.put((0,-1, start))
    
    while not frontier.empty():
        currentWeight, buffer, vertex = frontier.get()

        explored[vertex] = -INF
        if checked[vertex] == True : continue
        checked[vertex] = True
        
        visited.update({vertex: buffer})
        if vertex == end:
            while vertex != start:
                path.append(vertex)
                vertex = visited[vertex]
            path.append(start)
            path.reverse()
            break
        
        for edge in range(n_vertices):
            if matrix[vertex][edge] != 0 and checked[edge] == False:
                weight = currentWeight + matrix[vertex][edge]
                if weight < explored[edge]:
                    explored[edge] = weight
                    frontier.put((explored[edge],vertex, edge))

    return visited, path

# Heuristic of Greedy function
def H(matrix, start):

    frontier = PriorityQueue()

    n_vertices = matrix.shape[0]

    checked = [False for i in range(n_vertices)]
    Heuristic = [0 for i in range(n_vertices)]
    explored = [INF for i in range(n_vertices)]
    frontier.put((0,-1, start))
    
    while not frontier.empty():
        currentWeight, buffer, vertex = frontier.get()

        Heuristic[vertex] = currentWeight
        explored[vertex] = -INF
        if checked[vertex] == True : continue
        checked[vertex] = True
        
        for edge in range(n_vertices):
            if matrix[vertex][edge] != 0 and checked[edge] == False:
                weight = currentWeight + matrix[vertex][edge]
                if weight < explored[edge]:
                    explored[edge] = weight
                    frontier.put((explored[edge],vertex, edge))

    return Heuristic

def GBFS(matrix, start, end):
    """
    Greedy Best First Search algorithm
     Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node
   
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    
    # TODO: 
    path=[]
    visited={}

    global INF

    Heuristic = H(matrix, end)
    frontier = PriorityQueue()

    n_vertices = matrix.shape[0]

    frontier.put((0,-1, start))
    
    while not frontier.empty():
        ignore, buffer, vertex = frontier.get()
        visited.update({vertex: buffer})
        if vertex == end:
            while vertex != start:
                path.append(vertex)
                vertex = visited[vertex]
            path.append(start)
            path.reverse()
            break
        
        for edge in range(n_vertices):
            if matrix[vertex][edge] != 0:
                frontier.put((Heuristic[edge],vertex, edge))

    return visited, path


def Heuristic(pos, end, n_vertices):
    x_end = pos[end][0]
    y_end = pos[end][1]
    H = [0 for i in range(n_vertices)]
    for i in pos:
        x_i = pos[i][0]
        y_i = pos[i][1]
        H[i] = math.sqrt((x_i - x_end)*(x_i - x_end) + (y_i - y_end)*(y_i - y_end))
    return H

def Astar(matrix, start, end, pos):
    """
    A* Search algorithm
     Parameters:
    ---------------------------
    matrix: np array UCS
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node
    pos: dictionary. keys are nodes, values are positions
        positions of graph nodes
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO: 
    path=[]
    visited={}

    global INF


    frontier = PriorityQueue()

    n_vertices = matrix.shape[0]

    H = Heuristic(pos, end, n_vertices)

    checked = [False for i in range(n_vertices)]
    explored = [INF for i in range(n_vertices)]
    frontier.put((0, -1, start))
    
    while not frontier.empty():
        currentWeight,buffer, vertex = frontier.get()
        if checked[vertex] == True: continue
        explored[vertex] = -INF
        checked[vertex] = True
        visited.update({vertex: buffer})
        if vertex == end:
            while vertex != start:
                path.append(vertex)
                vertex = visited[vertex]
            path.append(start)
            path.reverse()
            break
        
        for edge in range(n_vertices):
            if matrix[vertex][edge] != 0 and checked[edge] == False:
                weight = currentWeight + matrix[vertex][edge] + H[edge]
                if weight < explored[edge]:
                    explored[edge] = weight
                    frontier.put((explored[edge],vertex, edge))

    return visited, path

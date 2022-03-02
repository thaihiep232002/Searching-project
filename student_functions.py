import numpy as np
from PriorityQueue import PriorityQueue

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

    frontier.append(start)
    
    while frontier:
        node = frontier.pop()
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
                frontier.append(i)
                visited[i] = node
    print(path)
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
   

    frontier = []
    n_vertices = matrix.shape[0]
    checked = [False for i in range(n_vertices)]

    frontier.append(start)
    
    while frontier:
        node = frontier.pop(0)
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
                frontier.append(i)
                visited[i] = node
    print(path)
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
    # path=[]
    # visited={}

    # frontier = PriorityQueue()

    # n_vertices = matrix.shape[0]
    # checked = [False for i in range(n_vertices)]

    # frontier.insert((start, 0))
    
    # while frontier:
    #     node = frontier.delete()

    #     vertex = node[0]
    #     cost = node[1]
    #     checked[vertex] = True
    #     if vertex == end:
    #         while vertex != start:
    #             path.append(vertex)
    #             vertex = visited[vertex]
    #         path.append(start)
    #         path.reverse()
    #         break

    #     for i in range(n_vertices):
    #         if matrix[vertex][i] != 0 and checked[i] == False:
    #             frontier.insert((i, matrix[vertex][i]))
    #             visited[i] = vertex
    # print(path)

    return visited, path


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
    return visited, path

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
    return visited, path


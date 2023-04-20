import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import heapq
from collections import defaultdict


graph = np.array([[0,1,1,1,1],
                  [1,0,1,0,0],
                  [1,1,0,0,0],
                  [1,0,0,0,1],
                  [1,0,0,1,0]])

G = nx.from_numpy_array(graph)
nx.draw(G, with_labels=True)
#plt.show() #permet d'afficher le graphe



def dijkstra(graph, start, end):
    predecessors = {start: None}

    distances = {}
    for node in graph:
        distances[node] = float("inf")
    distances[start] = 0

    queue = [start]
    while queue:
        current_node = queue.pop(0)
        if current_node == end:
            # construction du chemin à partir des précédents
            current_path = [end]
            while current_node != start:
                current_path.append(predecessors[current_node])
                current_node = predecessors[current_node]
            return current_path[::-1] # inversion du chemin

        for neighbour in graph[current_node]:
            tentative_distance = distances[current_node] + graph[current_node][neighbour]
            if tentative_distance < distances[neighbour]:
                distances[neighbour] = tentative_distance
                predecessors[neighbour] = current_node
                queue.append(neighbour)
    return None



def get_odd(graph):
    nodes = set()
    for i in range(len(graph)):
        for j in range(len(graph)):
            if graph[i][j] > 0:
                nodes.add(i)
                nodes.add(j)

    odd_nodes = []
    for node in nodes:
        degree = 0
        for i in range(len(graph)):
            if graph[node][i] > 0:
                degree += 1
        if degree % 2 == 1:
            odd_nodes.append(node)
    return odd_nodes


def sum_edges(graph):
    return np.sum(graph) // 2


def gen_pairs(odds):
    pairs = []
    while odds:
        odd = odds.pop()
        min_dist = float('inf')
        closest_node = None
        for node in odds:
            if graph[odd][node] < min_dist:
                min_dist = graph[odd][node]
                closest_node = node
        pairs.append((odd, closest_node))
        odds.remove(closest_node)
    return pairs


def shortest_path(graph, start_node, end_node):
    distances = {node: float('inf') for node in graph}
    distances[start_node] = 0
    queue = []
    heapq.heappush(queue, [distances[start_node], start_node])
    predecessors = {node: {} for node in graph} # initialisation de predecessors
    visited = set()

    while queue:
        current_distance, current_node = heapq.heappop(queue)
        if current_node == end_node:
            path = []
            while current_node != start_node:
                path.append(current_node)
                current_node = predecessors[current_node]
            path.append(start_node)
            return path[::-1], current_distance
        visited.add(current_node)
        for neighbor, weight in graph[current_node].items():
            if neighbor not in visited:
                tentative_distance = current_distance + weight
                if tentative_distance < distances[neighbor]:
                    distances[neighbor] = tentative_distance
                    predecessors[neighbor] = current_node # mettre a jour le predecesseur
                    heapq.heappush(queue, [tentative_distance, neighbor])
    return [], float('inf')





def Chinese_Postman(graph):
    odds = get_odd(graph)
    if len(odds) == 0:
        return sum_edges(graph), [] # retourne une liste vide pour le chemin si le graphe est déjà eulérien
    pairs = gen_pairs(odds)
    l = (len(pairs)+1)//2 
    pairings_sum = []

    def get_pairs(pairs, done = [], final = []):
        if len(pairs[0]) != 2:
            raise ValueError('Pairs should contain two nodes')
        if pairs[0][0] not in done:
            done.append(pairs[0][0])
            for i in pairs[0]:
                f = final[:]
                val = done[:]
                if i[1] not in val:
                    f.append(i)
                else:
                    continue
                
                if len(f) == l:
                    pairings_sum.append(f)
                    return 
                else:
                    val.append(i[1])
                    get_pairs(pairs[1:], val, f)
        else:
            get_pairs(pairs[1:], done, final)


    get_pairs(pairs)
    min_sums = []

    for i in pairings_sum:
        s = 0
        for j in range(len(i)):
            s += dijkstra(graph, i[j][0], i[j][1])
        min_sums.append(s)

    added_dis = min(min_sums)

    paths = shortest_path(graph, odds)
    visited = set()
    path = []
    start_vertex = odds[0]

    while len(visited) < len(odds):
        for i in range(1, len(odds)):
            if i not in visited:
                if (start_vertex, i) in paths:
                    path += paths[(start_vertex, i)][:-1]
                    visited.update([start_vertex, i])
                    start_vertex = i
                    break
        else:
            for i in range(1, len(odds)):
                if i not in visited:
                    path += paths[(start_vertex, i)][:-1]
                    visited.update([start_vertex, i])
                    start_vertex = i
                    break

    chinese_dis = added_dis + sum_edges(graph)
    return chinese_dis, path # retourne à la fois le poids total et le chemin


chinese_dis, path = Chinese_Postman(graph)

print('Le plus court chemin que doit emprunter le facteur a un poids de :', chinese_dis)
print('Le chemin est :', path)


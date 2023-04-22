import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict

"""
graph = [
[0, 1, 1, 0, 0],
 [1, 0, 1, 1, 0],
 [1, 1, 0, 1, 1],
 [0, 1, 1, 0, 1],
 [0, 0, 1, 1, 0]]



graph = [[0,0,1,0],
          [0,0,1,1],
          [1,1,0,1],
          [0,1,1,0]]




graph = [[0,1,0,0,0],  #graphe non eulerien mais qu'on peut transformer en eulérien
          [1,0,0,0,1],
          [0,0,0,1,1],
          [0,0,1,0,1],
          [0,1,1,1,0]]



graph = [[0,1,1,1,1],
          [1,0,1,0,0],   #graĥe eulérien
          [1,1,0,0,0],
          [1,0,0,0,1],
          [1,0,0,1,0]]



graph =                 [[0, 4, 0, 0, 0, 0, 0, 8, 0], 
                       [4, 0, 8, 0, 0, 0, 0, 11, 0], 
                        [0, 8, 0, 7, 0, 4, 0, 0, 2],   #graphe non eulerien mais qu'on peut transformer en eulérien
                        [0, 0, 7, 0, 9, 14, 0, 0, 0], 
                        [0, 0, 0, 9, 0, 10, 0, 0, 0], 
                        [0, 0, 4, 0, 10, 0, 2, 0, 0], 
                        [0, 0, 0, 14, 0, 2, 0, 1, 6], 
                        [8, 11, 0, 0, 0, 0, 1, 0, 7], 
                        [0, 0, 2, 0, 0, 0, 6, 7, 0] 
                    ]; 


graph =                [[0, 3, 1, 0, 5, 0], 
                        [3, 0, 0, 1, 0, 6], 
                        [1, 0, 0, 0, 2, 0],  #graphe non eulerien mais qu'on peut transformer en eulérien
                        [0, 1, 0, 0, 0, 1], 
                        [5, 0, 2, 0, 0, 4], 
                        [0, 6, 0, 1, 4, 0], 
                         
                    ]; 

"""


graph =     [[0,1,0,0,0,0,0,0,0],   #graphinsa impossible de transformer en eulérien
                [1,0,1,0,0,0,0,1,0],
                [0,1,0,1,0,0,1,0,0],
                [0,0,1,0,1,1,0,0,0],
                [0,0,0,1,0,0,0,0,0],
                [0,0,0,1,0,0,1,0,0],
                [0,0,1,0,0,1,0,1,1],
                [0,1,0,0,0,0,1,0,1],
                [0,0,0,0,0,0,1,1,0]]

            

G = nx.from_numpy_array(np.array(graph))
nx.draw(G, with_labels=True)
plt.show() #permet d'afficher le graphe


def sum_edges(graph):
    w_sum = 0
    l = len(graph)
    for i in range(l):
        for j in range(i,l):
            w_sum += graph[i][j]
    return w_sum
            

def dijktra(graph, source, dest):
    shortest = [0 for i in range(len(graph))]
    selected = [source]
    l = len(graph)
    #Base case from source
    inf = 10000000
    min_sel = inf
    for i in range(l):
        if(i==source):
            shortest[source] = 0 #graph[source][source]
        else:
            if(graph[source][i]==0):
                shortest[i] = inf
            else:
                shortest[i] = graph[source][i]
                if(shortest[i] < min_sel):
                    min_sel = shortest[i]
                    ind = i
                
    if(source==dest):
        return 0
    # Dijktra's in Play
    selected.append(ind) 
    while(ind!=dest):
        #print('ind',ind)
        for i in range(l):
            if i not in selected:
                if(graph[ind][i]!=0):
                    #Check if distance needs to be updated
                    if((graph[ind][i] + min_sel) < shortest[i]):
                        shortest[i] = graph[ind][i] + min_sel
        temp_min = 1000000
        #print('shortest:',shortest)
        #print('selected:',selected)
        
        for j in range(l):
            if j not in selected:
                if(shortest[j] < temp_min):
                    temp_min = shortest[j]
                    ind = j
        min_sel = temp_min
        selected.append(ind)
    
    return shortest[dest]
                            
#Finding odd degree vertices in graph

def get_odd(graph):
    degrees = [0 for i in range(len(graph))]
    for i in range(len(graph)):
        for j in range(len(graph)):
                if(graph[i][j]!=0):
                    degrees[i]+=1
                
    #print(degrees)
    odds = [i for i in range(len(degrees)) if degrees[i]%2!=0]
    #print('odds are:',odds)
    return odds

#Function to generate unique pairs
def gen_pairs(odds):
    pairs = []
    for i in range(len(odds)-1):
        pairs.append([])
        for j in range(i+1,len(odds)):
            pairs[i].append([odds[i],odds[j]])
        
    #print('pairs are:',pairs)
    #print('\n')
    return pairs


#Final Compiled Function
def Chinese_Postman(graph):
    odds = get_odd(graph)
    if(len(odds)==0):
        return sum_edges(graph)
    pairs = gen_pairs(odds)
    l = (len(pairs)+1)//2
    
    pairings_sum = []
    
    def get_pairs(pairs, done = [], final = []):
        
        if(pairs[0][0][0] not in done):
            done.append(pairs[0][0][0])
            
            for i in pairs[0]:
                f = final[:]
                val = done[:]
                if(i[1] not in val):
                    f.append(i)
                else:
                    continue
                
                if(len(f)==l):
                    pairings_sum.append(f)
                    return 
                else:
                    val.append(i[1])
                    get_pairs(pairs[1:],val, f)
                    
        else:
            get_pairs(pairs[1:], done, final)
            
    get_pairs(pairs)
    min_sums = []
    
    for i in pairings_sum:
        s = 0
        for j in range(len(i)):
            s += dijktra(graph, i[j][0], i[j][1])
        min_sums.append(s)
    
    added_dis = min(min_sums)
    chinese_dis = added_dis + sum_edges(graph)
    return chinese_dis    

def fleury(graph):
    # Copie de graph pour éviter de le modifier
    graph_copy = [row[:] for row in graph]
    
    # On récupère le premier sommet
    current_vertex = 0
    # On initialise le chemin
    path = [current_vertex]
    while sum_edges(graph_copy) != 0:
        # On recherche une arête qui n'est pas une arête de pont
        bridges = find_bridges(graph_copy)
        non_bridges = [(i,j) for i in range(len(graph_copy)) for j in range(len(graph_copy)) if graph_copy[i][j]!=0 and (i,j) not in bridges]
        if len(non_bridges) == 0:
            break
        next_vertex = None
        for edge in non_bridges:
            if edge[0] == current_vertex or edge[1] == current_vertex:
                next_vertex = edge[0] + edge[1] - current_vertex
                graph_copy[edge[0]][edge[1]] = 0
                graph_copy[edge[1]][edge[0]] = 0
                break
        if next_vertex is None:
            # Cas où tous les voisins sont des ponts
            next_vertex = path[-2]
            graph_copy[current_vertex][next_vertex] = 0
            graph_copy[next_vertex][current_vertex] = 0
            path = path[:-1]
        else:
            path.append(next_vertex)
        current_vertex = next_vertex
    
    return path



def find_bridges(graph):
    bridges = []
    visited = set()
    parent = [-1] * len(graph)
    low = [float('inf')] * len(graph)
    disc = [float('inf')] * len(graph)
    
    def dfs(v):
        visited.add(v)
        disc[v] = dfs.time
        low[v] = dfs.time
        dfs.time += 1
        
        for i in range(len(graph)):
            if graph[v][i] != 0:
                if i not in visited:
                    parent[i] = v
                    dfs(i)
                    low[v] = min(low[v], low[i])
                    if low[i] > disc[v]:
                        bridges.append((v, i))
                elif i != parent[v]:
                    low[v] = min(low[v], disc[i])
                    
    dfs.time = 0
    
    for i in range(len(graph)):
        if i not in visited:
            dfs(i)
            
    return bridges


def make_eulerian(adj_matrix):

    G = nx.from_numpy_array(np.array(adj_matrix))

    # vérifier si le graph est eulerien
    if nx.is_eulerian(G):
        return G

    # trouver les noeuds avec degrés impairs
    odd_nodes = [node for node, degree in G.degree() if degree % 2 == 1]

    # trouver toutes les paires de noeuds avec degrés impairs
    pairs = []
    while odd_nodes:
        node = odd_nodes.pop()
        for other_node in odd_nodes:
            pairs.append((node, other_node))

    # calculer les plus courtes distances entre chaque paire de noeuds avec degrés impairs
    shortest_paths = {}
    for start, end in pairs:
        shortest_paths[(start, end)] = nx.shortest_path_length(G, start, end)

    # ajouter des arêtes pour connecter les paires de noeuds avec degrés impairs
    # en utilisant les plus courtes distances trouvées précédemment
    for start, end in pairs:
        length = shortest_paths[(start, end)]
        G.add_edge(start, end, weight=length)

    # vérifier si le graphe est maintenant eulerien
    if not nx.is_eulerian(G):
        print("Erreur dans la création du graphe eulérien")

    return G




if nx.is_eulerian(G):
    eulerian_path = fleury(nx.to_numpy_array(G))
    print('Chemin eulérien :', eulerian_path)
else:
    G = make_eulerian(graph)
    non_eulerian_path = fleury(nx.to_numpy_array(G))
    print('Chemin non eulérien :', non_eulerian_path)
   



    
print('Le plus court chemin que doit emprunter le facteur a un poids de :',Chinese_Postman(graph))
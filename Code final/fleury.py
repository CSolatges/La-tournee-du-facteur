import networkx as nx
import matplotlib.pyplot as plt
import numpy as np

def somme_aretes(graph): #caclule la somme totale des arêtes
    somme = 0
    l = len(graph) # On récupère la taille de la matrice carrée
    for i in range(l):
        for j in range(i,l): # On boucle sur toutes les paires d'arêtes dans la matrice carrée
            somme += graph[i][j] # On ajoute le poids de chaque arête à la somme
    return somme


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

    return G


def find_bridges(graph):
    bridges = []  # stockera toutes les arêtes ponts trouvées
    visited = set()  # stockera tous les sommets visités lors de la recherche
    parent = [-1] * len(graph)  # stockera le parent de chaque sommet dans l'arborescence de la recherche
    low = [float('inf')] * len(graph)  # stockera le temps de découverte le plus bas accessible depuis chaque sommet
    disc = [float('inf')] * len(graph)  # stockera le temps de découverte de chaque sommet

    def dfs(v): 
        visited.add(v) # ajoute v aux noeuds visités
        disc[v] = dfs.time # initialise les temps de découverte et de bas niveau pour le noeud v
        low[v] = dfs.time
        dfs.time += 1 # incrémente le temps de découverte
        
        # explore les voisins du noeud v
        for i in range(len(graph)):
            if graph[v][i] != 0:
                # si le voisin n'a pas encore été visité, marque-le comme un enfant de v et explore-le récursivement
                if i not in visited:
                    parent[i] = v
                    dfs(i)
                    # met à jour le bas niveau de v avec le plus petit bas niveau de ses enfants
                    low[v] = min(low[v], low[i])
                    # si le bas niveau de l'enfant i est supérieur au temps de découverte de v, c'est une bridge
                    if low[i] > disc[v]:
                        bridges.append((v, i))
                # si le voisin i est déjà visité et n'est pas un parent direct de v, met à jour le bas niveau de v
                elif i != parent[v]:
                    low[v] = min(low[v], disc[i])

                    
    dfs.time = 0


def fleury(graph):
    # Copie de graph pour éviter de le modifier
    graph_copy = [row[:] for row in graph]
    
    # On récupère le premier sommet
    current_vertex = 0
    # On initialise le chemin
    path = [current_vertex]
    
    while somme_aretes(graph_copy) != 0:
        # On recherche une arête qui n'est pas une arête de pont
        bridges = find_bridges(graph_copy) # on récupère les ponts
        non_bridges = [(i,j) for i in range(len(graph_copy)) for j in range(len(graph_copy)) if graph_copy[i][j]!=0 and (i,j) not in bridges] # on récupère les arêtes qui ne sont pas des ponts
        if len(non_bridges) == 0:
            break
        next_vertex = None
        for edge in non_bridges: # on parcourt les arêtes non ponts
            if edge[0] == current_vertex or edge[1] == current_vertex: # on cherche une arête qui est connectée à l'actuel sommet
                next_vertex = edge[0] + edge[1] - current_vertex # on récupère le prochain sommet à visiter
                graph_copy[edge[0]][edge[1]] = 0 # on enlève l'arête du graphe
                graph_copy[edge[1]][edge[0]] = 0
                break
        if next_vertex is None:
            # Cas où tous les voisins sont des ponts
            next_vertex = path[-2] # on revient en arrière
            graph_copy[current_vertex][next_vertex] = 0 # on enlève l'arête du graphe
            graph_copy[next_vertex][current_vertex] = 0
            path = path[:-1] # on enlève le dernier sommet visité de la liste des sommets visités
        else:
            path.append(next_vertex) # on ajoute le prochain sommet visité à la liste des sommets visités
        current_vertex = next_vertex
    
    return path



graph = [[0, 4, 0, 0, 0, 0, 0, 8, 0], 
         [4, 0, 8, 0, 0, 0, 0, 11, 0], 
         [0, 8, 0, 7, 0, 4, 0, 0, 2], 
         [0, 0, 7, 0, 9, 14, 0, 0, 0], 
         [0, 0, 0, 9, 0, 10, 0, 0, 0], 
         [0, 0, 4, 14, 10, 0, 2, 0, 0], 
         [0, 0, 0, 0, 0, 2, 0, 1, 6], 
         [8, 11, 0, 0, 0, 0, 1, 0, 7], 
         [0, 0, 2, 0, 0, 0, 6, 7, 0]]


G=make_eulerian(graph)

pos = nx.spring_layout(G)
nx.draw(G, pos, with_labels=True)
labels = nx.get_edge_attributes(G, 'weight')
nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)
plt.show()

chemin=fleury(graph)
print('Le chemin le plus court est : ', chemin)
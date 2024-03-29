import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import heapq
from collections import defaultdict


def dijkstra(graph, source, dest):
    
   #Fonction qui implémente l'algorithme de Dijkstra pour trouver le plus court chemin entre deux sommets dans un graphe pondéré.
    
    shortest = [0 for i in range(len(graph))]   # initialisation des poids des sommets à 0
    selected = [source]   # initialisation de la liste des sommets sélectionnés, avec le sommet de départ
    l = len(graph)   # longueur de la matrice graph
    inf = 10000000
    min_sel = inf   # initialisation de la variable min_sel à l'infini
    for i in range(l):
        if(i==source):
            shortest[source] = 0  # le poids du sommet de départ est initialisé à 0
        else:
            if(graph[source][i]==0):
                shortest[i] = inf   # si il n'y a pas d'arête entre le sommet de départ et le sommet courant, le poids du sommet courant est initialisé à l'infini
            else:
                shortest[i] = graph[source][i]   # sinon, le poids du sommet courant est initialisé à la valeur de l'arête qui les relie
                if(shortest[i] < min_sel):
                    min_sel = shortest[i]   # on met à jour la valeur de min_sel si un poids plus petit est trouvé
                    ind = i   # on stocke l'indice du sommet courant dans la variable ind
                
    if(source==dest):
        return 0   # si le sommet de départ et le sommet d'arrivée sont les mêmes, la distance est de 0

    selected.append(ind)   # on ajoute le sommet courant dans la liste des sommets sélectionnés
    while(ind!=dest):
        for i in range(l):
            if i not in selected:   # si le sommet n'a pas été déjà sélectionné
                if(graph[ind][i]!=0):   # si il y a une arête entre le sommet courant et le sommet i
                    if((graph[ind][i] + min_sel) < shortest[i]):   # on vérifie si la distance doit être mise à jour
                        shortest[i] = graph[ind][i] + min_sel   # si c'est le cas, on met à jour le poids du sommet i
        temp = 1000000   # on initialise temp à une grande valeur (ici 1 million)
        for j in range(l):
            if j not in selected:   # si le sommet n'a pas été déjà sélectionné
                if(shortest[j] < temp):   # si le poids du sommet j est plus petit que temp
                    temp = shortest[j]   # on met à jour temp
                    ind = j   # on met à jour l'indice du sommet courant
        min_sel = temp   # on met à jour min_sel avec la valeur de temp
        selected.append(ind)   # on ajoute le sommet courant dans la liste des sommets sélectionnés
    
    return shortest[dest]   # on retourne le poids du sommet d'arrivée qui correspond au poids du plus court chemin entre deux sommets selectionnés


def add_shortest_paths(graph):
    """
    Fonction qui ajoute les poids des chemins minimaux entre toutes les paires de sommets dans un graphe initial.
    """
    l = len(graph)   # longueur de la matrice graph
    new_graph = [[0 for _ in range(l)] for _ in range(l)]   # initialisation du nouveau graphe avec des poids nuls

    for i in range(l):
        for j in range(i + 1, l):   # on parcourt uniquement la moitié supérieure de la matrice (évite la redondance)
            weight = dijkstra(graph, i, j)   # calcul du poids du chemin minimal entre les sommets i et j
            new_graph[i][j] = weight
            new_graph[j][i] = weight   # on attribue le poids symétriquement pour un graphe non orienté

    return new_graph


"""
graph = [[1, 1, 1, 1, 1, 1],
         [1, 1, 1, 1, 1, 1],
         [1, 1, 1, 1, 1, 1],
         [1, 1, 1, 1, 1, 1],
         [1, 1, 1, 1, 1, 1],
         [1, 1, 1, 1, 1, 1]]



# Exemple d'utilisation
graph = [
    [0, 3, 1, 0, 5, 0],
    [3, 0, 0, 1, 0, 6],
    [1, 0, 0, 0, 2, 0],
    [0, 1, 0, 0, 0, 1],
    [5, 0, 2, 0, 0, 4],
    [0, 6, 0, 1, 4, 0],
]
"""
graph = [[0,1,0,0,0], 
          [1,0,0,0,1],
          [0,0,0,1,1], 
          [0,0,1,0,1],
          [0,1,1,1,0]]


G = nx.from_numpy_array(np.array(graph))
pos = nx.spring_layout(G)
nx.draw(G, pos, with_labels=True)
labels = nx.get_edge_attributes(G, 'weight')
nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)
plt.show()

new_graph = add_shortest_paths(graph)
print("le graphe complet est ", new_graph)


#permet d'afficher le graphe avec les poids sur les arêres
G = nx.from_numpy_array(np.array(new_graph))
pos = nx.spring_layout(G)
nx.draw(G, pos, with_labels=True)
labels = nx.get_edge_attributes(G, 'weight')
nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)
#plt.show()

"""
graphe obtenu : 

graph = [
    [0, 3, 1, 4, 3, 5],
    [3, 0, 4, 1, 6, 2],
    [1, 4, 0, 5, 2, 6],
    [4, 1, 5, 0, 5, 1],
    [3, 6, 2, 5, 0, 4],
    [5, 2, 6, 1, 4, 0],
]


"""

import cplex


def find_minimum_matching(graph):
    """
    Fonction qui utilise CPLEX pour trouver le couplage minimal dans un graphe complet donné.
    """
    num_vertices = len(graph)
    edges = [(i, j) for i in range(num_vertices) for j in range(i + 1, num_vertices)]  # liste des arêtes du graphe complet

    # Création de l'objet CPLEX
    problem = cplex.Cplex()
    problem.set_results_stream(None)  # Désactiver les sorties de CPLEX

    # Ajout des variables binaires pour chaque arête
    variables = problem.variables.add(names=[f'x_{i}_{j}' for i, j in edges], types=[problem.variables.type.binary] * len(edges))

    # Ajout de la fonction objective pour maximiser la somme des poids des arêtes sélectionnées
    problem.objective.set_sense(problem.objective.sense.maximize)
    problem.objective.set_linear([(var, graph[i][j]) for (var, (i, j)) in zip(variables, edges)])

    # Ajout de la contrainte pour s'assurer qu'un sommet est incident à au plus une arête sélectionnée
    for vertex in range(num_vertices):
        incident_edges = [var_index for var_index, (i, j) in enumerate(edges) if i == vertex or j == vertex]
        problem.linear_constraints.add(lin_expr=[cplex.SparsePair(ind=incident_edges, val=[1] * len(incident_edges))],
                                       senses=['L'],
                                       rhs=[1])

    # Résolution du problème
    problem.solve()

    # Récupération du couplage minimal
    minimum_matching = [(i, j) for (var, (i, j)) in zip(variables, edges) if problem.solution.get_values(var) == 1]

    return minimum_matching



minimum_matching = find_minimum_matching(new_graph)
print("les arêtes de couplage minimal sont :", minimum_matching)




#il faut doubler les chemins obtenus par le couplage minimal dans le graphe de départ


def double_shortest_paths(graph, pairs):
    """
    Fonction qui double toutes les arêtes du plus court chemin entre chaque paire de sommets donnée.
    Utilise l'algorithme de Dijkstra pour calculer les plus courts chemins.
    """

    def dijkstra(graph, start):
        """
        Algorithme de Dijkstra pour trouver les plus courts chemins à partir d'un sommet de départ donné.
        """
        distances = defaultdict(lambda: float('inf'))
        distances[start] = 0

        pq = [(0, start)]

        while pq:
            dist, node = heapq.heappop(pq)

            if dist > distances[node]:
                continue

            for neighbor, weight in enumerate(graph[node]):
                if weight > 0:
                    new_dist = dist + weight
                    if new_dist < distances[neighbor]:
                        distances[neighbor] = new_dist
                        heapq.heappush(pq, (new_dist, neighbor))

        return distances

    doubled_graph = [row[:] for row in graph]  # Copie du graphe initial

    for u, v in pairs:
        distances = dijkstra(graph, u)  # Calcul des plus courts chemins à partir de u

        # Récupère le plus court chemin entre u et v
        path = [v]
        while path[-1] != u:
            prev = distances[path[-1]]
            for neighbor, weight in enumerate(graph[path[-1]]):
                if weight > 0 and distances[neighbor] + weight == prev:
                    path.append(neighbor)
                    break

        # Double les arêtes du chemin
        for i in range(len(path) - 1):
            node1, node2 = path[i], path[i + 1]
            doubled_graph[node1][node2] += 1
            doubled_graph[node2][node1] += 1

    return doubled_graph

pairs = find_minimum_matching(new_graph)
doubled_graph = double_shortest_paths(graph, pairs)
print("Le graphe doublé est :", doubled_graph)


def create_new_graph(graph, minimum_matching):
    num_vertices = len(graph)
    new_graph = nx.from_numpy_array(np.array(graph))  # Conversion de la matrice d'adjacence en graphe NetworkX
    for edge in minimum_matching:
        u, v = edge
        new_graph.add_edge(u, v)  # Ajout des arêtes du couplage minimal
    adj_matrix = nx.to_numpy_array(new_graph)  # Conversion en matrice d'adjacence
    return adj_matrix


minimum_matching = find_minimum_matching(new_graph)
adj_matrix = create_new_graph(graph, minimum_matching)

print("nouveau graphe obtenu à partir du graphe initial et du couplage min :",adj_matrix)
#ça marche pas donc à modifier 

#ensuite il faut trouver le chemin eulerien dans le graphe obtenus en doublant les arêtes et c'est terminé
#il reste juste ça à faire mc si jamais tu y arrives 


def find_eulerian_path(graph):
    """
    Fonction qui trouve un chemin eulérien dans un graphe s'il en existe un.
    """
    num_vertices = len(graph)

    # Vérifier si le graphe est eulérien
    for v in range(num_vertices):
        if sum(graph[v]) % 2 != 0:  # Si un sommet a un degré impair, le graphe n'est pas eulérien
            return None

    # Copie du graphe pour la suppression des arêtes visitées
    graph_copy = [row[:] for row in graph]

    # Fonction récursive pour trouver un chemin eulérien
    def eulerian_path(v, path):
        for u in range(num_vertices):
            if graph_copy[v][u] > 0:  # Si l'arête n'a pas été visitée
                graph_copy[v][u] -= 1  # Marquer l'arête comme visitée
                graph_copy[u][v] -= 1
                eulerian_path(u, path)  # Récursivement, trouver un chemin eulérien à partir du sommet suivant
        path.append(v)  # Ajouter le sommet au chemin

    # Trouver le sommet de départ pour le chemin eulérien
    start_vertex = 0
    for v in range(num_vertices):
        if sum(graph[v]) > 0:  # Trouver le sommet avec une arête non visitée
            start_vertex = v
            break

    # Initialiser le chemin eulérien
    eulerian_path_list = []
    eulerian_path(start_vertex, eulerian_path_list)

    eulerian_path_list.reverse()  # Inverser le chemin pour obtenir l'ordre correct

    return eulerian_path_list


eulerian_path = find_eulerian_path(doubled_graph)

if eulerian_path is None:
    print("Le graphe n'a pas de chemin eulérien.")
else:
    print("Chemin eulérien :", eulerian_path)

def get_odd(graph): #à utiliser avec le graphe doublé
    degrees = [0 for i in range(len(graph))]  # Création d'une liste de degrés pour chaque sommet, initialisée à 0
    for i in range(len(graph)):
        for j in range(len(graph)):
                if(graph[i][j]!=0):
                    degrees[i]+=1   # Pour chaque sommet, parcours de tous les voisins pour calculer son degré

    odds = [i for i in range(len(degrees)) if degrees[i]%2!=0]  # Récupération des sommets de degré impair
    return odds  # Retourne la liste des sommets de degré impair

odds = get_odd(doubled_graph)

print('Les sommets de degré impairs de ce graphe sont : ', odds)

def create_subgraph(graph, odd_vertices):
    subgraph = [[] for _ in range(len(odd_vertices))]

    for u in odd_vertices:
        for v in graph[u]:
            if v in odd_vertices:
                subgraph[odd_vertices.index(u)].append(0)  # Attribution du poids nul à chaque arête

    return subgraph



subgraph = create_subgraph(doubled_graph, odds)
print("Le sous-graphe formé par les sommets de degré impair est :", subgraph)

nouveau_couplage = find_minimum_matching(subgraph)
print("Le couplage minimal trouvé dans le graphe obtenu à partir des sommets impairs est :", nouveau_couplage)

graphe_couple=create_new_graph(doubled_graph, nouveau_couplage)
print("Le sous-graphe formé par le couplage et le graphe doublé est :", graphe_couple)


"""

def get_odd(graph):
    degrees = [0 for i in range(len(graph))]  # Création d'une liste de degrés pour chaque sommet, initialisée à 0
    for i in range(len(graph)):
        for j in range(len(graph)):
                if(graph[i][j]!=0):
                    degrees[i]+=1   # Pour chaque sommet, parcours de tous les voisins pour calculer son degré

    odds = [i for i in range(len(degrees)) if degrees[i]%2!=0]  # Récupération des sommets de degré impair
    return odds  # Retourne la liste des sommets de degré impair




odds=get_odd(adj_matrix)
print("les sommets de degré impairs sont :", odds)



def connect_odd_vertices(graph, odd_vertices):

    new_graph = nx.Graph(graph)  # Créez une copie du graphe initial

    for u in odd_vertices:
        shortest_paths = nx.single_source_dijkstra_path(graph, u)  # Calculez les chemins les plus courts depuis le sommet u

        for v in odd_vertices:
            if u != v and v in shortest_paths:
                # Ajoutez l'arête de poids minimal entre les sommets de degré impair
                min_weight_edge = None
                min_weight = float('inf')

                for i in range(len(shortest_paths[v]) - 1):
                    src = shortest_paths[v][i]
                    dest = shortest_paths[v][i+1]
                    weight = graph[src][dest]['weight']
                    
                    if weight < min_weight:
                        min_weight = weight
                        min_weight_edge = (src, dest)

                if min_weight_edge:
                    new_graph.add_edge(*min_weight_edge, weight=min_weight)

    return new_graph

# Convertir la matrice d'adjacence en graphe
graph = nx.from_numpy_array(adj_matrix)

# Utiliser le graphe pour connecter les sommets de degré impair
eulerian_graph = connect_odd_vertices(graph, odds)

nx.draw(eulerian_graph, with_labels=True)
plt.show()



#

# Vérification du résultat
print("le graphe eulerien est :", eulerian_graph)

# Obtenez le chemin eulérien
eulerian_path = list(nx.eulerian_circuit(eulerian_graph))

# Affichez le chemin eulérien
print(eulerian_path)

"""
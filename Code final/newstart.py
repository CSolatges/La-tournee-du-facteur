import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import heapq
from collections import defaultdict
from itertools import combinations
import cplex
from scipy.optimize import linear_sum_assignment



graph = [
    [0, 3, 1, 4, 3, 5],
    [3, 0, 4, 1, 6, 2],
    [1, 4, 0, 5, 2, 6],
    [4, 1, 5, 0, 5, 1],
    [3, 6, 2, 5, 0, 4],
    [5, 2, 6, 1, 4, 0],
]
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


G = nx.from_numpy_array(np.array(graph))
pos = nx.spring_layout(G)
nx.draw(G, pos, with_labels=True)
labels = nx.get_edge_attributes(G, 'weight')
nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)
plt.show()

graphe_complet = add_shortest_paths(graph)
print("le graphe complet est ", graphe_complet)


#permet d'afficher le graphe avec les poids sur les arêres
G = nx.from_numpy_array(np.array(graphe_complet))
pos = nx.spring_layout(G)
nx.draw(G, pos, with_labels=True)
labels = nx.get_edge_attributes(G, 'weight')
nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)
#plt.show()




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
    variables = problem.variables.add(names=[f'y_{i}_{j}' for i, j in edges], types=[problem.variables.type.binary] * len(edges))

    # Ajout de la fonction objective pour minimiser la somme des coûts des arêtes sélectionnées
    problem.objective.set_sense(problem.objective.sense.minimize)
    problem.objective.set_linear([(var, graph[i][j]) for (var, (i, j)) in zip(variables, edges)])

    # Ajout de la contrainte pour chaque sommet, il y a une et une seule arête sélectionnée
    for vertex in range(num_vertices):
        incident_edges = [var_index for var_index, (i, j) in enumerate(edges) if i == vertex or j == vertex]
        problem.linear_constraints.add(lin_expr=[cplex.SparsePair(ind=incident_edges, val=[1] * len(incident_edges))],
                                       senses=['E'],
                                       rhs=[1])

    # Résolution du problème
    problem.solve()

    # Récupération du couplage minimal
    minimum_matching = [(i, j) for (var, (i, j)) in zip(variables, edges) if problem.solution.get_values(var) == 1]

    return minimum_matching




minimum_matching = find_minimum_matching(graphe_complet)
print("les arêtes de couplage minimal sont :", minimum_matching)

def double_edges(graph, matching):
    G = nx.Graph()
    for i in range(len(graph)):
        for j in range(len(graph[i])):
            if graph[i][j] != 0:
                G.add_edge(i, j, weight=graph[i][j])
    doubled_edges = []
    for u, v in matching:
        shortest_path = nx.shortest_path(G, u, v, weight='weight')
        for i in range(len(shortest_path) - 1):
            edge_weight = G[shortest_path[i]][shortest_path[i + 1]]['weight']
            doubled_edges.append((shortest_path[i], shortest_path[i + 1], edge_weight))
            doubled_edges.append((shortest_path[i + 1], shortest_path[i], edge_weight))  # Ajouter l'arête en sens inverse

    # Ajouter les autres arêtes du graphe qui ne sont pas dans le couplage minimal
    for u, v, weight in G.edges(data='weight'):
        if (u, v) not in matching and (v, u) not in matching:
            doubled_edges.append((u, v, weight))

    return doubled_edges

doubled_edges = double_edges(graph, minimum_matching)
print("Liste d'arêtes avec les arêtes dans le graphe doublé :", doubled_edges)

#après là jsp quoi faire


# Création du graphe
G = nx.Graph()
G.add_weighted_edges_from(doubled_edges)

# Trouver les sommets de degré impair
odd_vertices = [v for v, d in G.degree() if d % 2 == 1]

# Relier les sommets de degré impair par des arêtes supplémentaires
double_edges = []
for i in range(len(odd_vertices) - 1):
    for j in range(i + 1, len(odd_vertices)):
        u = odd_vertices[i]
        v = odd_vertices[j]
        weight = 1  # Poids de l'arête supplémentaire (à définir selon vos besoins)
        double_edges.append((u, v, weight))
        double_edges.append((v, u, weight))

# Ajouter les arêtes supplémentaires au graphe
G.add_weighted_edges_from(double_edges)

# Afficher toutes les arêtes du graphe
print("Toutes les arêtes du graphe :")
print(G.edges(data=True))


# Création du graphe
G = nx.Graph()
G.add_weighted_edges_from(double_edges)

# Trouver les sommets de degré impair
odd_vertices = [v for v, d in G.degree() if d % 2 == 1]

# Afficher les sommets de degré impair
print("Sommet(s) de degré impair :")
print(odd_vertices)



"""
def find_odd_degree_vertices(graph):
    odd_degree_vertices = []
    for node in graph.nodes:
        if graph.degree(node) % 2 != 0:
            odd_degree_vertices.append(node)
    return odd_degree_vertices



# Trouver les sommets de degré impair
odd_degree_vertices = find_odd_degree_vertices(doubled_graph)

print('Les sommets de degré impairs de ce graphe sont : ',odd_degree_vertices)


def get_subgraph_with_odd_degrees(graph, vertices):
    odd_degree_vertices = [v for v in vertices if graph.degree[v] % 2 != 0]
    subgraph = graph.subgraph(odd_degree_vertices)

    num_vertices = len(odd_degree_vertices)
    adjacency_matrix = np.zeros((num_vertices, num_vertices))

    for u, v in subgraph.edges():
        weight = graph[u][v]['weight']
        u_index = odd_degree_vertices.index(u)
        v_index = odd_degree_vertices.index(v)
        adjacency_matrix[u_index][v_index] = weight
        adjacency_matrix[v_index][u_index] = weight

    return adjacency_matrix

subgraph_imp=get_subgraph_with_odd_degrees(doubled_graph, odd_degree_vertices)

print("graphe obtenu avec les sommets de degré impairs du graphe doublé :",subgraph_imp)


# Création du graphe initial
G = nx.Graph()
num_nodes = len(graph)
G.add_nodes_from(range(num_nodes))
for i in range(num_nodes):
    for j in range(i + 1, num_nodes):
        if graph[i][j] != 0:
            G.add_edge(i, j, weight=graph[i][j])

# Vérification si le graphe est fortement connexe
if not nx.is_connected(G):
    print("Le graphe n'est pas fortement connexe.")
    exit()

# Trouver les sommets de degré impair dans le graphe initial
odd_degree_vertices = [node for node in G.nodes if G.degree(node) % 2 != 0]

# Ajouter les arêtes manquantes pour rendre le graphe eulerien
for u in odd_degree_vertices:
    min_distance = float('inf')
    closest_vertex = None
    for v in odd_degree_vertices:
        if u != v:
            distance = nx.shortest_path_length(G, source=u, target=v, weight='weight')
            if distance < min_distance:
                min_distance = distance
                closest_vertex = v
    if closest_vertex is not None:
        G.add_edge(u, closest_vertex)

# Trouver un circuit eulerien dans le graphe
eulerian_circuit = list(nx.eulerian_circuit(G))

# Récupérer le chemin final en enlevant les doublons
final_path = [node for node, _ in eulerian_circuit]

print('Le chemin final pour le problème du postier chinois est :', final_path)

"""
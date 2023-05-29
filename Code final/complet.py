import networkx as nx
import matplotlib.pyplot as plt
import numpy as np

def dijkstra(graph, source, dest):
    """
   Fonction qui implémente l'algorithme de Dijkstra pour trouver le plus court chemin entre deux sommets dans un graphe pondéré.
    """
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


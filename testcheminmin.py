import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict



graph =     [[0,1,0,0,0,0,0,0,0],   #graphinsa impossible de transformer en eulérien
                [1,0,1,0,0,0,0,1,0],
                [0,1,0,1,0,0,1,0,0],
                [0,0,1,0,1,1,0,0,0],
                [0,0,0,1,0,0,0,0,0],
                [0,0,0,1,0,0,1,0,0],
                [0,0,1,0,0,1,0,1,1],
                [0,1,0,0,0,0,1,0,1],
                [0,0,0,0,0,0,1,1,0]]

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
        path = nx.shortest_path(G, start, end)
        for i in range(len(path) - 1):
            u, v = path[i], path[i + 1]
            G.add_edge(u, v, weight=length)
    eulerian_cycle=[]
    # vérifier si le graph est maintenant eulerien
    if nx.is_eulerian(G):
        

    # trouver un cycle eulerien dans le graph
     eulerian_cycle = list(nx.eulerian_circuit(G))

    # créer un nouveau graph avec le cycle eulerien
    H = nx.Graph()
    for u, v in eulerian_cycle:
        H.add_edge(u, v)

    return H


def trouver_chemin_minimal(graphe, graphe_eulerien):
    # Trouver la liste des sommets de degré impair dans le graphe eulerien
    sommets_impairs = [sommet for sommet in range(len(graphe_eulerien)) if sum(graphe_eulerien[sommet]) % 2 == 1]
    
    # Initialiser le chemin avec le premier sommet impair
    chemin = [sommets_impairs[0]]
    
    # Parcourir chaque sommet impair et ajouter le chemin minimal pour atteindre le prochain sommet impair
    for i in range(1, len(sommets_impairs)):
        chemin_minimal = dijkstra(graphe_eulerien, sommets_impairs[i-1], sommets_impairs[i])
        chemin += chemin_minimal[1:] # ajouter tous les sommets du chemin sauf le premier qui a déjà été visité
    
    # Retourner le chemin complet en supprimant les fausses arêtes ajoutées
    chemin_complet = []
    for i in range(len(chemin)-1):
        chemin_complet += dijkstra(graphe, chemin[i], chemin[i+1])[1:] # ajouter tous les sommets du chemin sauf le premier qui a déjà été visité
    chemin_complet.append(chemin[0]) # ajouter le point de départ à la fin
    
    return chemin_complet

graphe_eulerien =make_eulerian(graph)
path=trouver_chemin_minimal(graph, graphe_eulerien)

print('chemin min :', path)
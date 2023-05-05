import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict



"""

graph = [[0,0,1,0],
          [0,0,1,1], #manque le retour à 0
          [1,1,0,1],
          [0,1,1,0]]




graph = [[0,1,0,0,0], 
          [1,0,0,0,1],
          [0,0,0,1,1], #ok
          [0,0,1,0,1],
          [0,1,1,1,0]]


"""
graph = [[0,1,1,1,1],
          [1,0,1,0,0],   #ok
          [1,1,0,0,0],
          [1,0,0,0,1],
          [1,0,0,1,0]]

"""

graph = [[0, 4, 0, 0, 0, 0, 0, 8, 0], 
         [4, 0, 8, 0, 0, 0, 0, 11, 0], 
         [0, 8, 0, 7, 0, 4, 0, 0, 2], 
         [0, 0, 7, 0, 9, 14, 0, 0, 0], 
         [0, 0, 0, 9, 0, 10, 0, 0, 0], 
         [0, 0, 4, 14, 10, 0, 2, 0, 0], 
         [0, 0, 0, 0, 0, 2, 0, 1, 6], 
         [8, 11, 0, 0, 0, 0, 1, 0, 7], 
         [0, 0, 2, 0, 0, 0, 6, 7, 0]]


graph =                [[0, 3, 1, 0, 5, 0], 
                        [3, 0, 0, 1, 0, 6], 
                        [1, 0, 0, 0, 2, 0],  #ok
                        [0, 1, 0, 0, 0, 1], 
                        [5, 0, 2, 0, 0, 4], 
                        [0, 6, 0, 1, 4, 0], 
                         
                    ]; 



graph =     [[0,1,0,0,0,0,0,0,0],   #graphinsa impossible de transformer en eulérien ça marche po
                [1,0,1,0,0,0,0,1,0],
                [0,1,0,1,0,0,1,0,0],
                [0,0,1,0,1,1,0,0,0],
                [0,0,0,1,0,0,0,0,0],
                [0,0,0,1,0,0,1,0,0],
                [0,0,1,0,0,1,0,1,1],
                [0,1,0,0,0,0,1,0,1],
                [0,0,0,0,0,0,1,1,0]]

"""            


G = nx.from_numpy_array(np.array(graph))

nx.draw(G, with_labels=True)
plt.show() #permet d'afficher le graphe

def somme_aretes(graph): #caclule la somme totale des arêtes
    somme = 0
    l = len(graph) # On récupère la taille de la matrice carrée
    for i in range(l):
        for j in range(i,l): # On boucle sur toutes les paires d'arêtes dans la matrice carrée
            somme += graph[i][j] # On ajoute le poids de chaque arête à la somme
    return somme

            


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



def get_odd(graph):
    degrees = [0 for i in range(len(graph))]  # Création d'une liste de degrés pour chaque sommet, initialisée à 0
    for i in range(len(graph)):
        for j in range(len(graph)):
                if(graph[i][j]!=0):
                    degrees[i]+=1   # Pour chaque sommet, parcours de tous les voisins pour calculer son degré

    odds = [i for i in range(len(degrees)) if degrees[i]%2!=0]  # Récupération des sommets de degré impair
    #print('Les sommets de degré impairs de ce graphe sont : ', odds)
    #print('\n')
    return odds  # Retourne la liste des sommets de degré impair



def gen_pairs(odds):
    """
    Cette fonction prend en entrée une liste d'entiers impairs (odds) représentant les sommets de degré impair d'un graphe.
    Elle renvoie une liste de toutes les paires de sommets possibles parmis les sommets impairs.
    """
    pairs = []
    for i in range(len(odds)-1):
        pairs.append([])
        for j in range(i+1,len(odds)):
            pairs[i].append([odds[i],odds[j]])
        
    #print('Les paires possibles entre ces sommets sont :',pairs)
    #print('\n')
    return pairs


def Chinese_Postman(graph):
    # Trouver les noeuds avec des degrés impairs
    odds = get_odd(graph)
    
    # Si aucun noeud avec un degré impair, alors le graphe est déjà un chemin eulerien
    if(len(odds)==0):
        return somme_aretes(graph)
    
    # Générer toutes les paires possibles de noeuds impairs
    pairs = gen_pairs(odds)
    
    # Nombre de paires de noeuds impairs pour les apparier
    l = (len(pairs)+1)//2
    
    # Liste pour stocker les paires de noeuds impairs appariées
    pairings_sum = []
    
    # Fonction récursive pour appairer les noeuds impairs
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
            
    # Appairer les noeuds impairs
    get_pairs(pairs)
    
    # Calculer les distances les plus courtes pour chaque paire de noeuds impairs appariés
    min_sums = []
    for i in pairings_sum:
        s = 0
        for j in range(len(i)):
            s += dijkstra(graph, i[j][0], i[j][1])
        min_sums.append(s)
    
    # Calculer la distance ajoutée pour traverser chaque paire de noeuds impairs appariés
    added_dis = min(min_sums)
    
    # Calculer la distance totale pour parcourir tous les chemins en utilisant l'algorithme chinois du postier
    chinese_dis = added_dis + somme_aretes(graph)
    
    # Retourner la distance totale
    return chinese_dis
  

def fleury(graph): #permet de trouver un chemin eluérien
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


    return G
"""
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


"""
#appel des fonctions pour afficher le chemins 

if nx.is_eulerian(G):  #affichage du chemin si le graphe est eulérien
    eulerian_path = fleury(nx.to_numpy_array(G))
    print('Le plus court chemin que peut emprunter le facteur est :', eulerian_path)
    print('\n')
else:
    G = make_eulerian(graph) #si il ne l'est pas 
    non_eulerian_path = fleury(nx.to_numpy_array(G))
    print('Le plus court chemin que peut emprunter le facteur est :', non_eulerian_path)
    print('\n')




#affichage du poids
    
print('Le plus court chemin que doit emprunter le facteur a un poids de :',Chinese_Postman(graph))
print('\n')

#permet d'afficher le graphe avec les poids sur les arêres

pos = nx.spring_layout(G)
nx.draw(G, pos, with_labels=True)
labels = nx.get_edge_attributes(G, 'weight')
nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)
plt.show()
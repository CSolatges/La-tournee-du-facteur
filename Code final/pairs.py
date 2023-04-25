import networkx as nx
import matplotlib.pyplot as plt
import numpy as np

def get_odd(graph):
    degrees = [0 for i in range(len(graph))]  # Création d'une liste de degrés pour chaque sommet, initialisée à 0
    for i in range(len(graph)):
        for j in range(len(graph)):
                if(graph[i][j]!=0):
                    degrees[i]+=1   # Pour chaque sommet, parcours de tous les voisins pour calculer son degré

    odds = [i for i in range(len(degrees)) if degrees[i]%2!=0]  # Récupération des sommets de degré impair
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
    return pairs


graph = [[0, 4, 0, 0, 0, 0, 0, 8, 0], 
         [4, 0, 8, 0, 0, 0, 0, 11, 0], 
         [0, 8, 0, 7, 0, 4, 0, 0, 2], 
         [0, 0, 7, 0, 9, 14, 0, 0, 0], 
         [0, 0, 0, 9, 0, 10, 0, 0, 0], 
         [0, 0, 4, 14, 10, 0, 2, 0, 0], 
         [0, 0, 0, 0, 0, 2, 0, 1, 6], 
         [8, 11, 0, 0, 0, 0, 1, 0, 7], 
         [0, 0, 2, 0, 0, 0, 6, 7, 0]]


odds = get_odd(graph)
pairs=gen_pairs(odds)
print('Les sommets de degré impairs de ce graphe sont : ', odds)
print('\n')
print('Les paires possibles entre ces sommets sont :',pairs)

G = nx.from_numpy_array(np.array(graph))
pos = nx.spring_layout(G)
nx.draw(G, pos, with_labels=True)
labels = nx.get_edge_attributes(G, 'weight')
nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)
plt.show()
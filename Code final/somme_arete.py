import networkx as nx
import matplotlib.pyplot as plt
import numpy as np


graph = [[0, 4, 0, 0, 0, 0, 0, 8, 0], 
         [4, 0, 8, 0, 0, 0, 0, 11, 0], 
         [0, 8, 0, 7, 0, 4, 0, 0, 2], 
         [0, 0, 7, 0, 9, 14, 0, 0, 0], 
         [0, 0, 0, 9, 0, 10, 0, 0, 0], 
         [0, 0, 4, 14, 10, 0, 2, 0, 0], 
         [0, 0, 0, 0, 0, 2, 0, 1, 6], 
         [8, 11, 0, 0, 0, 0, 1, 0, 7], 
         [0, 0, 2, 0, 0, 0, 6, 7, 0]]



def somme_aretes(graph): #caclule la somme totale des arêtes
    somme = 0
    l = len(graph) # On récupère la taille de la matrice carrée
    for i in range(l):
        for j in range(i,l): # On boucle sur toutes les paires d'arêtes dans la matrice carrée
            somme += graph[i][j] # On ajoute le poids de chaque arête à la somme
    return somme


somme=somme_aretes(graph)
print('La somme des arêtes est : ', somme)
G = nx.from_numpy_array(np.array(graph))
nx.draw(G, with_labels=True)
plt.show() #permet d'afficher le graphe
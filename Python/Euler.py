## L'objectif de ce code est de déterminer les cycles Eulérien d'un graphe par l'algorithme d'Euler

#Écrire une méthode qui retourne le nombre de sommets de degrés impairs dans un graphe.77

import numpy as np

#Basé sur le TD algo graphes eulériens


"Cette partie permet de déterminer si un graphe admet un cycle ou une chaîne eulerienne"

def nbimpaires(G):
    imp=0
    for s in range (len(G)): #parcours les sommets de G
        arrete=0
        for i in range(len(G[s])): #on condère que les sommets sontdes listes remplis de 1 et de 0
            arrete+=i
        if arrete%2==1:
            imp+=1 
    return (imp)

def cycleul(G): 
    #Un graphe connexe admet un cycle eulérien si et seulement si il ne possède aucun sommet de degré impair.
    nbimp=nbimpaires(G)
    if nbimp==0:
        return True
    else:
        return False
    
def chaineul(G):
    #Un graphe connexe admet une chaîne eulérien si et seulement si il ne possède 0 ou 2 de degré impair.
    nbimp=nbimpaires(G)
    if nbimp==0:
        return True
    elif nbimp==2:
        return True
    else:
        return False
    

"Ces fonctions permettent de trouver un cycle quelconque dans un graphe par parcous en profondeur du graphe"

from collections import defaultdict

def findCycleUtil(v, visited, parent, adjacency_matrix, cycle, s):
    visited[v] = True
    cycle.append(v)

    for i in range(len(adjacency_matrix)):
        if adjacency_matrix[v][i] == 1:
            if visited[i] == False:
                if findCycleUtil(i, visited, v, adjacency_matrix, cycle, s):
                    return True
            elif parent != i and i == s:
                cycle.append(i)
                return True

    return False

def trouverCycle(adjacency_matrix,s):
    visited = [False] * len(adjacency_matrix)
    cycle = []

    for i in range(len(adjacency_matrix)):
        if visited[i] == False:
            if findCycleUtil(i, visited, -1, adjacency_matrix, cycle, s):
                return cycle

    return cycle



"Cette partie permet de déterminer un cycle Eulérien"

def cycleulerien(G,s):
    #fonction qui retourne une cycle eulérien dans le graphe G en partant d'un sommet S
    if cycleul(G):
        A=[] #liste des sommets adjacents à S
        for i in range(len(G[s])):
            if G[s][i]!=0:
                A.append(i) 
        if A==[]:
            return ([s])
        else: 
            C=trouverCycle(G,s) #un cycle quelconque dans G qui part de s
            print ("C:",C)
            #ici on va supprimer les arrêtes de ce cycle dans G
            l=len(C)
            for i in range(1,l):
                if G[C[i-1]][C[i]]!=0:
                    G[C[i-1]][C[i]]-=1
            R=[]
            for i in range(l):
                R+=(cycleulerien(G,C[i]))
            print("R:", R)
            return (R)
    else:
        return("pas de cycle eulerien dans ce graphe")
        


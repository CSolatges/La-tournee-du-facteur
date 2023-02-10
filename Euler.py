## L'objectif de ce code est de déterminer les cycles Eulérien d'un graphe par l'algorithme d'Euler

#Écrire une méthode qui retourne le nombre de sommets de degrés impairs dans un graphe.77

import numpy as np

#Basé sur le TD algo graphes eulériens


"Cette partie permet de déterminer si un graphe admet un cycle ou une chaîne eulerienne"

def nbimpaires(G):
    imp=0
    for sommet in G: #parcours les sommets de G
        arrete=0
        for i in len(sommet): #on condère que les sommets sontdes listes remplis de 1 et de 0
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
    

"Cette partie permet de déterminer un cycle Eulérien"

def cycleulerien(G,s):
    #fonction qui retourne une cycle eulérien dans le graphe G en partant d'un sommet S
    A=[]
    for i in range(len(s)):
        if s[i]!=0:
            A.append(i) 
    if A==[]:
        return ([s])
    else:
        C=[] #un cycle quelconque dans G 
        #ici on va supprimer les arrêtes de ce cycle dans G
        #Je sais pas encore comment faire ça
        l=len(C)
        R=[]
        for i in range(l):
            R.append(cycleulerien(G,C[i]))
        return (R)
    
        


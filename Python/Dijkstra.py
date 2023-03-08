## L'objectif de ce code est de déterminer les plus courts chemin par Algorithme de Dijkstra
import numpy as np


def dijkstraDist(G, depart):
    "fonction qui prend en agument la matrice d'adjacence d'un graphe et le point de départ et retourne la distance du "
    "plus court chemin de ce point vers tous les autres points du graphe"
    
    X = np.size(G,0) #nombre de sommets du graphe
    # Initialisation du tableau des plus courts chemins
    # Le booléen pour savoir si le sommet a déjà été sélectionné
    pcc = list()
    for i in range(X):
        pcc.append([np.inf, False, None])
    sommet_u = depart
    dist_u = 0
    pcc[depart][0] = 0
    pcc[depart][1] = True
    cpt = 0 #Compteur du nombre de sommets sélectionné
    while cpt != X-1:
        # À chaque étape, la solution optimale doit être conservée
        # (pour sélection du sommet correspondant à l’étape suivante)
        minimum = np.inf 
        
        for k in range(X):
            if pcc[k][1] == False: #On s'assure que le sommet k n'a pas encore été selectionné
                dist_uv = G[sommet_u][k]
                dist_totale = dist_u + dist_uv# Distance totale du chemin s -> ... -> u -> v

                # Mise à jour du tableau des plus courts chemins
                if dist_totale < pcc[k][0]:
                    pcc[k][0] = dist_totale
                    pcc[k][2] = sommet_u
                # Mise à jour de la solution minimale à cette étape
                if pcc[k][0] < minimum:
                    minimum = pcc[k][0]
                    prochain_sommet_select = k

        cpt = cpt + 1
        sommet_u = prochain_sommet_select #Mise à jour du sommet d'étude
        pcc[sommet_u][1] = True 
        dist_u = pcc[sommet_u][0] #Mise à jour de la distance
    return(pcc)

def dijkstraPCC(G, depart, arrivee):
    pcc = dijkstraDist(G, depart) #On récupère les plus courts chemins
    chemin = list()
    # On reconstitue le plus court chemin d’arrivee vers depart
    ville = arrivee
    chemin.append(ville)
    while ville != depart:
        ville = pcc[ville][2]
        chemin.append(ville)
    return(list(reversed(chemin))) #on affiche la liste dans l'autre sens pour avoir le trajet du départ à l'arrivée
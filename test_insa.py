import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import heapq

# Initialisez la matrice d'adjacence avec des zéros
matrice_adj = [[0,1,0,0,0,0,0,0,0],
                [1,0,1,0,0,0,0,1,0],
                [0,1,0,1,0,0,1,0,0],
                [0,0,1,0,1,1,0,0,0],
                [0,0,0,1,0,0,0,0,0],
                [0,0,0,1,0,0,1,0,0],
                [0,0,1,0,0,1,0,1,1],
                [0,1,0,0,0,0,1,0,1],
                [0,0,0,0,0,0,1,1,0]]

G = nx.from_numpy_array(np.array(matrice_adj))
nx.draw(G, with_labels=True)
#plt.show() #permet d'afficher le graphe

def trouver_cycle_eulerien(matrice_adj, sommet_de_depart): #Hierholzer 
    # Initialisez une pile pour stocker le chemin actuel
    pile = [sommet_de_depart]
    # Initialisez une liste pour stocker le cycle eulérien final
    cycle_eulerien = []

    while pile:
        sommet = pile[-1]

        # Trouvez le prochain sommet adjacent non visité
        for adj in range(len(matrice_adj[sommet])):
            if matrice_adj[sommet][adj] > 0:
                pile.append(adj)
                matrice_adj[sommet][adj] -= 1
                matrice_adj[adj][sommet] -= 1
                break
        else:
            cycle_eulerien.append(pile.pop())

    # Inversez la liste pour obtenir le cycle eulérien dans l'ordre correct
    cycle_eulerien.reverse()

    return cycle_eulerien



def dijkstra(matrice_adj, sommet_de_depart): #qui est utilisé quand le graphe n'est pas eulérien
    # Initialisez la liste des distances minimales avec une distance infinie pour tous les sommets
    distance = [float('inf')] * len(matrice_adj)
    # La distance de départ est de 0
    distance[sommet_de_depart] = 0
    # Initialisez une file de priorité avec le sommet de départ et sa distance
    file_de_priorite = [(0, sommet_de_depart)]

    while file_de_priorite:
        (dist, sommet) = heapq.heappop(file_de_priorite)

        # Vérifiez si la nouvelle distance est plus courte que la distance actuelle
        if dist > distance[sommet]:
            continue

        # Parcourez tous les sommets adjacents
        for adj in range(len(matrice_adj[sommet])):
            # Ignorez les sommets non connectés
            if matrice_adj[sommet][adj] == 0:
                continue

            # Calculez la nouvelle distance
            new_distance = distance[sommet] + matrice_adj[sommet][adj]

            # Vérifiez si la nouvelle distance est plus courte que la distance actuelle
            if new_distance < distance[adj]:
                # Mettez à jour la distance minimale
                distance[adj] = new_distance
                # Ajoutez le sommet à la file de priorité avec sa nouvelle distance
                heapq.heappush(file_de_priorite, (new_distance, adj))

    # Retournez la liste des distances minimales
    return distance

def creer_sous_graphe_eulerien(matrice_adj):
    # Trouvez tous les sommets de degré impair
    sommets_impairs = [sommet for sommet in range(len(matrice_adj)) if sum(matrice_adj[sommet]) % 2 != 0]
    # Si tous les sommets ont un degré pair, le graphe est déjà eulérien
    if not sommets_impairs:
        return matrice_adj

    # Utilisez l'algorithme de Dijkstra pour trouver les plus courts chemins entre les sommets impairs
    chemins = {}
    for i in range(len(sommets_impairs)):
        for j in range(i+1, len(sommets_impairs)):
            distance = dijkstra(matrice_adj, sommets_impairs[i])[sommets_impairs[j]]
            chemins[(sommets_impairs[i], sommets_impairs[j])] = distance
            chemins[(sommets_impairs[j], sommets_impairs[i])] = distance

    # Créez un nouveau graphe en connectant les sommets impairs avec les plus courts chemins
    nouveau_graphe = [[0 for _ in range(len(matrice_adj))] for _ in range(len(matrice_adj))]
    for i in range(len(sommets_impairs)):
        for j in range(i+1, len(sommets_impairs)):
            nouveau_graphe[sommets_impairs[i]][sommets_impairs[j]] = chemins[(sommets_impairs[i], sommets_impairs[j])]
            nouveau_graphe[sommets_impairs[j]][sommets_impairs[i]] = chemins[(sommets_impairs[j], sommets_impairs[i])]

    # Trouvez un cycle eulérien dans le nouveau graphe
    cycle_eulerien = trouver_cycle_eulerien(nouveau_graphe, sommets_impairs[0])

    # Ajoutez les arêtes du cycle eulérien au graphe d'origine pour obtenir un graphe eulérien
    for i in range(len(cycle_eulerien)-1):
        u = cycle_eulerien[i]
        v = cycle_eulerien[i+1]
        matrice_adj[u][v] += 1
        matrice_adj[v][u] += 1

    return matrice_adj

def t_join(matrice_adj, sommet_de_depart):
    # Initialisez la liste des arêtes
    aretes = []
    for i in range(len(matrice_adj)):
        for j in range(i+1, len(matrice_adj)):
            if matrice_adj[i][j] > 0:
                aretes.append((i, j, matrice_adj[i][j]))

    # Triez les arêtes par poids croissant
    aretes.sort(key=lambda x: x[2])

    # Initialisez la liste des ensembles disjoints
    ensembles_disjoints = [set([i]) for i in range(len(matrice_adj))]

    # Parcourez toutes les arêtes
    for u, v in aretes:
        # Vérifiez si u et v sont dans des ensembles disjoints différents
        ensemble_u = None
        ensemble_v = None
        for ensemble in ensembles_disjoints:
            if u in ensemble:
                ensemble_u = ensemble
            if v in ensemble:
                ensemble_v = ensemble
        if ensemble_u == ensemble_v:
            continue

        # Fusionnez les ensembles disjoints
        ensemble_u.update(ensemble_v)
        ensembles_disjoints.remove(ensemble_v)

        # Ajoutez l'arête au graphe
        matrice_adj[u][v] += 1
        matrice_adj[v][u] += 1

    # Trouvez un cycle eulérien dans le graphe
    cycle_eulerien = trouver_cycle_eulerien(matrice_adj, sommet_de_depart)

    return cycle_eulerien

def calculer_cout_solution(matrice_adj, cycle_eulerien):
    # Initialisez le coût de la solution
    cout = 0

    # Parcourez toutes les arêtes du cycle eulérien
    for i in range(len(cycle_eulerien)-1):
        u = cycle_eulerien[i]
        v = cycle_eulerien[i+1]

        # Ajoutez le poids de l'arête au coût de la solution
        cout += matrice_adj[u][v]

        # Décrémentez le poids de l'arête pour éviter de compter plusieurs fois
        matrice_adj[u][v] -= 1
        matrice_adj[v][u] -= 1

    # Retournez le coût de la solution
    return cout

import heapq

def trouver_chemins_impairs(matrice_adj):
    # Trouver les sommets de degré impair
    sommets_impairs = [sommet for sommet in range(len(matrice_adj)) if sum(matrice_adj[sommet]) % 2 == 1]

    # Trouver tous les plus courts chemins entre les sommets de degré impair
    chemins = []
    for i in range(len(sommets_impairs)):
        for j in range(i+1, len(sommets_impairs)):
            u = sommets_impairs[i]
            v = sommets_impairs[j]
            cout, chemin = dijkstra(matrice_adj, u, v)
            chemins.append((cout, chemin))

    # Trier les chemins par ordre croissant de coût
    chemins.sort()

    return chemins



def resoudre_postier_chinois(matrice_adj, sommet_de_depart):
    # Trouvez un cycle eulérien dans le graphe
    cycle_eulerien = trouver_cycle_eulerien(matrice_adj, sommet_de_depart)

    # Si le graphe est eulérien, renvoyez le cycle eulérien
    if cycle_eulerien is not None:
        cout = calculer_cout_solution(matrice_adj, cycle_eulerien)
        chemin = [str(x) for x in cycle_eulerien]
        print("Le graphe est eulérien. Le cycle eulérien est : " + " -> ".join(chemin) + ".")
        print("Le poids de la solution est : " + str(cout) + ".")
        return

    # Trouvez tous les plus courts chemins entre les sommets de degré impair
    chemins = trouver_chemins_impairs(matrice_adj)

    # Créez un sous-graphe qui est eulérien
    sous_graphe_eulerien = creer_sous_graphe_eulerien(matrice_adj, chemins)

    # Utilisez l'algorithme de T-join pour obtenir un graphe eulérien
    cycle_eulerien = t_join(sous_graphe_eulerien, sommet_de_depart)

    # Ajoutez les arêtes du cycle eulérien au graphe d'origine pour obtenir un graphe eulérien
    for i in range(len(cycle_eulerien)-1):
        u = cycle_eulerien[i]
        v = cycle_eulerien[i+1]
        matrice_adj[u][v] += 1
        matrice_adj[v][u] += 1

    # Calculez le coût de la solution
    cout = calculer_cout_solution(matrice_adj, cycle_eulerien)
      # Affichez le chemin du facteur ainsi que son poids
    chemin = [str(x) for x in cycle_eulerien]
    print("Le graphe n'est pas eulérien. Le chemin du facteur est : " + " -> ".join(chemin) + ".")
    print("Le poids de la solution est : " + str(cout) + ".")


    return chemin

chemin = resoudre_postier_chinois(matrice_adj, 0)


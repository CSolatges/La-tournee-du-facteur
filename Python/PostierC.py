import sys

# Taille du graphe (nombre de sommets)
N = 9

# Matrice d'adjacence du graphe
Adjacence = [
    [0, 1, 0, 0, 0, 0, 0, 0, 0],
    [1, 0, 1, 0, 0, 0, 0, 1, 0],
    [0, 1, 0, 1, 0, 0, 1, 0, 0],
    [0, 0, 1, 0, 1, 1, 0, 0, 0],
    [0, 0, 0, 1, 0, 0, 0, 0, 0],
    [0, 0, 0, 1, 0, 0, 1, 0, 0],
    [0, 0, 1, 0, 0, 1, 0, 1, 1],
    [0, 1, 0, 0, 0, 0, 1, 0, 1],
    [0, 0, 0, 0, 0, 0, 1, 1, 0]
]

# Charge maximale

# Matrice des poids du graphe
# représentant les kilomètres à parcourir par
# le postier chinois.
INFINI = sys.maxsize
Kilometres = [
    [INFINI, 1, INFINI, INFINI, INFINI, INFINI, INFINI, INFINI, INFINI],
    [1, INFINI, 1, INFINI, INFINI, INFINI, INFINI, 1, INFINI],
    [INFINI, 1, INFINI, 1, INFINI, INFINI, 1, INFINI, INFINI],
    [INFINI, INFINI, 1, INFINI, 1, 1, INFINI, INFINI, INFINI],
    [INFINI, INFINI, INFINI, 1, INFINI, INFINI, INFINI, INFINI, INFINI],
    [INFINI , INFINI , INFINI , 1 , INFINI , INFINI , 1 ,INFINI , INFINI],
    [INFINI , INFINI , 1 , INFINI , INFINI , 1 , INFINI , 1 , 1],
    [INFINI , 1 , INFINI , INFINI , INFINI , INFINI , 1 , INFINI , 1],
    [INFINI, INFINI , INFINI, INFINI , INFINI , INFINI , 1 , 1 , INFINI]]
 

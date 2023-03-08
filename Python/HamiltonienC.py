import numpy as np
N=9
Adjacence=np.zeros(N)

label = [0]*N  # étiquette si le sommet a été parcouru
chemin = [[i] for i in range(N)]  # enregistrer le sommet prochain de chaque sommet
chemin_hamiltonien = [0]*N  # enregistrer le résultat: un chemin hamiltonien

def init_chemin():  # initialisation du chemin
    for i in range(N):
        chemin[i][0] = i

def label_test():  # vérifier si tous les sommets sont parcourus
    for i in range(N):
        if label[i] == 0:
            return 0
    return 1

def chemin_construire(origine):  # une translation de chemin à cycle hamitonien
    for i in range(N):
        chemin_hamiltonien[i] = origine
        origine = chemin[origine][0]
    for i in range(N-1):
        print("%d -> " % chemin_hamiltonien[i], end='')
    print("%d" % chemin_hamiltonien[N-1])

def cycleHamilton(depart, origine):  # DFS & Back Propagation pour trouver le cycle hamiltonien
    global chemin_hamiltonien
    arrive = -1  # Si le sommet peut aller à un autre sommet
    for i in range(N):
        if Adjacence[depart][i] != 0 and label[i] == 0:  # si ce sommet est accessible
            arrive = i
            label[arrive] = 1  # étiqueter

            chemin[depart][0] = arrive  # enregistrer
            if cycleHamilton(arrive, origine) == 1:  # Si trouver un cycle hamiltonien, c'est terminé!
                return 1
    if arrive == -1 and not label_test():  # Si on a un cycle mais pas un cycle hamiltonien
        label[depart] = 0  # On enlève l'étiquette
        return 0
    if label_test() == 1 and Adjacence[depart][origine] != 0:  # C'est la condition pour trouver un cycle hamiltonien
        chemin[depart][0] = origine  # Connecter l'origine et l'arrivée
        return 1
    else:
        label[depart] = 0
        label[arrive] = 0
        return 0

def hamiltonien(origine):
    global chemin_hamiltonien
    label[origine] = 1
    cycleHamilton(origine, origine)
    chemin_construire(origine)

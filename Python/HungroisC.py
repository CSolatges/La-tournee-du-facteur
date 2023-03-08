import sys

N = 6
INFINITY = sys.maxsize

Kilometres = [[INFINITY,1,3,2,1,2],
              [1,INFINITY,3,1,2,3],
              [3,3,INFINITY,5,3,2],
              [2,1,5,INFINITY,2,3],
              [1,2,3,2,INFINITY,1],
              [2,3,2,3,1,INFINITY]]

impaire = [0] * N
sousGraph = [[0] * N for i in range(N)]

copy_couplage = [0] * N  # enregistrer le couplage
pi = 100000  # poids minimal
poids = 0  # le poids minimal de couplage maximal
Arret = [[0] * N for i in range(N)]  # enregistrer les arrets qui sont lies
etiquer = [0] * N  # Si le sommet a ete etique
copy_etique = [0] * N  # enregistrer l'etique
nouveau_couplage = [-1] * N  # Si le sommet est dans le couplage


def copyEtique():
    global copy_etique, etiquer
    copy_etique = etiquer.copy()
    return


def copytoEtique():
    global etiquer, copy_etique
    etiquer = copy_etique.copy()
    return


def Arret_lier():
    global Arret, sousGraph
    for i in range(N):
        for j in range(N):
            Arret[i][j] = -1
    for i in range(N):
        k = 0
        for j in range(N):
            if sousGraph[i][j] != INFINITY:
                Arret[i][k] = j
                k += 1
    return


def refreshEtique():
    global nouveau_couplage, etiquer
    for i in range(N):
        if nouveau_couplage[i] != -1:
            etiquer[i] = 1
        else:
            etiquer[i] = 0
    return


def init_nouvcouplage():
    global nouveau_couplage
    nouveau_couplage = [-1] * N
    return


def init_copyCouplage():
    global copy_couplage
    copy_couplage = [-1] * N
    return


def copyCouplage():
    global nouveau_couplage, copy_couplage
    copy_couplage = nouveau_couplage.copy()
    return


def copyNouveauCouplage():
    global nouveau_couplage, copy_couplage
    nouveau_couplage = copy_couplage.copy()
    return


def DFS(depart, start):
    global nouveau_couplage, etiquer, Arret
    etiquer[depart] = 1
    for i in range(start, N):
        if Arret[depart][i] != -1:
            arrive = Arret[depart][i]
            if etiquer[arrive] == 0:
                etiquer[arrive] = 1
                if nouveau_couplage[arrive] == -1 or DFS(nouveau_couplage[arrive], 0):
                    nouveau_couplage[arrive] = depart
                    nouveau_couplage[depart] = arrive
                    return 1
    return 0


def sum_couplage(): # calculate the weight of the matching
    sum = 0
    for i in range(N):
        if nouveau_couplage[i] != -1:
            sum += sousGraph[i][nouveau_couplage[i]]
    return (sum // 2)

def init_etiquer(): # initialization of the label
    for i in range(N):
        etiquer[i] = 0

def print_couplage(): # for verification
    for i in range(N):
        if nouveau_couplage[i] != -1:
            print("{", i, ",", nouveau_couplage[i], "}")

def print_etique(): # for verification
    for i in range(N):
        if etiquer[i] != 0:
            print(i, "is etique")

def couplage_poids():
    global poids
    global pi
    global nouveau_couplage
    global couplage
    global etiquer

    flag = 0

    # initialization
    Arret_lier()
    init_copyCouplage()
    init_nouvcouplage()

    # find all augmenting paths starting from each vertex
    for k in range(N):
        for i in range(N):
            if etiquer[i] == 0:
                for j in range(N):
                    init_etiquer()
                    if DFS(i, j) == 1: # if augmenting path exists
                        poids = sum_couplage()
                        if poids < pi:
                            pi = poids # update minimal weight
                            copyNouveauCouplage() # important! Matching must be saved, otherwise DFS will have a problem
                    refreshEtique()

        if pi == 10000: # if no augmenting path, terminate
            break

        for i in range(N): # search for minimal weight to update matching
            if etiquer[i] == 0:
                init_etiquer()
                for j in range(N):
                    if DFS(i, j) == 1:
                        poids = sum_couplage()
                        if poids == pi: # matching found!
                            pi = 10000
                            flag = 1
                            copyCouplage() # new matching
                            copyEtique() # new label
                            refreshEtique()
                            break
                        else:
                            copyNouveauCouplage()
                            init_etiquer()
                if flag != 0:
                    flag = 0
                    break

    return poids

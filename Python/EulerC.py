import numpy as np
from PostierC import Adjacence
N=len(Adjacence)


class Stack:
    def __init__(self):
        self.top = 0
        self.sommet = [0] * N

s = Stack()

Chemin = [0] * (N*N)
longueur = 0

def dfs(depart):
    s.top += 1
    s.sommet[s.top] = depart # push
    for i in range(N):
        if Adjacence[i][depart]:
            Adjacence[i][depart] -= 1
            Adjacence[depart][i] -= 1 # Car on a doublé le chemin
            dfs(i)
            break
    return

def Euler(depart):
    # push le premier sommet dans s
    s = {"top": 0, "sommet": [depart]}
    print("[ ", end="")
    # Si s n'est pas vide
    while s["top"] >= 0:
        flag = 0
        for i in range(N):
            if Adjacence[i][s["sommet"][s["top"]]]: # S'il y a encore d'autres arrêts partant depuis ce sommet
                flag = 1
                break
        if not flag: # Si tous les arrêts sont parcourus
            print(f"{s['sommet'][s['top']]} ", end="")
            Chemin.append(s['sommet'][s['top']])
            s["top"] -= 1 # pop
        else:
            dfs(s["sommet"][s["top"]])
            s["top"] -= 1
    print("]\n")

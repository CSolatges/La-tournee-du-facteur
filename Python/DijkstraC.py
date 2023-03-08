import sys
import numpy as np
from PostierC import Kilometres, Adjacence 

N = len(Kilometres)

def dijkstra(depart, arrive):
    pi = np.zeros(N)
    TN = np.zeros(N)
    pre = np.zeros(N)
    TN[depart] = 1
    
    for i in range(N):
        pi[i] = Kilometres[depart][i]
    
    fois = 0
    m = 1000
    n = 1000
    
    for i in range(N):
        if m > pi[i] and TN[i] != 1:
            m = pi[i]
            n = i
    
    pre[n] = depart
    
    while fois < N:
        Pi = 1000
        pt = 1000
        
        for i in range(N):
            if Pi > pi[i] and TN[i] != 1:
                Pi = pi[i]
                pt = i
        
        if pt == 1000:
            continue
        else:
            TN[pt] = 1
        
        for i in range(N):
            if TN[i] != 1:
                if pi[i] > Pi + Kilometres[pt][i]:
                    pi[i] = Pi + Kilometres[pt][i]
                    pre[i] = pt
        
        fois += 1
    
    for i in range(N):
        if pre[i] == 0:
            pre[i] = depart
    
    return int(pi[arrive])


def doubler(depart, arrive):
    pi = np.zeros(N)
    TN = np.zeros(N)
    pre = np.zeros(N)
    TN[depart] = 1
    
    for i in range(N):
        pi[i] = Kilometres[depart][i]
    
    fois = 0
    m = 1000
    n = 1000
    
    for i in range(N):
        if m > pi[i] and TN[i] != 1:
            m = pi[i]
            n = i
    
    pre[n] = depart
    
    while fois < N:
        Pi = 1000
        pt = 1000
        
        for i in range(N):
            if Pi > pi[i] and TN[i] != 1:
                Pi = pi[i]
                pt = i
        
        if pt == 1000:
            continue
        else:
            TN[pt] = 1
        
        for i in range(N):
            if TN[i] != 1:
                if pi[i] > Pi + Kilometres[pt][i]:
                    pi[i] = Pi + Kilometres[pt][i]
                    pre[i] = pt
        
        fois += 1
    
    for i in range(N):
        if pre[i] == 0:
            pre[i] = depart
    
    while arrive != depart:
        Adjacence[int(pre[arrive])][int(arrive)] += 1
        Adjacence[int(arrive)][int(pre[arrive])] += 1
        arrive = pre[arrive]
    
    return

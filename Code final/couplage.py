import cplex

def resolutioncplex(graph):
    #création du problème à résoudre
    Probleme=cplex.Cplex()
    #définition des variables
    n=len(graph)
    for u in range(n):
        for v in range(n):
            if graph[u][v]!=0: #on vérifie qu'il y a un arc
                Probleme.variables.add(names=[f'x_{u}_{v}'], types=[Probleme.variables.type.binary]) 

    #définition de la fonction objectif
    Probleme.objective.set_sense(Probleme.objective.sense.minimize)
    Probleme.objective.set_linear([(f'x_{u}_{v}'*graph[u][v], 1.0) for u in range(n) for v in range(n) if graph[u][v] !=0])

   # Définition des contraintes pour les nœuds de U
    for u in range(n):
        for v in range(n):
            if graph[u][v] != 0:
                contrainte = cplex.SparsePair(ind=[f'x_{u}_{v}'], val=[1.0])
                Probleme.linear_constraints.add(lin_expr=[contrainte], senses=['L'], rhs=[1.0])
            


    # Définition des contraintes pour les nœuds de V
    for v in range(n):
        for u in range(n):
            if graph[u][v] != 0:
                contrainte = cplex.SparsePair(ind=[f'x_{u}_{v}'], val=[1.0])
                Probleme.linear_constraints.add(lin_expr=[contrainte], senses=['L'], rhs=[1.0])
          




    #résolution du problème
    Probleme.solve()
    solution = Probleme.solution.get_values()
    return solution



graph = [
    [0, 3, 1, 4, 3, 5],
    [3, 0, 4, 1, 6, 2],
    [1, 4, 0, 5, 2, 6],
    [4, 1, 5, 0, 5, 1],
    [3, 6, 2, 5, 0, 4],
    [5, 2, 6, 1, 4, 0],
]

"""

graph = [[1, 1, 1, 1, 1, 1],
         [1, 1, 1, 1, 1, 1],
         [1, 1, 1, 1, 1, 1],
         [1, 1, 1, 1, 1, 1],
         [1, 1, 1, 1, 1, 1],
         [1, 1, 1, 1, 1, 1]]



graph = [[0,1,0,0,0], 
          [1,0,0,0,1],
          [0,0,0,1,1], #ok
          [0,0,1,0,1],
          [0,1,1,1,0]]


graph =     [[0,1,0,0,0,0,0,0,0],   #graphinsa impossible de transformer en eulérien ça marche po
                [1,0,1,0,0,0,0,1,0],
                [0,1,0,1,0,0,1,0,0],
                [0,0,1,0,1,1,0,0,0],
                [0,0,0,1,0,0,0,0,0],
                [0,0,0,1,0,0,1,0,0],
                [0,0,1,0,0,1,0,1,1],
                [0,1,0,0,0,0,1,0,1],
                [0,0,0,0,0,0,1,1,0]]

graph = [[0, 1, 0, 0, 0],
         [1, 0, 1, 0, 0],
         [0, 1, 0, 1, 0],
         [0, 0, 1, 0, 1],
         [0, 0, 0, 1, 0]]
"""
print(resolutioncplex(graph))

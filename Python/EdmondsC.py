import mystack #Ca faut trouver ce que c'est

N = 6

# Definie une pile pour enregistrer le couplage
Couplage = mystack.Stack()

# Definie des etiquettes pour tous les sommets
class etiquette:
    def __init__(self):
        self.origine = 0
        self.paire = 0
        self.pre = 0

etique = [etiquette() for i in range(N)]

# marque de couplage
couplage = [[0] for i in range(N)]

# matrice d'adjacence
Adjacence = [
    [0, 1, 1, 0, 0, 1],
    [1, 0, 0, 1, 1, 1],
    [1, 0, 0, 1, 0, 0],
    [0, 1, 1, 0, 1, 0],
    [0, 1, 0, 1, 0, 0],
    [1, 1, 0, 0, 0, 0]
]

marque_arret = [[0 for i in range(N)] for j in range(N)]

def init_etique():
    for i in range(N):
        etique[i].origine = 0
        etique[i].paire = 0
        etique[i].pre = 0

def init_marque():
    for i in range(N):
        for j in range(N):
            marque_arret[i][j] = 0

def dans_Couplage(point):
    if couplage[point][0] != 0:
        return 1
    for i in range(N):
        if couplage[i][0] == point + 1:
            return 1
    return 0

def renouveler_Couplage(s):
    while not Couplage.is_empty():
        x, y = Couplage.pop()
        couplage[x-1][0] = 0
        couplage[y-1][0] = 0
    n = N
    while n > 0:
        for i in range(N):
            if etique[i].pre == s + 1 and etique[i].paire == 1:
                Couplage.push((i+1, s+1))
                couplage[i][0] = s + 1
                couplage[s][0] = i + 1
                print("{%d,%d}" % (i+1, s+1))
                s = i
                break
            elif etique[i].pre == s + 1 and etique[i].paire == 0:
                s = i
                break
        n -= 1

def etape2(x, s):
    for y in range(N):
        if Adjacence[x][y] == 1 and marque_arret[x][y] == 0 and etique[x].origine == s + 1 and etique[x].paire == 0:
            if etique[y].origine == 0 and not dans_Couplage(y):
                couplage[x][0] = y + 1
                etique[y].origine = s + 1
                etique[y].paire = 1
                etique[y].pre = x + 1
                print("couplage de", s + 1, "a", y + 1)
                return
            elif etique[y].origine == 0 and dans_Couplage(y):
                couplage[x][0] = y + 1
                z = couplage[y][0] - 1
                etique[y].origine = s + 1
                etique[y].paire = 1
                etique[y].pre = x + 1
                etique[z].origine = s + 1
                etique[z].paire = 0
                etique[z].pre = y + 1
                for i in range(N):
                    marque_arret[i][y] = 1
                    marque_arret[y][i] = 1
                etape2(z, s)
                return
            elif etique[y].origine != 0 and etique[y].paire == 0 and etique[y].origine != s + 1:
                u = etique[y].origine
                print("couplage de", s + 1, "a", u + 1)
                return
    return

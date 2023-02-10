"On va tester les différentes fonction implémentées sur notre graphe de Magellan"
#Import des modules contenant nos fonctions à tester
import numpy as np
import Dijkstra as D
import Euler as E
import Hierholzer as H

#Matrice d'adjacence de notre graphe
GINSA= np.array([[0,1,0,0,0,0,0,0,0],[1,0,1,0,0,0,0,1,0], [0,1,0,1,0,0,1,0,0], [0,0,1,0,1,1,0,0,0],[0,0,0,1,0,0,0,0,0],[0,0,0,1,0,0,1,0,0],[0,0,1,0,0,1,0,1,1],[0,1,0,0,0,0,1,0,1],[0,0,0,0,0,0,1,1,0]])
#1 quand il y a une arrête et 0 sinon

GINSAdist= np.array([[np.inf,1,np.inf,np.inf,np.inf,np.inf,np.inf,np.inf,np.inf],[1,np.inf,1,np.inf,np.inf,np.inf,np.inf,1,np.inf], [np.inf,1,np.inf,1,np.inf,np.inf,1,np.inf,np.inf], [np.inf,np.inf,1,np.inf,1,1,np.inf,np.inf,np.inf],[np.inf,np.inf,np.inf,1,np.inf,np.inf,np.inf,np.inf,np.inf],[np.inf,np.inf,np.inf,1,np.inf,np.inf,1,np.inf,np.inf],[np.inf,np.inf,1,np.inf,np.inf,1,np.inf,1,1],[np.inf,1,np.inf,np.inf,np.inf,np.inf,1,np.inf,1],[np.inf,np.inf,np.inf,np.inf,np.inf,np.inf,1,1,np.inf]])
#1 (ou poind de l'arrête) quand il y a une arrête et +inf sinon

"Module Dijkstra"

#Test de la fonction dijkstreDist
print(D.dijkstraDist(GINSAdist, 0)) 
# affiche une liste de liste avec [[distance minimal, sommet visité, sommet précédent]]

#Test de la fonction dijkstraPCC
print(D.dijkstraPCC(GINSAdist,0,8))
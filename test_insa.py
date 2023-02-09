"On va tester les différentes fonction implémentées sur notre graphe de Magellan"
#Import des modules contenant nos fonctions à tester
import numpy as np
import Dijkstra as D
import Euler as E
import Hierholzer as H

#Matrice d'adjacence de notre graphe
GINSA= np.array([[np.inf,1,np.inf,np.inf,np.inf,np.inf,np.inf,np.inf,np.inf],[1,np.inf,1,np.inf,np.inf,np.inf,np.inf,1,np.inf], [np.inf,1,np.inf,1,np.inf,np.inf,1,np.inf,np.inf], [np.inf,np.inf,1,np.inf,1,1,np.inf,np.inf,np.inf],[np.inf,np.inf,np.inf,1,np.inf,np.inf,np.inf,np.inf,np.inf],[np.inf,np.inf,np.inf,1,np.inf,np.inf,1,np.inf,np.inf],[np.inf,np.inf,1,np.inf,np.inf,1,np.inf,1,1],[np.inf,1,np.inf,np.inf,np.inf,np.inf,1,np.inf,1],[np.inf,np.inf,np.inf,np.inf,np.inf,np.inf,1,1,np.inf]])
#1 quand il y a une arrête et +inf sinon

"Module Dijkstra"

#Test de la fonction dijkstreDist
print(D.dijkstraDist(GINSA, 0)) 
# affiche une liste de liste avec [[distance minimal, sommet visité, sommet précédent]]

#Test de la fonction dijkstraPCC
print(D.dijkstraPCC(GINSA,0,8))
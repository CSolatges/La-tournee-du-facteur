## L'objectif de ce code est de déterminer les cycles Eulérien d'un graphe par l'algorithme de Hierholzer

#1 Choisir n'importe quel sommet initial v
#2 Suivre un chemin arbitraire d'arêtes jusqu'à retourner à v en traversant chaque arête une fois au plus, obtenant un cycle partiel c
#3 Tant qu'il y a des sommets u dans le cycle c avec des arêtes qu'on a pas encore choisi faire:
#3.1 Suivre un chemin à partir de u jusqu'à retourner à u. on obtient un cycle c'
#3.2 prolonger c par c'




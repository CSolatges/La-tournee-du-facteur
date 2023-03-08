#include <limits.h>
#define INFINI INT_MAX

// Taille du graphe (nombre de sommets)
#define N 9

// Matrice d'adjacence du graphe
unsigned int Adjacence[N][N] = {
    {0 , 1 , 0 , 0 , 0 , 0 , 0 , 0 , 0},
    {1 , 0 , 1 , 0 , 0 , 0 , 0 , 1 , 0},
    {0 , 1 , 0 , 1 , 0 , 0 , 1 , 0 , 0},
    {0 , 0 , 1 , 0 , 1 , 1 , 0 , 0 , 0},
    {0 , 0 , 0 , 1 , 0 , 0 , 0 , 0 , 0},
    {0 , 0 , 0 , 1 , 0 , 0 , 1 , 0 , 0},
    {0 , 0 , 1 , 0 , 0 , 1 , 0 , 1 , 1},
    {0 , 1 , 0 , 0 , 0 , 0 , 1 , 0 , 1},
    {0 , 0 , 0 , 0 , 0 , 0 , 1 , 1 , 0}};

// Matrice des poids du graphe
// représentant les kilomètres à parcourir par
// le postier chinois.
unsigned int Kilometres[N][N] = {
    {INFINI , 1 , INFINI , INFINI , INFINI , INFINI , INFINI , INFINI , INFINI},
    {1 , INFINI , 1 , INFINI , INFINI , INFINI , INFINI , 1 , INFINI},
    {INFINI , 1 , INFINI , 1 , INFINI ,INFINI , 1 , INFINI , INFINI},
    {INFINI, INFINI , 1 , INFINI , 1 , 1 , INFINI , INFINI , INFINI},
    {INFINI , INFINI , INFINI , 1 , INFINI , INFINI , INFINI , INFINI , INFINI},
    {INFINI , INFINI , INFINI , 1 , INFINI , INFINI , 1 ,INFINI , INFINI},
    {INFINI , INFINI , 1 , INFINI , INFINI , 1 , INFINI , 1 , 1},
    {INFINI , 1 , INFINI , INFINI , INFINI , INFINI , 1 , INFINI , 1},
    {INFINI, INFINI , INFINI, INFINI , INFINI , INFINI , 1 , 1 , INFINI}};
 
// Charge maximale


#include <limits.h>
#define INFINI INT_MAX

// Taille du graphe (nombre de sommets)
#define N 8

// Matrice d'adjacence du graphe
unsigned int Adjacence[N][N] = {
    {0 , 1 , 1 , 1 , 1 , 1 , 1 , 1 },
    {1 , 0 , 1 , 1 , 1 , 1 , 1 , 1 },
    {1 , 1 , 0 , 1 , 1 , 1 , 1 , 1 },
    {1 , 1 , 1 , 0 , 1 , 1 , 1 , 1 },
    {1 , 1 , 1 , 1 , 0 , 1 , 1 , 1 },
    {1 , 1 , 1 , 1 , 1 , 0 , 1 , 1 },
    {1 , 1 , 1 , 1 , 1 , 1 , 0 , 1 },
    {1 , 1 , 1 , 1 , 1 , 1 , 1 , 0 }};

// Matrice des poids du graphe
// représentant les kilomètres à parcourir par
// le postier chinois.
unsigned int Kilometres[N][N] = {
    {INFINI , 4 , 7 , 7 , 7 , 10, 4 , 10},
    {4 , INFINI , 10, 7 , 8 , 5 , 10, 1 },
    {7 , 10, INFINI , 10, 7 , 1 , 6 , 6 },
    {7 , 7 , 10, INFINI , 4 , 9 , 6 , 10},
    {7 , 8 , 7 , 4 , INFINI , 2 , 5 , 6 },
    {10, 5 , 1 , 9 , 2 , INFINI , 5 , 3 },
    {4 , 10, 6 , 6 , 5 , 5 , INFINI , 3 },
    {10, 1 , 6 , 10, 6 , 3 , 3 , INFINI }};

// Charge maximale


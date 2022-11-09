#include <pch.h>

#include "P2_Heuristics.h"

// Euclidean heuristic method
float euclidean(const int xSelf, const int ySelf, const GridPos goal)
{
    auto xDiff = (goal.col > xSelf) ? goal.col - xSelf : xSelf - goal.col;
    auto yDiff = (goal.row > ySelf) ? goal.row - ySelf : ySelf - goal.row;

    return (float)sqrt(xDiff * xDiff + yDiff * yDiff);
}

// Octile heuristic method
float octile(const int xSelf, const int ySelf, const GridPos goal)
{
    auto xDiff = (goal.col > xSelf) ? goal.col - xSelf : xSelf - goal.col;
    auto yDiff = (goal.row > ySelf) ? goal.row - ySelf : ySelf - goal.row;

    return float(min(xDiff, yDiff) * sqrt(2) + max(xDiff, yDiff) - min(xDiff, yDiff));
}

// Chebyshev heuristic method
float chebyshev(const int xSelf, const int ySelf, const GridPos goal)
{
    auto xDiff = (goal.col > xSelf) ? goal.col - xSelf : xSelf - goal.col;
    auto yDiff = (goal.row > ySelf) ? goal.row - ySelf : ySelf - goal.row;

    return xDiff > yDiff ? (float)xDiff : (float)yDiff;
}

// Manhattan heuristic method
float manhattan(const int xSelf, const int ySelf, const GridPos goal)
{
    auto xDiff = (goal.col > xSelf) ? goal.col - xSelf : xSelf - goal.col;
    auto yDiff = (goal.row > ySelf) ? goal.row - ySelf : ySelf - goal.row;

    return (float)(xDiff + yDiff);
}

#pragma once
#include "Misc/PathfindingDetails.hpp"

using namespace std;

float euclidean(int xSelf, int ySelf, GridPos goal);
float octile(int xSelf, int ySelf, GridPos goal);
float chebyshev(int xSelf, int ySelf, GridPos goal);
float manhattan(int xSelf, int ySelf, GridPos goal);

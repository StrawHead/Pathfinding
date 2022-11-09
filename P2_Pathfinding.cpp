#include <pch.h>
#include "Projects/ProjectTwo.h"
#include "P2_Heuristics.h"
#include "P2_Pathfinding.h"

#include <algorithm>

using namespace std;

#pragma region Extra Credit
bool ProjectTwo::implemented_floyd_warshall()
{
    return false;
}

bool ProjectTwo::implemented_goal_bounding()
{
    return false;
}

bool ProjectTwo::implemented_jps_plus()
{
    return false;
}
#pragma endregion

bool AStarPather::Node::operator==(const AStarPather::Node& rhs) const
{
    return true;
}

bool AStarPather::initialize()
{
    return true; // return false if any errors actually occur, to stop engine initialization
}

void AStarPather::shutdown()
{
}

PathResult AStarPather::compute_path(PathRequest& request)
{
    /*
        This is where you handle pathing requests, each request has several fields:

        start/goal - start and goal world positions
        path - where you will build the path upon completion, path should be
            start to goal, not goal to start
        heuristic - which heuristic calculation to use
        weight - the heuristic weight to be applied
        newRequest - whether this is the first request for this path, should generally
            be true, unless single step is on

        smoothing - whether to apply smoothing to the path
        rubberBanding - whether to apply rubber banding
        singleStep - whether to perform only a single A* step
        debugColoring - whether to color the grid based on the A* state:
            closed list nodes - yellow
            open list nodes - blue

            use terrain->set_color(row, col, Colors::YourColor);
            also it can be helpful to temporarily use other colors for specific states
            when you are testing your algorithms

        method - which algorithm to use: A*, Floyd-Warshall, JPS+, or goal bounding,
            will be A* generally, unless you implement extra credit features

        The return values are:
            PROCESSING - a path hasn't been found yet, should only be returned in
                single step mode until a path is found
            COMPLETE - a path to the goal was found and has been built in request.path
            IMPOSSIBLE - a path from start to goal does not exist, do not add start position to path
    */
    Node* cheapestNode;

    if (request.newRequest)
    {
		//Get the request start and goal GridPos
		startGridPos = terrain->get_grid_position(request.start);
		goalGridPos = terrain->get_grid_position(request.goal);

		//Get the request start and goal WorldPos
		startPos = terrain->get_world_position(startGridPos);
		goalPos = terrain->get_world_position(goalGridPos);

		//Get the request heuristic
		heuristic = request.settings.heuristic;

		//Get the request weight
		weight = request.settings.weight;

		bool validStart = terrain->is_valid_grid_position(startGridPos);
		bool validGoal = terrain->is_valid_grid_position(goalGridPos);

		if (validStart)
		{
			if (validGoal)
			{
				Node* startNode = &arr2D[terrain->get_grid_position(request.start).row][terrain->get_grid_position(request.start).col];

				// Push Start Node onto the Open List
				openList.clear();
				resetGrid();
				setStartNode();
				pushToOpenList(startNode);
				terrain->set_color(goalGridPos, Colors::Red);
				cheapestNode = startNode;
			}
		}
    }

    // While Open List is not empty
    while (!openList.empty())
    {
		if (request.settings.debugColoring == false)
		{
			for (int i = 0; i < ROW; ++i)
			{
				for (int j = 0; j < COL; ++j)
				{
					if (terrain->is_valid_grid_position(i, j))
					{
						terrain->set_color(i, j, Colors::White);
					}
				}
			}
		}

        // Pop cheapest node off Open List(parent node).
        cheapestNode = findCheapest();

		openList.remove(cheapestNode);

        // If node is the Goal Node, then path found
        if (isGoalNode(cheapestNode))
        {
			WaypointList newPath;

            createPath(cheapestNode, request);

			if (request.settings.rubberBanding && request.settings.smoothing)
			{
				//do rubber banding first
				rubberband(request.path);

				//TODO: Fix the rubberband + smoothing

				////get the first two points on the path
				//WaypointList::iterator p1 = request.path.begin();
				//WaypointList::iterator p2 = next(p1, 1);

				////turn the points into gridPos
				//GridPos gp1;
				//GridPos gp2;

				//int newX;
				//int newY;

				//Vec3 newPoint;

				//while(p2 != request.path.end())
				//{
				//	gp1 = terrain->get_grid_position(*p1);
				//	gp2 = terrain->get_grid_position(*p2);

				//	//if the points are too far apart, add some back
				//	if (gp2.row - gp1.row > 1.5f || gp2.col - gp1.col > 1.5f)
				//	{
				//		newX = (gp2.col - gp1.col) / 2;
				//		newY = (gp2.row - gp1.row) / 2;

				//		newPoint = terrain->get_world_position(newY, newX);
				//		request.path.insert(next(p1, 1), newPoint);

				//		p2 = next(p1, 1);
				//	}
				//	else
				//	{
				//		p1++;
				//		p2++;
				//	}
				//}

				newPath = smoothing(request.path);
				request.path.clear();
				request.path = newPath;
			}
			else if (request.settings.rubberBanding)
			{
				rubberband(request.path);
			}
			else if (request.settings.smoothing)
			{
				smoothing(request.path);
				newPath = smoothing(request.path);
				request.path.clear();
				request.path = newPath;
			}

            return PathResult::COMPLETE;
        }

        // For all neighboring child nodes
        findNeighbors(cheapestNode, request);

        // Place parent node on the Closed List
		pushToClosedList(cheapestNode);

        // If taken too much time this frame (or in single step mode) abort search for now and resume next frame 
        if (request.settings.singleStep)
        {
            return PathResult::PROCESSING;
        }
    }

    // Open List empty, thus no path possible (RETURN “fail”)
    return PathResult::IMPOSSIBLE;
}

void AStarPather::resetGrid()
{
    // Baseline
    for (int i = 0; i < ROW; ++i)
    {
        for (int j = 0; j < COL; ++j)
        {
            if(terrain->is_valid_grid_position(i,j))
            {
                arr2D[i][j].xParent = INT_MIN;
                arr2D[i][j].yParent = INT_MIN;
                arr2D[i][j].xSelf = j;
                arr2D[i][j].ySelf = i;
                arr2D[i][j].cost = 0.0f;
                arr2D[i][j].given = 0.0f;
                arr2D[i][j].startNode = false;
                arr2D[i][j].nodeList = onList::NONE;
                terrain->set_color(i, j, Colors::White);
            }
        }
    }
}

void AStarPather::setStartNode()
{
	Node* node = &arr2D[startGridPos.row][startGridPos.col];

	node->given = 0.0f;
	node->startNode = true;
	calcTotalCost(node, node->given);
}

void AStarPather::pushToOpenList(Node* node)
{
	Vec3 nodeWorldPos = terrain->get_world_position(node->ySelf, node->xSelf);
	GridPos nodeGridPos = terrain->get_grid_position(nodeWorldPos);

	node->nodeList = onList::OPEN;
	terrain->set_color(nodeGridPos, Colors::Blue);
	openList.push_back(node);
}

void AStarPather::pushToClosedList(Node* node)
{
	node->nodeList = onList::CLOSED;
	terrain->set_color(node->ySelf, node->xSelf, Colors::Yellow);
		
}

void AStarPather::rubberband(WaypointList& path)
{
	//can't reduce any further
	if(path.size() < 3)
	{
		return;
	}

	//get our first set of points
	WaypointList::iterator p1 = path.begin();
	WaypointList::iterator p2 = next(p1, 1);
	WaypointList::iterator p3 = next(p2, 1);
	
	GridPos gp1;
	GridPos gp2;
	GridPos gp3;

	//flags
	bool done = false;
	bool foundWall = false;

	//bounding box parameters
	int minRow, maxRow, minCol, maxCol;

	while (!done)
	{
		gp1 = terrain->get_grid_position(*p1);
		gp2 = terrain->get_grid_position(*p2);
		gp3 = terrain->get_grid_position(*p3);

		minRow = min(min(gp1.row, gp2.row), gp3.row);
		maxRow = max(max(gp1.row, gp2.row), gp3.row);
		minCol = min(min(gp1.col, gp2.col), gp3.col);
		maxCol = max(max(gp1.col, gp2.col), gp3.col);

		foundWall = false;

		for (int i = minRow; i <= maxRow; ++i)
		{
			if (!foundWall)
			{
				for (int j = minCol; j <= maxCol; ++j)
				{
					if(!foundWall)
					{
						if (terrain->is_wall(i, j))
						{
							foundWall = true;
						}
					}
				}
			}
		}

		//no walls
		if (!foundWall)
		{
			path.remove(*p2);
			p2 = p3;
			p3++;
		}
		else//found a wall
		{
			p1 = p2;
			p2 = p3;
			p3 = next(p3, 1);
		}

		//end of the path
		if (p3 == path.end())
		{
			done = true;
		}
	}
}

WaypointList AStarPather::smoothing(WaypointList& path)
{
	//can't smooth with less than 3 way points
	if (path.size() < 3)
	{
		return path;
	}

	//get our first set of points
	WaypointList::iterator p1 = path.begin();
	WaypointList::iterator p2 = p1;
	WaypointList::iterator p3 = next(p2, 1);
	WaypointList::iterator p4 = next(p3, 1);

	WaypointList newList;

	//CatmullRom spline output
	Vec3 pOut;

	while(p4 != path.end())
	{
		float t = 0.0f;

		for (int i = 0; i < 4; ++i)
		{
			//get the new waypoint
			pOut = p1->CatmullRom(*p1, *p2, *p3, *p4, t);

			//insert the new waypoint
			newList.push_back(pOut);

			//increment t and go again
			t += 0.25f;
		}

		if (p1 == p2)
		{
			p2++;
			p3++;
			p4++;
		}
		else if(next(p4, 1) != path.end())
		{
			p1++;
			p2++;
			p3++;
			p4++;
		}
		else
		{
			p1++;
			p2++;
			p3++;
		}

		if (p3 == path.end())
			break;
	}
	
	return newList;
}

void AStarPather::evaluateNeighbors(pair<int, int> p1, pair<int, int> p2)
{
	Node* child = &arr2D[p1.second][p1.first];

	Node* parent = &arr2D[p2.second][p2.first];

	float tempGiven = calcGivenCost(child, parent);

	float tempTotal = calcTotalCost(child, tempGiven);

	if (child->nodeList == onList::NONE)
	{
		// Set the parent
		child->xParent = p2.first;
		child->yParent = p2.second;

		child->given = tempGiven;
		child->cost = tempTotal;

		pushToOpenList(child);
	}
	else if(tempTotal < child->cost)
	{
		// Set the parent
		child->xParent = p2.first;
		child->yParent = p2.second;

		child->given = tempGiven;
		child->cost = tempTotal;

		pushToOpenList(child);
	}
}

void AStarPather::createPath(Node* cheapestNode, PathRequest& request)
{
    int startRow = startGridPos.row;
    int startCol = startGridPos.col;

    while (cheapestNode->ySelf != startRow || cheapestNode->xSelf != startCol)
    {
		Vec3 worldPos = terrain->get_world_position(cheapestNode->ySelf, cheapestNode->xSelf);
		Node* cheapestNodeParent = &arr2D[cheapestNode->yParent][cheapestNode->xParent];

        request.path.push_back(worldPos);

        cheapestNode = cheapestNodeParent;
    }

    // Push start first
    request.path.push_back(startPos);

    std::reverse(request.path.begin(), request.path.end());
}

void AStarPather::findNeighbors(Node* node, PathRequest& request)
{
	pair<int, int> parent = { node->xSelf, node->ySelf };

	pair<int, int> topChild = { node->xSelf, node->ySelf + 1 };
	pair<int, int> rightChild = { node->xSelf + 1, node->ySelf };
	pair<int, int> bottomChild = { node->xSelf, node->ySelf - 1 };
	pair<int, int> LeftChild = { node->xSelf - 1, node->ySelf };
	pair<int, int> topRightChild = { node->xSelf + 1, node->ySelf + 1 };
	pair<int, int> bottomRightChild = { node->xSelf + 1, node->ySelf - 1 };
	pair<int, int> bottomLeftChild = { node->xSelf - 1, node->ySelf - 1 };
	pair<int, int> topLeftChild = { node->xSelf - 1, node->ySelf + 1 };

	float temp = 0.0f;

	// Is the TOP neighbor accessible? 
	if (accessible(topChild))
	{
		// If child node is not on Open or Closed list, put it on Open List
		evaluateNeighbors(topChild, parent);
	}

	// Is the RIGHT neighbor accessible? 
	if (accessible(rightChild))
	{
		// If child node is not on Open or Closed list, put it on Open List
		evaluateNeighbors(rightChild, parent);
	}

	// Is the BOTTOM neighbor accessible? 
	if (accessible(bottomChild))
	{
		// If child node is not on Open or Closed list, put it on Open List
		evaluateNeighbors(bottomChild, parent);
	}

	// Is the LEFT neighbor accessible? 
	if (accessible(LeftChild))
	{
		// If child node is not on Open or Closed list, put it on Open List
		evaluateNeighbors(LeftChild, parent);
	}

	// Is the TOP RIGHT neighbor accessible? 
	if (accessible(topRightChild))
	{
		// Is the TOP neighbor accessible? 
		if (accessible(topChild))
		{
			// Is the RIGHT neighbor accessible? 
			if (accessible(rightChild))
			{
				// If child node is not on Open or Closed list, put it on Open List
				evaluateNeighbors(topRightChild, parent);
			}
		}
	}

	// Is the BOTTOM RIGHT neighbor accessible?  
	if (accessible(bottomRightChild))
	{
		// Is the RIGHT neighbor accessible?
		if (accessible(rightChild))
		{
			// Is the BOTTOM neighbor accessible?
			if (accessible(bottomChild))
			{
				// If child node is not on Open or Closed list, put it on Open List
				evaluateNeighbors(bottomRightChild, parent);
			}
		}
	}

	// Is the BOTTOM LEFT neighbor accessible?  
	if (accessible(bottomLeftChild))
	{
		// Is the BOTTOM neighbor accessible?  
		if (accessible(bottomChild))
		{
			// Is the LEFT neighbor accessible?  
			if (accessible(LeftChild))
			{
				// If child node is not on Open or Closed list, put it on Open List
				evaluateNeighbors(bottomLeftChild, parent);
			}
		}
	}

	// Is the TOP LEFT neighbor on the grid?  
	if (accessible(topLeftChild))
	{
		// Is the TOP neighbor on the grid?  
		if (accessible(topChild))
		{
			// Is the LEFT neighbor on the grid?  
			if (accessible(LeftChild))
			{
				// If child node is not on Open or Closed list, put it on Open List
				evaluateNeighbors(topLeftChild, parent);
			}
		}
	}
}

// Find the cheapest node on the open list
AStarPather::Node* AStarPather::findCheapest()
{
	Node* tempNode = openList.front(); 

    // Iterate through the remainder of the open list
    for (list<Node*>::iterator it = openList.begin(); it != openList.end(); ++it)
    {
        if((*it)->nodeList == onList::OPEN && ((*it)->cost < tempNode->cost))
		{
			tempNode = (*it);
		}
    }

    return tempNode;
}

float AStarPather::calcGivenCost(Node* child, Node* parent)
{
	bool TOPLEFT = (parent->xSelf == child->xSelf - 1 && parent->ySelf == child->ySelf + 1);
	bool TOPRIGHT = (parent->xSelf == child->xSelf + 1 && parent->ySelf == child->ySelf + 1);
	bool BOTTOMLEFT = (parent->xSelf == child->xSelf - 1 && parent->ySelf == child->ySelf - 1);
	bool BOTTOMRIGHT = (parent->xSelf == child->xSelf + 1 && parent->ySelf == child->ySelf - 1);

	//startNode is handled separately
	if (child->startNode)
	{
		return 0.0f;
	}
	else if (TOPLEFT || TOPRIGHT || BOTTOMLEFT || BOTTOMRIGHT)
	{
		return parent->given + 1.41f;
	}
	else // anything else
	{
		return parent->given + 1;
	}
}

float AStarPather::calcTotalCost(Node* child, float given)
{
	float temp = 0.0f;

	if (heuristic == Heuristic::EUCLIDEAN)
	{
		temp = given + (euclidean(child->xSelf, child->ySelf, goalGridPos) * weight);
	}
	else if (heuristic == Heuristic::OCTILE)
	{
		temp = given + (octile(child->xSelf, child->ySelf, goalGridPos) * weight);
	}
	else if (heuristic == Heuristic::CHEBYSHEV)
	{
		temp = given + (chebyshev(child->xSelf, child->ySelf, goalGridPos) * weight);
	}
	else
	{
		temp = given + (manhattan(child->xSelf, child->ySelf, goalGridPos) * weight);
	}
		
	return temp;
}

bool AStarPather::isGoalNode(Node* node)
{
	Vec3 nodeWP = terrain->get_world_position(node->ySelf, node->xSelf);

    GridPos nodeGP = terrain->get_grid_position(nodeWP);
    
    return (nodeGP == goalGridPos);
}

bool AStarPather::accessible(pair<int, int> p)
{
	if (terrain->is_valid_grid_position(p.second, p.first))
	{
		if (!terrain->is_wall(p.second, p.first))
		{
			return true;
		}
	}
	return false;
}

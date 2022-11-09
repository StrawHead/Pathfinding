#pragma once
#include "Misc/PathfindingDetails.hpp"
#include <vector>


using namespace std;

#define ROW 40
#define COL 40

class AStarPather
{
    enum class onList
    {
        NONE,
        OPEN,
        CLOSED
    }; // On open/closed list?

    // A structure to hold the necessary parameters
    struct Node
    {
        int xParent, yParent;   // Parent
        int xSelf, ySelf;

        float cost = 0.0f;      // Total 
        float given = 0.0f;     // Given cost

        bool startNode = false;

        onList nodeList = onList::NONE;

        bool operator==(const Node& rhs) const;
    };

public:

    /* ************************************************** */
    // DO NOT MODIFY THESE SIGNATURES
    bool initialize();
    void shutdown();
    PathResult compute_path(PathRequest& request);
    /* ************************************************** */

    void resetGrid();
    void setStartNode();
    void pushToOpenList(Node* node);
    void pushToClosedList(Node* node);
    void rubberband(WaypointList &path);
    WaypointList smoothing(WaypointList& path);
    void evaluateNeighbors(pair<int, int> p1, pair<int, int> p2);
    void createPath(Node* cheapestNode, PathRequest& request);
    void findNeighbors(Node* node, PathRequest& request);

    Node* findCheapest();

    float calcTotalCost(Node* child, float given);
	float calcGivenCost(Node* child, Node* parent);

    bool isGoalNode(Node* node);
    bool accessible(pair<int, int> p);


private:

    Node arr2D[ROW][COL];

    GridPos startGridPos;
    GridPos goalGridPos;

    Vec3 startPos;
	Vec3 goalPos;

    Heuristic heuristic;

    float weight = 0.0f;

    list<Node*> openList;
};

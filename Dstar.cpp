#include "Dstar.h"

Dstar::Dstar(Matrix* map1)
{
	map = map1;
	int** grid = (*map).getGrid();
	//int numRow = sizeof(grid)/sizeof(grid[0]);
	//int numCol = sizeof(grid[0])/sizeof(grid[0][0]);
	int numRow = (*map).getWidth();
	int numCol = (*map).getLength();
	std::vector<Point> robotLoc = (*map).getRobotLoc();
	std::vector<Point> goalLoc = (*map).getGoalLoc();
	start = new Node(new Point(robotLoc.at(0)), new Point(robotLoc.at(1)), new Point(robotLoc.at(2)), new Point(robotLoc.at(3)), 0);
	goal = new Node(new Point(goalLoc.at(0)), new Point(goalLoc.at(1)), new Point(goalLoc.at(2)), new Point(goalLoc.at(3)), 0);

	for(int i = 0; i < numRow-1; i++)
	{
		for(int j = 0; j < numCol-1; j++)
		{
			int val = 0;
			int grid1 = map->getValue(i, j);
			int grid2 = map->getValue(i+1, j);
			int grid3 = map->getValue(i, j+1);
			int grid4 = map->getValue(i+1, j+1);
			int minimum = min(grid1, min(grid2, min(grid3, grid4)));
			if(minimum == -1)
			{
				val = 100000;
			}
			else
			{
				val = 0;
			}
			Node* node = new Node(new Point((double)i, (double)j, (double)((*map).getValue(i,j))),
													new Point((double)i+1, (double)j, (double)((*map).getValue(i+1,j))),
													new Point((double)i, (double)j+1, (double)((*map).getValue(i,j+1))),
													new Point((double)i+1, (double)j+1, (double)((*map).getValue(i+1,j+1))),
												val);
			if(*node == *start)
			{
				nodes.push_back(start);
			}
			else if(*node == *goal)
			{
				nodes.push_back(goal);
			}
			else
			{
				nodes.push_back(node);
			}
		}
	}
	for(Node* node1 : nodes)
	{
		for(int i = 0; i < nodes.size(); i++)
		{
			Node* node2 = nodes.at(i);
			if((*node1).isNeighbor(node2))
			{
				(*node1).addNeighbor(node2);
			}
		}
	}

}

int Dstar::where(std::vector<Node*> v, Node* node)
{
	for(int i = 0; i < v.size(); i++)
	{
		if(node == v.at(i))
		{
			return i;
		}
	}
	return -1;
}

std::vector<Node> Dstar::reconstruct_path(std::map<Node*, Node*> cameFrom, Node* current)
{
	std::vector<Node> total_path;
	total_path.push_back(*current);
	while(cameFrom.count(current) > 0)
	{
	    current = cameFrom[current];
	    total_path.push_back(*current);
	}
	return total_path;
}

std::vector<Node> Dstar::plan_path()
{
		double distance = -(*start).getAvgPoint().getX() + (*goal).getAvgPoint().getX() - (*start).getAvgPoint().getY() + (*goal).getAvgPoint().getY();

		std::map<Node*, Node*> cameFrom;

		std::map<Node*, double> gScore;
		gScore[start] = 0;

		std::map<Node*, double> fScore;
		fScore[start] = distance;

		std::vector<Node*> openSet;
		openSet.push_back(start);

		std::vector<Node*> closedSet;

		while(openSet.size() > 0)
		{
			Node* current = lowest(openSet, fScore);
			if(current == goal)
			{
				return reconstruct_path(cameFrom, current);
			}
			openSet.erase(openSet.begin() + where(openSet, current));
			closedSet.push_back(current);
			vector<Node*> neighbors = current->getNeighbors();
			for(Node* neighbor : neighbors)
			{
				 if(where(closedSet, neighbor) != -1)
				 {
						 continue;		// Ignore the neighbor which is already evaluated.
				 }

				 // The distance from start to a neighbor
				 double tentative_gScore = gScore[current] + dist_between(*current, *neighbor) + (*neighbor).getCost();

				 if(where(openSet, neighbor) == -1)	// Discover a new node
				 {
						 openSet.push_back(neighbor);
				 }
				 else if (tentative_gScore >= gScore[neighbor])
				 {
					 continue;
				 }

				 // This path is the best until now. Record it!
				 cameFrom[neighbor] = current;
				 gScore[neighbor] = tentative_gScore;
				 fScore[neighbor] = gScore[neighbor] + dist_between(*neighbor, *goal);
			 }
		}
		vector<Node> fail;
		return(fail);
}

Node* Dstar::lowest(std::vector<Node*> openSet, std::map<Node*, double> fScore)
{
	int lowestVal = 1000000;
	Node* lowest = openSet.at(0);
	for(Node* node: openSet)
	{
		if (fScore[node] < fScore[lowest]) {
			lowest = node;
		}
	}
	return lowest;
}

double Dstar::dist_between(Node node1, Node node2)
{
	Point point1 = node1.getAvgPoint();
	Point point2 = node2.getAvgPoint();
	return abs((point1).getX() - (point2).getX()) + abs((point1).getY() - (point2).getY());
}

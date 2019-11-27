#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <algorithm>
#include "Node.h"
#include "Matrix.h"
#include "Point.h"

using namespace std;

#ifndef DSTAR_H
#define DSTAR_H

class Dstar {
public:
	Dstar(Matrix* map);
	std::vector<Node> plan_path();

private:
	std::vector<Node*> nodes;
	double dist_between(Node i, Node j);
	Matrix* map;
	std::vector<Node> reconstruct_path(std::map<Node*, Node*> cameFrom, Node* current);
	int where(std::vector<Node*> v, Node* node);
	Node* start;
	Node* goal;
	Node* lowest(std::vector<Node*> openSet, std::map<Node*, double> fScore);
};

#endif

#include <string>
#include <iterator>
#include <vector>
#include "Point.h"

using namespace std;

#ifndef NODE_H
#define NODE_H

class Node {
public:
	// constructor, pass in a filename(csv) with grid values
	Node(Point* p1, Point* p2, Point* p3, Point* p4, int cost);
	Node(const Node& other);

	vector<Node*> getNeighbors();
	void addNeighbor(Node* node);
	bool isNeighbor(Node* possibleNeighbor);
	bool operator==(const Node &other) const;
	Point getAvgPoint();
	Point getP1();
	Point getP2();
	Point getP3();
	Point getP4();
	int getCost();

private:
	vector<Node*> neighbors;
	Point* p1;
	Point* p2;
	Point* p3;
	Point* p4;
	int cost;
};

#endif

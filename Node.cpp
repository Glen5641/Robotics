#include "Node.h"

Node::Node(Point* point1, Point* point2, Point* point3, Point* point4, int val)
{
	p1 = point1;
	p2 = point2;
	p3 = point3;
	p4 = point4;
	cost = val;
}

Node::Node(const Node& other)
{
	p1 = other.p1;
	p2 = other.p2;
	p3 = other.p3;
	p4 = other.p4;
	cost = other.cost;
}

bool Node::operator==(const Node &other) const
{
	int trueCount = 0;
	bool p1b = (*p1 == *other.p1) ||
		(*p1 == *other.p2) ||
		(*p1 == *other.p3) ||
		(*p1 == *other.p4);

	bool p2b = (*p2 == *other.p1) ||
		(*p2 == *other.p2) ||
		(*p2 == *other.p3) ||
		(*p2 == *other.p4);

	bool p3b = (*p3 == *other.p1) ||
		(*p3 == *other.p2) ||
		(*p3 == *other.p3) ||
		(*p3 == *other.p4);

	bool p4b = (*p3 == *other.p1) ||
		(*p4 == *other.p2) ||
		(*p4 == *other.p3) ||
		(*p4 == *other.p4);
	if (p1b)
		trueCount++;
	if (p2b)
		trueCount++;
	if (p3b)
		trueCount++;
	if (p4b)
		trueCount++;

	if (trueCount == 4)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void Node::addNeighbor(Node* neighbor)
{
	neighbors.push_back(neighbor);
}

bool Node::isNeighbor(Node* possibleNeighbor)
{
	int trueCount = 0;
	bool p1b = (*p1 == (*possibleNeighbor).getP1()) ||
				(*p1 == (*possibleNeighbor).getP2()) ||
				(*p1 == (*possibleNeighbor).getP3()) ||
				(*p1 == (*possibleNeighbor).getP4());

	bool p2b = (*p2 == (*possibleNeighbor).getP1()) ||
				(*p2 == (*possibleNeighbor).getP2()) ||
				(*p2 == (*possibleNeighbor).getP3()) ||
				(*p2 == (*possibleNeighbor).getP4());

	bool p3b = (*p3 == (*possibleNeighbor).getP1()) ||
				(*p3 == (*possibleNeighbor).getP2()) ||
				(*p3 == (*possibleNeighbor).getP3()) ||
				(*p3 == (*possibleNeighbor).getP4());

	bool p4b = (*p3 == (*possibleNeighbor).getP1()) ||
				(*p4 == (*possibleNeighbor).getP2()) ||
				(*p4 == (*possibleNeighbor).getP3()) ||
				(*p4 == (*possibleNeighbor).getP4());
	if (p1b)
		trueCount++;
	if (p2b)
		trueCount++;
	if (p3b)
		trueCount++;
	if (p4b)
		trueCount++;

	if (trueCount == 2)
	{
		return true;
	}
	else
	{
		return false;
	}
}

Point Node::getP1()
{
	return *p1;
}

Point Node::getP2()
{
	return *p2;
}

Point Node::getP3()
{
	return *p3;
}

Point Node::getP4()
{
	return *p4;
}

vector<Node*> Node::getNeighbors()
{
		return neighbors;
}

Point Node::getAvgPoint()
{
	double avgX = ((*p1).getX() + (*p2).getX() + (*p3).getX() + (*p4).getX())/4;
	double avgY = ((*p1).getY() + (*p2).getY() + (*p3).getY() + (*p4).getY())/4;
	Point* point = new Point(avgX, avgY, 0);
	return *point;
}

int Node::getCost()
{
	return cost;
}

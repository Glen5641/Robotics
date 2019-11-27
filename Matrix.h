#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>
#include <vector>
#include "Point.h"

using namespace std;

#ifndef MATRIX_H
#define MATRIX_H

//TODO robotLoc as an array of Points and a local variable, add goalLoc DO NOT keep track of robot on the grid
class Matrix {
public:
	// constructor, pass in a filename(csv) with grid values
	Matrix(string filename);

	// update the orientation
	void updateOrientation(int direction);
	void updateOrientation(char turn);

	// update grid cell with new value
	void updateGrid(int x, int y, int val);

	// get the orientation
	int getOrientation();

	// get the entire grid
	int** getGrid();

	// get the turtlebots current location
	std::vector<Point> getRobotLoc();

	void setRobotLoc(std::vector<Point> loc);

	// get the turtlebots current location
	std::vector<Point> getGoalLoc();

	void setGoalLoc(std::vector<Point> loc);

	// simply returns the value in this cell
	int getValue(int x, int y);

	// display the grid as a visual aid to users
	void displayGrid();

	// functions for finding Cartesian and Manhattan distances
	double cartesian(int x1, int y1, int x2, int y2);
	double manhattan(int x1, int y1, int x2, int y2);

	int getWidth();
	int getLength();

private:
	// matrix the grid/map will be stored in
	int** grid;

	// keep in memory the length and width for easy access
	int gridWidth;
	int gridLength;

	// store the orientation of the turtlebot
	int orientation;

	std::vector<Point> robotLoc;
	std::vector<Point> goalLoc;

	// helper function to retrieve length & width of map
	void findDim(string filename, int& length, int& width);
};

#endif

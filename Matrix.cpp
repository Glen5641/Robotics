#include "Matrix.h"
#include <ros/console.h>
#include <stdio.h>

/*
Constructor that takes in a filename(csv) and initializes
the grid with values

filename should be of the format below with values corresponding
to the following key:

-1 blocked area
0 open space
1 turtlebot

-1,-1,-1,-1,-1,-1,-1,
-1,0,0,0,0,0,-1,
-1,0,0,0,0,0,-1,
-1,0,0,0,0,0,-1,
-1,-1,-1,-1,-1,-1,-1
*/
Matrix::Matrix(string filename)
{
	gridLength = 0;
	gridWidth = 0;
	findDim(filename, gridLength, gridWidth);
	// Define the array here using the newfound dimensions
	grid = new int*[gridLength];
	for (int k = 0; k < gridLength; k++) grid[k] = new int[gridWidth];

	string value = "";
	int val = -1;
	ifstream file(filename);
	// Store values from the file into the grid matrix
	for (int i = 0; i < gridLength; i++)
	{
		for (int j = 0; j < gridWidth; j++)
		{
			getline(file, value, ',');
			if (i == 0 && j == 0)
			{
				grid[i][j] = val;
			}
			else
			{
				// Convert from string to int and store
				stringstream temp(value);
				temp >> val;
				grid[i][j] = val;
			}
		}
	}
}

/*
1: North
2: East
3: South
4: West
*/
void Matrix::updateOrientation(int direction)
{
	orientation = direction;
}

/*
l/L: Left turn
r/R: Right turn
*/
void Matrix::updateOrientation(char turn)
{
	if (orientation == 1)
	{
		if (turn == 'l' || turn == 'L') orientation = 4;
		if (turn == 'r' || turn == 'R') orientation = 2;
	}

	else if (orientation == 2)
	{
		if (turn == 'l' || turn == 'L') orientation = 1;
		if (turn == 'r' || turn == 'R') orientation = 3;
	}

	else if (orientation == 3)
	{
		if (turn == 'l' || turn == 'L') orientation = 2;
		if (turn == 'r' || turn == 'R') orientation = 4;
	}

	else if (orientation == 4)
	{
		if (turn == 'l' || turn == 'L') orientation = 3;
		if (turn == 'r' || turn == 'R') orientation = 1;
	}
}

/*
Given x and y update the cell with the new value
*/
void Matrix::updateGrid(int x, int y, int val)
{
	grid[y][x] = val;
}


/*
1: North
2: East
3: South
4: West
*/
int Matrix::getOrientation()
{
	return orientation;
}

/*
Returns the current grid
Can be used to calculate distances
*/
int** Matrix::getGrid()
{
	return grid;
}

void Matrix::setRobotLoc(vector<Point> location)
{
	robotLoc = location;
}

void Matrix::setGoalLoc(vector<Point> location)
{
	goalLoc = location;
}

/*
Gets the robots current location
x and y are updated to the robots current top-left cell
*/
vector<Point> Matrix::getRobotLoc()
{
	return robotLoc;
}

vector<Point> Matrix::getGoalLoc()
{
	return goalLoc;
}

/*
Return the value in the specified cell
*/
int Matrix::getValue(int x, int y)
{
	return grid[y][x];
}

/*
Creates an aesthetically pleasing view of the grid for users
*/
void Matrix::displayGrid()
{
	for (int i = 0; i < gridLength; i++)
	{
		for (int j = 0; j < gridWidth; j++)
		{
			if (grid[i][j] != -1) ROS_ERROR_STREAM(" ");
			ROS_ERROR_STREAM(grid[i][j]);
		}
		ROS_ERROR_STREAM(std::endl);
	}
	ROS_ERROR_STREAM(std::endl);
}

/*
This function finds the Cartesian distance between the given points
*/
double Matrix::cartesian(int x1, int y1, int x2, int y2)
{
	return sqrt(pow(x2-x1, 2) + pow(y2-y1, 2));
}

/*
This function finds the Manhattan distance between the given points
*/
double Matrix::manhattan(int x1, int y1, int x2, int y2)
{
	return abs((x2-x1) + (y2-y1));
}

void Matrix::findDim(string filename, int& length, int& width)
{
	string line;
	ifstream file(filename);

	// Retrieve the "length" of the designated area
	while (!file.eof())
	{
		getline(file, line);
		length++;
	}
	file.clear();
	file.seekg(0, ios::beg);

	// Retrieve the "width" of the designated area
	while (getline(file, line, ','))
	{
		width++;
	}
	width = width / length;
	width += 2;

}

int Matrix::getWidth()
{
	return gridWidth;
}

int Matrix::getLength()
{
	return gridLength;
}

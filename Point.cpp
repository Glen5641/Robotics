#include "Point.h"

Point::Point(double x1, double y1, double value1)
{
	x = x1;
  y = y1;
  value = value1;
}

Point::Point(const Point& other)
{
	x = other.x;
	y = other.y;
	value = other.value;
}

void Point::setX(double newX)
{
  x = newX;
}

void Point::setY(double newY)
{
  y = newY;
}

void Point::setValue(double newValue)
{
  value = newValue;
}

double Point::getX()
{
  return x;
}

double Point::getY()
{
  return y;
}

double Point::getValue()
{
  return value;
}

bool Point::operator==(const Point &other) const
{
  	if(x == other.x && y == other.y)
		{
			return true;
		}
		else
		{
			return false;
		}
}

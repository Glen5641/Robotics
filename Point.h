using namespace std;

#ifndef POINT_H
#define POINT_H

class Point {
public:
	Point(double x, double y, double value);
	Point(const Point& other);

  double getX();
  double getY();
  double getValue();
  void setX(double newX);
  void setY(double newY);
  void setValue(double newVal);
  bool operator==(const Point &other) const;

private:
  double x;
  double y;
  double value;
};

#endif

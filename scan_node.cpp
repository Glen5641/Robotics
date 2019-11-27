#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <std_msgs/Int32.h>
#include <ros/console.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <curses.h>
#include <boost/thread/thread.hpp>
#include <string>
#include <iostream>
#include <tf/transform_listener.h>
#include <math.h>
#include "Dstar.h"
#include "Matrix.h"
#include "Node.h"
#include "Point.h"

//Global Message Publisher/Subscriber
ros::Publisher vel_cmd;
ros::Subscriber sub_laser;
ros::Subscriber sub_bumper;

//Twist and window
geometry_msgs::Twist tw;
WINDOW* my_win;

// Not used
double base_length = .228;

// IF 1, VALS are set to SIMULATOR VALS
double SIM = 0;

//One Cell in SIMULATOR
double one_foot = .55;

//TURNS AND 1 FT in SIM
double SIM90 = 0.80;
double SIM180 = 1.6;
double SIMFT = 0.160;

//Trying to solve bug
int start_removed = 0;

//REAL LIFE TURN AND 1 FT
double deg90 = 1.11;
double deg180 = 1.95;
double ft = 0.16;

double PI 							= 3.14159265358979;	//PI CONSTANT
double SYM_VALUE 				= 0.015;		//SYMMETRY MACRO
double NUM_CHECK_VALUES = 319;			//LAZER MACRO
int FINISHED 						= 0;				//PROGRAM COMPLETION MACRO
int DRIVE 							= 1;				//USER DRIVE THREAD MACRO
int IS_OK 							= 1;				//BUMPER THREAD MACRO
int ESCAPE 							= 0;				//ESCAPE THREAD MACRO
int AVOID 							= 0;				//AVOID THREAD MACRO
int AUTODRIVE 					= 0;				//AUTONOMOUS DRIVE MACRO

int counter = 0;

//Orientation of robot
int orientation = 3;

//Global Matrix object
Matrix 	matrix = Matrix("grid.csv");

//Global Nodes for reading
vector<Node> nodes;

/////////////////////////////////////////////////////////////////////////
//FINISHED  FINISHED  FINISHED  FINISHED  FINISHED  FINISHED  FINISHED
//TOP  TOP  TOP  TOP  TOP  TOP  TOP  TOP  TOP  TOP  TOP  TOP  TOP  TOP
/////////////////////////////////////////////////////////////////////////
void turn_robot(int direction) {
	if(!orientation){
		tw.linear.x  = 0;
		tw.angular.z = 0;
		return;
	}
	tw.linear.x = 0;
	mvwprintw(my_win, 41, 28, " Current Direction: %d , Turning To: %d", orientation, direction);
	wrefresh(my_win);
	if(orientation == 1){
		if(direction == 1) tw.angular.z = 0.0;
		if(direction == 2) tw.angular.z = -deg90;
		if(direction == 3) tw.angular.z = deg180;
		if(direction == 4) tw.angular.z = deg90;
	}
	if(orientation == 2){
		if(direction == 1) tw.angular.z = deg90;
		if(direction == 2) tw.angular.z = 0.0;
		if(direction == 3) tw.angular.z = -deg90;
		if(direction == 4) tw.angular.z = deg180;
	}
	if(orientation == 3){
		if(direction == 1) tw.angular.z = deg180;
		if(direction == 2) tw.angular.z = deg90;
		if(direction == 3) tw.angular.z = 0.0;
		if(direction == 4) tw.angular.z = -deg90;
	}
	if(orientation == 4){
		if(direction == 1) tw.angular.z = -deg90;
		if(direction == 2) tw.angular.z = deg180;
		if(direction == 3) tw.angular.z = deg90;
		if(direction == 4) tw.angular.z = 0.0;
	}
	orientation = direction;
	return;
}
/////////////////////////////////////////////////////////////////////////
//FINISHED  FINISHED  FINISHED  FINISHED  FINISHED  FINISHED  FINISHED
//BOTTOM  BOTTOM  BOTTOM  BOTTOM  BOTTOM  BOTTOM  BOTTOM  BOTTOM  BOTTOM
/////////////////////////////////////////////////////////////////////////















///////////////////////////////////////////////////////////////////////
//Lazer Function
///////////////////////////////////////////////////////////////////////
void lazer_detected(const sensor_msgs::LaserScan::ConstPtr& msg) {
	//Sleep Rate
  ros::Rate r(10);

	//If in auto drive, continue
	if(DRIVE == 0 && AUTODRIVE == 1){

		//If nodes arent empty due to popping last nodes, continue
		if(!nodes.empty()){
			if(nodes.size() <= 1){
				mvwprintw(my_win, 3, 24, "           PATH REACHED           ");
				mvwprintw(my_win, 4, 24, "    To Exit Program (DELETE)      ");
				mvwprintw(my_win, 5, 24, "                                  ");
				mvwprintw(my_win, 6, 24, "                                  ");
				wrefresh(my_win);
				DRIVE = 1;
				AUTODRIVE = 0;
			} else {
				/////////////////////////////////////////////////////////////////
				//DETERMINE ORIENTATION
				/////////////////////////////////////////////////////////////////

				//SIZE CHECK
				Point *p = new Point(0,0,0);
				Node current = Node(p, p, p, p, 1);
				Node next = Node(p, p, p, p, 1);
				int o = 1;
				//if(nodes.size()-1 >= 0)
					current = nodes.at(nodes.size()-1);
				//else return;
				//if(nodes.size()-2 >= 0)
					next = nodes.at(nodes.size()-2);
				//else return;

				mvwprintw(my_win, 36, 28, "Next Node at (%lf, %lf)", next.Node::getAvgPoint().Point::getX(), next.Node::getAvgPoint().Point::getY());
				mvwprintw(my_win, 37, 28, "Counter (%d)", counter);
				counter++;
				wrefresh(my_win);

				//If next x is more, go WEST
				if((int)current.Node::getAvgPoint().Point::getX() > (int)next.Node::getAvgPoint().Point::getX()){
					o = 4;
				}
				//If next x is less, go East
				if((int)current.Node::getAvgPoint().Point::getX() < (int)next.Node::getAvgPoint().Point::getX()){
					o = 2;
				}
				//If next y is more, go North
				if((int)current.Node::getAvgPoint().Point::getY() > (int)next.Node::getAvgPoint().Point::getY()){
					o = 1;
				}
				//If next y is less, go South
				if((int)current.Node::getAvgPoint().Point::getY() < (int)next.Node::getAvgPoint().Point::getY()){
					o = 3;
				}
				if(orientation != o){
					//Turn 2 seconds
					ros::Time start_time = ros::Time::now();
					ros::Duration h_sec(0.25);
					tw.linear.x = 0.0;
					tw.angular.z = 0.0;
					while(ros::Time::now() - start_time < h_sec) {
						vel_cmd.publish(tw);
						r.sleep();
					}
					turn_robot(o);
					orientation = o;
					mvwprintw(my_win, 40, 25, "    %4.6f     ", tw.angular.z );
					start_time = ros::Time::now();
					ros::Duration one_sec(2.0);
					while(ros::Time::now() - start_time < one_sec) {
						vel_cmd.publish(tw);
						r.sleep();
					}
					//Stop for half second
					start_time = ros::Time::now();
					tw.linear.x = 0.0;
					tw.angular.z = 0.0;
					while(ros::Time::now() - start_time < h_sec) {
						vel_cmd.publish(tw);
						r.sleep();
					}
					mvwprintw(my_win, 43, 28, "ORIENTATION SET");
					//Return
					return;
					////////////////////////////////////////////////////////////////
					//Done Dealing With Orientation
					////////////////////////////////////////////////////////////////
				}	else {
					////////////////////////////////////////////////////////////////
					//Determining Number of Steps
					////////////////////////////////////////////////////////////////
					int steps = 0;
					int turned = 0;
					for(int i = nodes.size()-2; i >= 0; i -= 1){
						if(i >= 0 && i < nodes.size())
							next = nodes.at(i);
						else
							return;
						if(steps < 2 && turned == 0){
							if(orientation == 4 && ( current.Node::getAvgPoint().Point::getX() > next.Node::getAvgPoint().Point::getX() ) )
								steps++;
							else if(orientation == 2 && ( current.Node::getAvgPoint().Point::getX() < next.Node::getAvgPoint().Point::getX() ) )
								steps++;
							else if(orientation == 1 && ( current.Node::getAvgPoint().Point::getY() > next.Node::getAvgPoint().Point::getY() ) )
								steps++;
							else if(orientation == 3 && ( current.Node::getAvgPoint().Point::getY() < next.Node::getAvgPoint().Point::getY() ) )
								steps++;
							else
								turned = 1;
						}
					}
					////////////////////////////////////////////////////////////////
					//Check to make sure N steps are Ok
					////////////////////////////////////////////////////////////////
					//Print out mid for ranging
					double mid = msg->ranges[msg->ranges.size()/2];
					mvwprintw(my_win, 8, 25, "    %4.6f     ", msg->ranges[mid] );
					wrefresh(my_win);
					//Loop through mid messages and if not nan
					int incount = 0;
					for(int i = (msg->ranges.size()/4); i < (msg->ranges.size()/4*3); i += 5){
						//If less than moving range, object is detected, update and recalc
						int spot = 0;
						if(msg->ranges[i] <= one_foot*steps){
							spot = 1;
						}
						//if (!isnan(msg->ranges[i])){
							//incount++;
						//}
						//if(incount > 57){
						//	spot = 1;
						//}


						////////////////////////////////////////////////////////////////
						//Obstructed Path, Recalculate A* and Return
						////////////////////////////////////////////////////////////////
						if(spot != 0){
							mvwprintw(my_win, 39, 28, "RECALC!!!!!");
							wrefresh(my_win);
							vector<Point> location = matrix.Matrix::getRobotLoc();
							if(orientation == 1){
								matrix.Matrix::updateGrid((int)location.at(0).Point::getX(), (int)location.at(0).Point::getY()-spot, -1);
								matrix.Matrix::updateGrid((int)location.at(1).Point::getX(), (int)location.at(1).Point::getY()-spot, -1);
								mvwprintw(my_win, 32, 28, "Obstruction found at (%d, %d)", (int)location.at(0).Point::getX(), (int)location.at(0).Point::getY()-spot);
								mvwprintw(my_win, 33, 28, "Obstruction found at (%d, %d)", (int)location.at(1).Point::getX(), (int)location.at(1).Point::getY()-spot);
								mvwprintw(my_win, 34, 28, "SPOT: %d", spot);
								mvwprintw(my_win, 35, 28, "ORIENTATION: %d", orientation);
								wrefresh(my_win);
							}
							if(orientation == 2){
								matrix.Matrix::updateGrid((int)location.at(1).Point::getX()+spot, (int)location.at(1).Point::getY(), -1);
								matrix.Matrix::updateGrid((int)location.at(3).Point::getX()+spot, (int)location.at(3).Point::getY(), -1);
								mvwprintw(my_win, 32, 28, "Obstruction found at (%d, %d)", (int)location.at(1).Point::getX(), (int)location.at(1).Point::getY()-spot);
								mvwprintw(my_win, 33, 28, "Obstruction found at (%d, %d)", (int)location.at(3).Point::getX(), (int)location.at(3).Point::getY()-spot);
								mvwprintw(my_win, 34, 28, "SPOT: %d", spot);
								mvwprintw(my_win, 35, 28, "ORIENTATION: %d", orientation);
								wrefresh(my_win);
							}
							if(orientation == 3){
								matrix.Matrix::updateGrid((int)location.at(3).Point::getX(), (int)location.at(3).Point::getY()+spot, -1);
								matrix.Matrix::updateGrid((int)location.at(2).Point::getX(), (int)location.at(2).Point::getY()+spot, -1);
								mvwprintw(my_win, 32, 28, "Obstruction found at (%d, %d)", (int)location.at(3).Point::getX(), (int)location.at(3).Point::getY()-spot);
								mvwprintw(my_win, 33, 28, "Obstruction found at (%d, %d)", (int)location.at(2).Point::getX(), (int)location.at(2).Point::getY()-spot);
								mvwprintw(my_win, 34, 28, "SPOT: %d", spot);
								mvwprintw(my_win, 35, 28, "ORIENTATION: %d", orientation);
								wrefresh(my_win);
							}
							if(orientation == 4){
								matrix.Matrix::updateGrid((int)location.at(2).Point::getX()-spot, (int)location.at(2).Point::getY(), -1);
								matrix.Matrix::updateGrid((int)location.at(0).Point::getX()-spot, (int)location.at(0).Point::getY(), -1);
								mvwprintw(my_win, 32, 28, "Obstruction found at (%d, %d)", (int)location.at(0).Point::getX()-spot, (int)location.at(0).Point::getY());
								mvwprintw(my_win, 33, 28, "Obstruction found at (%d, %d)", (int)location.at(2).Point::getX()-spot, (int)location.at(2).Point::getY());
								mvwprintw(my_win, 34, 28, "SPOT: %d", spot);
								mvwprintw(my_win, 35, 28, "ORIENTATION: %d", orientation);
								wrefresh(my_win);
							}
							Dstar star = Dstar(&matrix);
							nodes = star.Dstar::plan_path();

							int **printable = matrix.Matrix::getGrid();
							mvwprintw(my_win, 43, 28, "6         ");
							wrefresh(my_win);
							int gridLength = matrix.Matrix::getLength();
							int gridWidth = matrix.Matrix::getWidth();
							for (int i = 0; i < gridLength; i++) {
								for (int j = 0; j < gridWidth; j++) {

									if (printable[i][j] == 0) {
										mvwprintw(my_win, 10+i, 20+(j*2), " 0");
									} else if (printable[i][j] == -1) {
										mvwprintw(my_win, 10+i, 20+(j*2), "-1");
									} else if (printable[i][j] == 2) {
										mvwprintw(my_win, 10+i, 20+(j*2), " 2");
									} else {
										mvwprintw(my_win, 10+i, 20+(j*2), " 1");
									}
								}
							}
							wrefresh(my_win);
							mvwprintw(my_win, 43, 28, "Recalc Finished           ");
							wrefresh(my_win);
							return;
						}
					}

					////////////////////////////////////////////////////////////////
					//If made it this far, Path is ok and need to take steps forward
					////////////////////////////////////////////////////////////////

					mvwprintw(my_win, 45, 28, "STEPS: %d        ", steps);
					wrefresh(my_win);
					for(int taken = 0; taken < steps; taken++){
						tw.linear.x = ft;
						tw.angular.z = 0.0;
						ros::Time start_time = ros::Time::now();
						ros::Duration one_sec(2.0);
						while(ros::Time::now() - start_time < one_sec) {
							vel_cmd.publish(tw);
							r.sleep();
						}
						mvwprintw(my_win, 43, 28, "8        ");
						wrefresh(my_win);
						start_time = ros::Time::now();
						ros::Duration h_sec(0.25);
						tw.linear.x = 0.0;
						tw.angular.z = 0.0;
						while(ros::Time::now() - start_time < h_sec) {
							vel_cmd.publish(tw);
							r.sleep();
						}
						mvwprintw(my_win, 43, 28, "9        ");
						wrefresh(my_win);
						//if(nodes.size()-2 >= 0)
							next = nodes.at((nodes.size()-2));
						vector<Point> new_loc;
						new_loc.push_back(next.Node::getP1());
						new_loc.push_back(next.Node::getP2());
						new_loc.push_back(next.Node::getP3());
						new_loc.push_back(next.Node::getP4());
						matrix.Matrix::setRobotLoc(new_loc);
						nodes.pop_back();
						mvwprintw(my_win, 43, 28, "10        ");
						wrefresh(my_win);
					}
					mvwprintw(my_win, 33, 28, "Size: %d        ", nodes.size());
					wrefresh(my_win);
					return;
				}
			}
		} else {

			//Never Reached
			mvwprintw(my_win, 3, 24, "         PATH UNREACHABLE         ");
			mvwprintw(my_win, 4, 24, "    To Exit Program (DELETE)      ");
			mvwprintw(my_win, 5, 24, "                                  ");
			mvwprintw(my_win, 6, 24, "                                  ");
			wrefresh(my_win);
			DRIVE = 1;
			AUTODRIVE = 0;
		}
	}
}



















/////////////////////////////////////////////////////////////////////////
//FINISHED  FINISHED  FINISHED  FINISHED  FINISHED  FINISHED  FINISHED
//TOP  TOP  TOP  TOP  TOP  TOP  TOP  TOP  TOP  TOP  TOP  TOP  TOP  TOP
/////////////////////////////////////////////////////////////////////////
void userDrive(){
	ros::Rate r(10);

	mvwprintw(my_win, 3, 24, "To Enter Autonomous Drive (ENTER) ");
	mvwprintw(my_win, 4, 24, "  Drive Robot By Using (ARROWS)   ");
	mvwprintw(my_win, 5, 24, "Set Orientation of Robot North (S)");
	mvwprintw(my_win, 6, 24, "    To Exit Program (DELETE)      ");
	wrefresh(my_win);

	int c = 0;
	while(IS_OK && !FINISHED) {
		if(DRIVE == 0) {
			switch((c=getch())) {
				case '\n':
					DRIVE = 1;
					AUTODRIVE = 0;
					vel_cmd.publish(geometry_msgs::Twist());
					r.sleep();
					break;
				case KEY_DC:
				  delwin(my_win);
					endwin();
					FINISHED = 1;
					return;
				default:
					break;
			}
		}
		if(DRIVE == 1) {
			tw.linear.x = 0;
			tw.angular.z = 0;
			switch((c=getch())) {
				case KEY_UP:
					tw.linear.x = ft;
					mvwprintw(my_win, 9, 40, "    FORWARD     " );
					wrefresh(my_win);
					break;
				case KEY_DOWN:
					tw.linear.x = -ft;
					mvwprintw(my_win, 9, 40, "    REVERSE     " );
					wrefresh(my_win);
					break;
				case KEY_RIGHT:
					tw.angular.z = -0.1;
					mvwprintw(my_win, 9, 40, "     RIGHT      " );
					wrefresh(my_win);
					break;
				case KEY_LEFT:
					tw.angular.z = 0.1;
					mvwprintw(my_win, 9, 40, "     LEFT       " );
					wrefresh(my_win);
					break;
				case '1':
					turn_robot(1);
					mvwprintw(my_win, 9, 40, "     NORTH      " );
					wrefresh(my_win);
					break;
				case '2':
					turn_robot(2);
					mvwprintw(my_win, 9, 40, "     EAST       " );
					wrefresh(my_win);
					break;
				case '3':
					turn_robot(3);
					mvwprintw(my_win, 9, 40, "     SOUTH      " );
					wrefresh(my_win);
					break;
				case '4':
					turn_robot(4);
					mvwprintw(my_win, 9, 40, "     WEST       " );
					wrefresh(my_win);
					break;
				case 's':
					orientation = 3;
					mvwprintw(my_win, 6, 24, "  Set Robot Direction (1,2,3,4)   ");
					mvwprintw(my_win, 7, 24, "    To Exit Program (DELETE)      ");
					mvwprintw(my_win, 9, 40, "SET ORIENTATION" );
					wrefresh(my_win);
					break;
				case '\n':
					mvwprintw(my_win, 3, 24, "          SELF NAVIGATING         ");
					mvwprintw(my_win, 4, 24, "    To Exit Program (DELETE)      ");
					mvwprintw(my_win, 5, 24, "                                  ");
					mvwprintw(my_win, 6, 24, "                                  ");
					mvwprintw(my_win, 9, 40, "                 " );
					wrefresh(my_win);
					DRIVE = 0;
					AUTODRIVE = 1;
					break;
				case KEY_DC:
					delwin(my_win);
					endwin();
					FINISHED = 1;
					return;
				default:
					break;
			}
      ros::Time start_time = ros::Time::now();
      ros::Duration one_sec(2.0);
      while(ros::Time::now() - start_time < one_sec) {
			  vel_cmd.publish(tw);
			  r.sleep();
      }
		}
	}
	delwin(my_win);
	endwin();
	return;
}
/////////////////////////////////////////////////////////////////////////
//FINISHED  FINISHED  FINISHED  FINISHED  FINISHED  FINISHED  FINISHED
//BOTTOM  BOTTOM  BOTTOM  BOTTOM  BOTTOM  BOTTOM  BOTTOM  BOTTOM  BOTTOM
/////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv){

	my_win = initscr();
	refresh();
	keypad(my_win, TRUE);
	box(my_win, 0, 0);  // added for easy viewing

	vector<Point> loc;
	vector<Point> goal;

	loc.push_back(Point(3,14,0));
	loc.push_back(Point(4,14,0));
	loc.push_back(Point(3,15,0));
	loc.push_back(Point(4,15,0));

	goal.push_back(Point(18,2,0));
	goal.push_back(Point(19,2,0));
	goal.push_back(Point(18,3,0));
	goal.push_back(Point(19,3,0));

	matrix.setRobotLoc(loc);
	matrix.setGoalLoc(goal);

	Dstar star = Dstar(&matrix);
	nodes = star.Dstar::plan_path();

	int **printable = matrix.Matrix::getGrid();
	int gridLength = matrix.Matrix::getLength();
	int gridWidth = matrix.Matrix::getWidth();
	for (int i = 0; i < gridLength; i++) {
		for (int j = 0; j < gridWidth; j++) {
			if (printable[i][j] == 0) {
				mvwprintw(my_win, 10+i, 20+(j*2), " 0");
			} else if (printable[i][j] == -1) {
				mvwprintw(my_win, 10+i, 20+(j*2), "-1");
			} else if (printable[i][j] == 2) {
				mvwprintw(my_win, 10+i, 20+(j*2), " 2");
			} else {
				mvwprintw(my_win, 10+i, 20+(j*2), " 1");
			}
		}
	}
	wrefresh(my_win);

	orientation = 2;

	if(SIM){
		deg90 = SIM90;
		deg180 = SIM180;
		ft = SIMFT;
	}

	//Initialize the node to control the robot
	ros::init(argc, argv, "scan_node");
	ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();
	//Subscribe to lazer scan topic

	//Publish to Command Velocity Topic
	vel_cmd = n->advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
	sub_laser = n->subscribe("/scan", 1, lazer_detected);
	//sub_bumper = n->subscribe("/mobile_base/events/bumper", 1, bumper_detected);

	//Let user drive robot
	boost::thread drive(userDrive);
	//boost::thread advance(autoDrive);

	//While ros is doing fine, go ahead and spin once
	while(1) {
		if(!IS_OK){
			drive.join();
			///advance.join();
			exit(EXIT_FAILURE);
		}
		if(FINISHED){
			drive.join();
			//advance.join();
			return(0);
		}
		ros::spinOnce();
	}
}















































/*double DTR(double degrees) {
	return degrees*PI/180;
}

double RTD(double rad) {
	return rad*180/PI;
}

int get_symmetry(const sensor_msgs::LaserScan::ConstPtr& msg){

	//Sums for the left and right sides of the sensor range
	float left_sum = 0;
	float right_sum = 0;

	//midpoint of the sensor values
	int mid = msg->ranges.size()/2;

	//Sums the left side of the sensor values starting at the mid - NUM_CHECK_VALUES
	for(int i = mid-NUM_CHECK_VALUES; i < mid; i++){
		if(!isnan(msg->ranges[i])){
			left_sum += msg->ranges[i];
		}else{
			left_sum += 100;
		}
	}

	//Sums the right side of the sensor values ending at the mid + NUM_CHECK_VALUES
	for(int i = mid+NUM_CHECK_VALUES; i > mid; i--){
		if(!isnan(msg->ranges[i])){
      right_sum += msg->ranges[i];
    }else{
      right_sum += 100;
    }
  }

	//Takes the mean of the sums and then normalizes them based on the left_sum
	float left_avg = left_sum/NUM_CHECK_VALUES;
	float right_avg = right_sum/NUM_CHECK_VALUES;

	//Symetry to return
	int sym = 0;

	//If the difference is greater than the threshold value the object is left centered
	if(left_avg-right_avg > 0){
		sym = -1;
	}	else if(left_avg-right_avg < 0){
		sym = 1;
	}

	//else the sym is centered
	return sym;
}*/

/*#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0.1;
  double vy = -0.1;
  double vth = 0.1;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(1.0);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}

/*#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

class RobotDriver
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "cmd_vel" topic to issue commands
  ros::Publisher cmd_vel_pub_;
  //! We will be listening to TF transforms as well
  tf::TransformListener listener_;

public:
  //! ROS node initialization
  RobotDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  }

  //! Drive forward a specified distance based on odometry information
  bool driveForwardOdom(double distance)
  {
    //wait for the listener to get the first message
    listener_.waitForTransform("base_footprint", "odom_combined",
                               ros::Time(0), ros::Duration(10.0));

    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the odometry to the base frame
    listener_.lookupTransform("base_footprint", "odom_combined",
                              ros::Time(0), start_transform);

    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
    //the command will be to go forward at 0.25 m/s
    base_cmd.linear.y = base_cmd.angular.z = 0;
    base_cmd.linear.x = 0.25;

    ros::Rate rate(10.0);
    bool done = false;
    while (!done && nh_.ok())
    {
      //send the drive command
      cmd_vel_pub_.publish(base_cmd);
      rate.sleep();
      //get the current transform
      try
      {
        listener_.lookupTransform("base_footprint", "odom_combined",
                                  ros::Time(5), current_transform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        break;
      }
      //see how far we've traveled
      tf::Transform relative_transform =
        start_transform.inverse() * current_transform;
      double dist_moved = relative_transform.getOrigin().length();

      if(dist_moved > distance) done = true;
    }
    if (done) return true;
    return false;
  }
};

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;

  RobotDriver driver(nh);
  driver.driveForwardOdom(0.5);
}

/*

bool turnOdom(bool clockwise, double radians) {
  while(radians < 0) radians += 2*M_PI;
  while(radians > 2*M_PI) radians -= 2*M_PI;

	tf::TransformListener listener_;

  //wait for the listener to get the first message
  listener_.waitForTransform("base_footprint", "odom_combined",
           ros::Time(0), ros::Duration(1.0));

  //we will record transforms here
  tf::StampedTransform start_transform;
  tf::StampedTransform current_transform;

  //record the starting transform from the odometry to the base frame
  listener_.lookupTransform("base_footprint", "odom_combined",
        	ros::Time(0), start_transform);

  //we will be sending commands of type "twist"
  geometry_msgs::Twist base_cmd;
  //the command will be to turn at 0.75 rad/s
  base_cmd.linear.x = base_cmd.linear.y = 0.0;
  base_cmd.angular.z = 0.75;
  if (clockwise) base_cmd.angular.z = -base_cmd.angular.z;

  //the axis we want to be rotating by
  tf::Vector3 desired_turn_axis(0,0,1);
  if (!clockwise) desired_turn_axis = -desired_turn_axis;

  ros::Rate rate(10.0);
  bool done = false;
  while (!done) {
    //send the drive command
    vel_cmd.publish(base_cmd);
    rate.sleep();
    //get the current transform
    try {
      listener_.lookupTransform("base_footprint", "odom_combined",
            ros::Time(0), current_transform);
    }
    catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
      break;
    }
    tf::Transform relative_transform =
      start_transform.inverse() * current_transform;
    tf::Vector3 actual_turn_axis =
      relative_transform.getRotation().getAxis();
    double angle_turned = relative_transform.getRotation().getAngle();
    if ( fabs(angle_turned) < 1.0e-2) continue;

    if ( actual_turn_axis.dot( desired_turn_axis ) < 0 )
      angle_turned = 2 * M_PI - angle_turned;

    if (angle_turned > radians) done = true;
  }
  if (done) return true;
  return false;
}

////////////////////////////////////////////////////////////////////////////////
// HELPER FUNCTIONS
////////////////////////////////////////////////////////////////////////////////
/*
 * Function
 * Generate radians from a degree angle
 * Formula Radian = 180/PI Degrees
 *//*
double generateAngular(int angle){
	return (double)((rand() % (angle*2)) - angle)/180.0*PI;
}

/*
 * Function
 * Determines if the sensor data is detecting a symetric object, a left centered object or a right centered object
 * returns -1 for left 0 for symetric 1 for right
*//*
int get_symetry(const sensor_msgs::LaserScan::ConstPtr& msg){
	//Sums for the left and right sides of the sensor range
	float left_sum = 0;
	float right_sum = 0;
	//midpoint of the sensor values
	int mid = msg->ranges.size()/2;
	//Sums the left side of the sensor values starting at the mid - NUM_CHECK_VALUES
	for(int i = mid-NUM_CHECK_VALUES; i < mid; i++){
		if(!isnan(msg->ranges[i])){
			left_sum += msg->ranges[i];
		}else{
			left_sum += 100;
		}
	}
	//Sums the right side of the sensor values ending at the mid + NUM_CHECK_VALUES
	for(int i = mid+NUM_CHECK_VALUES; i > mid; i--){
		if(!isnan(msg->ranges[i])){
      right_sum += msg->ranges[i];
    }else{
      right_sum += 100;
    }
  }
	//Takes the mean of the sums and then normalizes them based on the left_sum
	float left_avg = left_sum/NUM_CHECK_VALUES;
	float right_avg = (right_sum/NUM_CHECK_VALUES)/left_avg;
	left_avg = 1;
	//Symetry to return
	int sym = 0;
	//If the difference is greater than the threshold value the object is left centered
	if(left_avg-right_avg > SYM_VALUE){
		sym = -1;
	}	else if(left_avg-right_avg < -SYM_VALUE){
		sym = 1;
	}
	//else the sym is centered
	return sym;
}

////////////////////////////////////////////////////////////////////////////////
// LEVEL 6
////////////////////////////////////////////////////////////////////////////////
void turn() {

	//Set rate of robot to 10 Hz
	ros::Rate r(10);

	//Set Robot to turn +-15 degrees in 1 second
	tw.linear.x = 0.2;
	tw.angular.z = generateAngular(15);

	//Loop for 1 second publishing velocy message
	ros::Time start_time = ros::Time::now();
	ros::Duration one_sec(1.0);
	while(ros::Time::now() - start_time < one_sec) {
		vel_cmd.publish(tw);
		r.sleep();
	}
}

void autoDrive() {

	while(IS_OK && !FINISHED){
		if(AUTODRIVE){
			//Set rate of robot to 10 Hz
			ros::Rate r(10);

			//Set Robot to move 1 foot in 1 second
			tw.linear.x = 0.2;
			tw.angular.z = 0;

			//Loop for 1 second publishing velocy message
			vel_cmd.publish(tw);
			r.sleep();
			turn();
		}
		else {
			if(!DRIVE){
				ros::Time start_time = ros::Time::now();
				ros::Duration one_sec(1.0);
				while(ros::Time::now() - start_time < one_sec);
			}
		}
	}
	return;
}

////////////////////////////////////////////////////////////////////////////////
// LEVEL 5
////////////////////////////////////////////////////////////////////////////////
void avoid(int sym){

	mvwprintw(my_win, height/2,width/2, "AVOIDING" );
	//Set rate of the robot to 10 Hz
	ros::Rate r(10);

	//Stop the robot
	tw.linear.x = 0;
	//Check to see if the robot should turn left or right
	if(sym == -1){
		tw.angular.z = -1;
	}
	else if(sym == 1){
		tw.angular.z = 1;
	}

	//Publish the velocities to the robot
	vel_cmd.publish(tw);
	return;
}

////////////////////////////////////////////////////////////////////////////////
// LEVEL 4
////////////////////////////////////////////////////////////////////////////////
void escape() {

	mvwprintw(my_win, height/2,width/2, "ESCAPING" );
	ros::Rate r(10);

	//Set Robot to stop driving
	tw.linear.x = 0;
	tw.angular.z = PI+generateAngular(30);

	//Loop for 1 second publishing velocy message
	ros::Time start_time = ros::Time::now();
	ros::Duration one_sec(1.0);
	while(ros::Time::now() - start_time < one_sec) {
		vel_cmd.publish(tw);
		r.sleep();
	}

	//Loop for 1 second publishing zero velocy message to stop turning
	start_time = ros::Time::now();
	while(ros::Time::now() - start_time < one_sec) {
		vel_cmd.publish(geometry_msgs::Twist());
		r.sleep();
	}
	AUTODRIVE = 1;

}

////////////////////////////////////////////////////////////////////////////////
// LEVEL 3
////////////////////////////////////////////////////////////////////////////////
void lazer_detected(const sensor_msgs::LaserScan::ConstPtr& msg) {

			ros::Rate r(10);

			//Declare variables to obtain lowest value
			float lowest = 999.99;
			int lowest_index = -1;

			//Loop to check every 80th message.
			//Still gives multiple scans to read from for this case.
			for(int i = 0; i < msg->ranges.size(); i += 50){

				//Grab lowest
				if(msg->ranges[i]<lowest){
					lowest = msg->ranges[i];
					lowest_index = i;
				}
			}
			mvwprintw(my_win, height/2,width/2, "SCANNING" );
			wrefresh(my_win);
			//If closest object is within 1 unit activate either escape or avoid behaviors
			if(lowest <= 1) {
				AUTODRIVE = 0;
				//Check for symetry or asymetry
				int sym = get_symetry(msg);

				//sym = 0 on symetrical object
				if(sym == 0){
					vel_cmd.publish(geometry_msgs::Twist());
					r.sleep();
					escape();
				} else {
					vel_cmd.publish(geometry_msgs::Twist());
					r.sleep();
					avoid(sym);
				}
			} else {
				AUTODRIVE = 1;
			}

			//Publish center lazer scan for debugging
			//ROS_INFO("%f", msg->ranges[msg->ranges.size()/2]);
}

////////////////////////////////////////////////////////////////////////////////
// LEVEL 2
////////////////////////////////////////////////////////////////////////////////
void bumper_detected(const kobuki_msgs::BumperEvent::ConstPtr& msg) {
	ros::Rate r(10);
	tw.linear.x = 0;
	tw.angular.z = 0;
	vel_cmd.publish(tw);
	IS_OK = 0;
	return;
}*/

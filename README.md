# Robotics Project

## Description
Uses ROS and multithreading to control a turtlebot. Program allows turtlebot  
to traverse through an enclosed environment and dynamically change habits  
depending on objects in the way. Path finding is 1ft cells in a matrix along  
with the A* Path Finding Algorithm.

## Files
| File          | Language | Description                           |
|---------------|:--------:|---------------------------------------|
| Dstar         | C++      | Performs A* on Matrix                 |
| Matrix        | C++      | Matrix that outlines the room         |
| Node          | C++      | Helper for A* finding neighbors       |
| scan_node     | C++      | Driver Program that uses Pthreads     |
| Point         | C++      | Allows for Robotic Location on Matrix |
| grid          | csv      | CSV file with matrix. Easy Loading    |

#ifndef NBV_GRID
#define NBV_GRID
#pragma once
#include <vector>
#include "cmesh.h"

//make sure that quadrant and the camera_direction are corresponding
enum quadrant
{First = 0, Second, Third, Fourth ,Fifth, Sixth, Seventh, Eighth};

const Point3f 
camera_directions[8] = 
            { Point3f(1.0f, 1.0f, 1.0f),Point3f(-1.0f, 1.0f, 1.0f), Point3f(-1.0f, 1.0f, -1.0f), 
              Point3f(1.0f, 1.0f, -1.0f), Point3f(1.0f, -1.0f, 1.0f), Point3f(-1.0f, -1.0f, 1.0f), 
              Point3f(-1.0f, -1.0f, -1.0f), Point3f(1.0f, -1.0f, -1.0f)};

class NBVGrid
{
public:
  NBVGrid(): x_idx(-1), y_idx(-1), z_idx(-1) {}
  NBVGrid(int x_index, int y_index, int z_index);

  ~NBVGrid();

public:
  int    x_idx;
  int    y_idx;
  int    z_idx;
  double confidence;
  
  std::vector<int>      direction_count;
  //std::vector<Point3f>  camera_direction;
};

#endif
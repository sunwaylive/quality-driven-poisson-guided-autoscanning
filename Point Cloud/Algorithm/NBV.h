#pragma once
#include "PointCloudAlgorithm.h"
#include "GlobalFunction.h"
#include "NBVGrid.h"
#include <iostream>


using std::cout;
using std::endl;
using vcg::Point3f;

class NBV : public PointCloudAlgorithm
{
public:
  NBV(RichParameterSet* _para);
  ~NBV();

  void run();
  void setInput(DataMgr *pData);
  void setParameterSet(RichParameterSet *_para) { para = _para;}
  RichParameterSet * getParameterSet() { return para;}
  void clear();

private:
  void buildGrid();
  void propogate();
  int getStep(double s);

private:
  RichParameterSet *para;
  CMesh            *model;
  CMesh            *original;
  CMesh            *iso_points;
  CMesh            *all_nbv_grid_centers;
  double           grid_resolution;
  Point3f          whole_space_box_max;
  Point3f          whole_space_box_min;
  std::vector<NBVGrid>  *all_nbv_grids; //grids in all the space
};
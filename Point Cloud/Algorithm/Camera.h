#pragma once
#include <iostream>
#include <algorithm>
#include "GlobalFunction.h"
#include "PointCloudAlgorithm.h"

namespace vcc{

  using namespace std;

  struct CameraParameter{
    double near_dist;
    double far_dist;
    double near_width;
    double far_width;
    double angle;
  };

  class Camera : public PointCloudAlgorithm
  {
  public :
    Camera():
    pos(Point3f(0.0f, 0.0f, 1.0f)), 
    direction(Point3f(0.0f, 0.0f, -1.0f)),
    up(0.0f, 1.0f, 0.0f),
    right(1.0f, 0.0f, 0.0f)
    {
    }

    Camera(RichParameterSet* _para);
    ~Camera(){target = NULL; current_scanned_mesh = NULL; scanned_results = NULL;}

    void setInput(DataMgr* pData);
    void setParameterSet(RichParameterSet* _para){ para = _para;}
    RichParameterSet* getParameterSet() {return para;}
    void run();
    void clear() {}

  private:
    void runInitialScan();
    void runNBVScan();
    void runVirtualScan();
	  void runOneKeyNewScan();

  public:
     void computeUpAndRight();

  public:
    RichParameterSet*        para;
    CMesh*                   target;
    CMesh*                   original;
    vector<ScanCandidate>*   init_scan_candidates;//for initialization
    vector<ScanCandidate>*   scan_candidates;     //for nbv computing
    CMesh*                   current_scanned_mesh;
	  CMesh*                   nbv_candidates;
    //fix: this should be released in "compute nbv" function
    vector<CMesh* >*         scanned_results;
    double                   dist_to_model;
    Point3f                  pos;
    Point3f                  direction;
    Point3f                  up;
    Point3f                  right;
    double                   resolution;
    double                   far_distance;
    double                   near_distance;
    double                   far_horizon_dist;  //far horizontal range
    double                   far_vertical_dist;  //far vertical range
    double                   near_horizon_dist;
    double                   near_vertical_dist;
    CameraParameter          camera_para;
  };

}
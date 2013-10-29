#pragma once
#include <iostream>
#include <algorithm>
#include "GlobalFunction.h"
#include "PointCloudAlgorithm.h"

namespace vcc{

  using namespace std;

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

  public:
     void computeUpAndRight();

  public:
    RichParameterSet*        para;
    CMesh*                   target;
    CMesh*                   original;
    vector<ScanCandidate>*   init_scan_candidates;//for initialization
    vector<ScanCandidate>*   scan_candidates;     //for nbv computing
    CMesh*                   current_scanned_mesh;
    //fix: this should be released in "compute nbv" function
    vector<CMesh* >*         scanned_results;
    double                   dist_to_model;
    Point3f                  pos;
    Point3f                  direction;
    Point3f                  up;
    Point3f                  right;
    double                   resolution;
    double                   max_distance;
    double                   min_distance;
    double                   horizon_dist;  //total horizontal range
    double                   vertical_dist; //total vertical range
  };

}
#ifndef VISIBILITY_BASED_NBV_H
#define VISIBILITY_BASED_NBV_H

#pragma once
#include <iostream>
#include <math.h>
#include "PointCloudAlgorithm.h"
#include "GlobalFunction.h"
#include <QMessageBox>

class VisibilityBasedNBV : public PointCloudAlgorithm
{
public:
  VisibilityBasedNBV(RichParameterSet* _para);
  ~VisibilityBasedNBV();

  void run();
  void setInput(DataMgr *pData);
  void setParameterSet(RichParameterSet* _para) {para = _para;}
  RichParameterSet* getParameterSet() {return para;}
  void clear();

private:
  void runVisibilityPropagate();
  void runVisibilityCandidatesCluster();
  void runVisibilityCandidatesPick(); // wsh 5-19

  void runVisibilityMerge();
  void runVisibilityUpdate();
  void runVisibilitySmooth();
  bool isPointWellVisible(const CVertex &target, const Point3f &view_pos, const Point3f &view_dir, const CMesh* mesh_surface);
  void runComputeCurrentVisibility();

  static bool cmp2(const CVertex &v1, const CVertex &v2);

private:
  RichParameterSet      *para;
  double                optimalDist;
  CMesh                 *original;
  CMesh                 *model;
  CMesh                 *nbv_candidates;
  vector<ScanCandidate> *scan_history;
  vector<ScanCandidate> *scan_candidates;
  vector<CMesh *>*      scanned_results;
  static double         general_radius;
};
#endif
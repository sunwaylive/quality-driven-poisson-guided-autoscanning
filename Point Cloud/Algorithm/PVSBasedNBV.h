#ifndef PVS_BASED_NBV_H
#define PVS_BASED_NBV_H

#include <iostream>
#include <math.h>
#include "PointCloudAlgorithm.h"
#include "GlobalFunction.h"
#include "vcg\complex\trimesh\update\selection.h"
#include "vcg\complex\algorithms\update\topology.h"
#include "vcg\complex\complex.h"
#include "vcg/complex/algorithms/create/platonic.h" //for mesh copy

typedef vcg::tri::UpdateFlags<CMesh>::EdgeSorter MyEdge;
typedef std::vector<MyEdge>::iterator MyEdgeIter;
typedef vcg::tri::UpdateFlags<CMesh>::FaceIterator FaceIter;

class PVSBasedNBV : public PointCloudAlgorithm
{
public:
  //tri::UpdateSelection<CMeshO>::VertexFromBorderFlag(m.cm);
  void detectBoundary();
  PVSBasedNBV(RichParameterSet* _para);
  ~PVSBasedNBV();

  void run();
  void setInput(DataMgr *pData);
  void setParameterSet(RichParameterSet* _para) {para = _para;}
  RichParameterSet* getParameterSet() {return para;}
  void clear();

private:
  void runPVSDetectBoundary();

private:
  RichParameterSet      *para;
  double                optimalDist;
  CMesh                 *sample;
  CMesh                 *original;
  CMesh                 *nbv_candidates;
  vector<ScanCandidate> *scan_candidates;
  vector<CMesh *>*      scanned_results;

  class Boundary{
  public:
    Boundary()  { flag = 0x0000;}
    ~Boundary() { }

    enum{
      UP = 0x0001,   //up boundary
      DOWN = 0x0002, //down boundary
      LEFT = 0x0004, //left boundary
      RIGHT = 0x0008 //right boundary
    };

    std::vector<MyEdge> v_board_edges;
    int flag;

    inline bool isUpBoundary() const {return this->flag & UP;}
    inline bool isDownBoundary() const {return this->flag & DOWN;}
    inline bool isLeftBoundary() const {return this->flag & LEFT;}
    inline bool isRightBoundary() const {return this->flag & RIGHT;}
    inline void setUpBoundary() {this->flag |= UP;}
    inline void setDownBoundary() {this->flag |= DOWN;}
    inline void setLeftBoundary() {this->flag |= LEFT;}
    inline void setRightBoundary() {this->flag |= RIGHT;}
    inline void clearUpBoundary() {this->flag &= (~UP);}
    inline void clearDownBoundary() {this->flag &= (~DOWN);}
    inline void clearLeftBoundary() {this->flag &= (~LEFT);}
    inline void clearRightBoundary() {this->flag &= (~RIGHT);}
  };
};

#endif
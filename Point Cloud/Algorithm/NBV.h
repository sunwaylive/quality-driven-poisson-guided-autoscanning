#pragma once
#include "PointCloudAlgorithm.h"
#include "GlobalFunction.h"
#include <iostream>

using std::cout;
using std::endl;

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
  void runBuildGrid();

private:
  RichParameterSet *para;
  CMesh            *original;
};
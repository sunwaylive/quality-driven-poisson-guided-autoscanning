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


private:
  RichParameterSet *para;
  CMesh            *original;
};
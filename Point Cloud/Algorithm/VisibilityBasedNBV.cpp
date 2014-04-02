#include "VisibilityBasedNBV.h"

VisibilityBasedNBV::VisibilityBasedNBV(RichParameterSet* _para)
{
  std::cout<<"VisibilityBasedNBV constructed!"<<std::endl;
  para = _para;
}

VisibilityBasedNBV::~VisibilityBasedNBV()
{

}

void VisibilityBasedNBV::setInput(DataMgr *pData)
{
  optimalDist = global_paraMgr.camera.getDouble("Camera Dist To Model");
}

void VisibilityBasedNBV::run()
{
  if (para->getBool("Run Visibility Propagate"))
  {
    std::cout<< "Run Visibility Propagate" <<std::endl;
  }
}

void VisibilityBasedNBV::clear()
{

}
#include "NBV.h"

NBV::NBV(RichParameterSet *_para)
{
  cout<<"NBV constructed!"<<endl;
  para = _para;
  original = NULL;
}

NBV::~NBV()
{

}

void
NBV::run()
{
  if (para->getBool("Run Build Grid"))
  {
    runBuildGrid();
    return;
  }
}

void
NBV::setInput(DataMgr *pData)
{
  if (!pData->isOriginalEmpty() && !pData->isModelEmpty())
  {
    CMesh *_original = pData->getCurrentOriginal();
    CMesh *_model = pData->getCurrentModel();

    if (NULL == _original)
    {
      cout<<"ERROR: NBV original == NULL !"<<endl;
      return;
    }
    if (NULL == _model)
    {
      cout<<"ERROR: NBV model == NULL !"<<endl;
      return;
    }
    model = _model;
    original = _original;
  }else
  {
    cout<<"ERROR: NBV::setInput empty!"<<endl;
  }
}

void
NBV::clear()
{
  original = NULL;
}

void
NBV::runBuildGrid()
{
   Point3f bbox_max = model->bbox.max;
   Point3f bbox_min = model->bbox.min;
   //get the whole 3D space that a camera may exist
   double camera_max_dist = global_paraMgr.camera.getDouble("Camera Max Dist");
   Point3f whole_space_box_max = bbox_max + Point3f(camera_max_dist, camera_max_dist, camera_max_dist);
   Point3f whole_space_box_min = bbox_min - Point3f(camera_max_dist, camera_max_dist, camera_max_dist);
   //compute the size of the 3D space
   Point3f dif = whole_space_box_max - whole_space_box_min;
   
}
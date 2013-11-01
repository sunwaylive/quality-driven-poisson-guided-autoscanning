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
  if (!pData->isOriginalEmpty())
  {
    CMesh *_original = pData->getCurrentOriginal();
    if (NULL == _original)
    {
      cout<<"ERROR: NBV original == NULL !"<<endl;
      return;
    }

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

}
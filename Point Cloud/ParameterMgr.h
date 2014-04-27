#pragma once
#include "Parameter.h"
#include "CMesh.h"

class ParameterMgr
{
public:
	ParameterMgr(void);
	~ParameterMgr(void);
  RichParameterSet* getDataParameterSet()              { return &data; }
  RichParameterSet* getDrawerParameterSet()            { return &drawer; }
  RichParameterSet* getGlareaParameterSet()            { return &glarea; }
  RichParameterSet* getWLopParameterSet()              { return &wLop; }
	RichParameterSet* getSkeletonParameterSet()          { return &skeleton; }	
	RichParameterSet* getNormalSmootherParameterSet()    { return &norSmooth; }
	RichParameterSet* getUpsamplingParameterSet()        { return &upsampling; }
  RichParameterSet* getPoissonParameterSet()           { return &poisson; }
  RichParameterSet* getCameraParameterSet()            { return &camera; }
  RichParameterSet* getNBVParameterSet()               { return &nbv;   }
  RichParameterSet* getVisibilityBasedNBVParameterSet(){ return &visibilityBasedNBV;}
  RichParameterSet* getPVSBasedNBVParameterSet()       { return &pvsBasedNBV;}

	void setGlobalParameter(QString paraName,Value& val);
	typedef enum {GLAREA, DATA, DRAWER, WLOP, NOR_SMOOTH, SKELETON, UPSAMPLING, POISSON}ParaType;

private:
	void initDataMgrParameter();
	void initDrawerParameter();
	void initGlareaParameter();
	void initWLopParameter();
	void initSkeletonParameter();
	void initNormalSmootherParameter();
	void initUpsamplingParameter();
  void initPoissonParameter();
  void initCameraParameter();
  void initNBVParameter();
  void initVisibilityBasedNBVParameter();
  void initPVSBasedNBVParameter();

public:
	RichParameterSet glarea;
	RichParameterSet data;
	RichParameterSet drawer;
	RichParameterSet wLop;
	RichParameterSet norSmooth;
	RichParameterSet skeleton;
	RichParameterSet upsampling;
  RichParameterSet poisson;
  RichParameterSet camera;
  RichParameterSet nbv;
  RichParameterSet visibilityBasedNBV;
  RichParameterSet pvsBasedNBV;

private:
	static int init_time;
	double grid_r;
};

extern ParameterMgr global_paraMgr;
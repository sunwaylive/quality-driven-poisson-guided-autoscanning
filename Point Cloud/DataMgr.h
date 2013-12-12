#pragma once

#include "cmesh.h"
#include "Parameter.h"
#include "GlobalFunction.h"
#include "Algorithm/Skeleton.h"
#include "NBVGrid.h"
//#include "Algorithm/Poisson.h"

#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export.h>

#include <sstream>
#include <fstream>
#include <set>
#include <utility>

using namespace vcg;
using namespace std;
using namespace tri;

typedef pair<Point3f, Point3f> ScanCandidate;

class DataMgr
{
public:
	DataMgr(RichParameterSet* _para);
	~DataMgr(void);

  void      loadPlyToModel(QString fileName);
  void      loadPlyToOriginal(QString fileName);
  void      loadPlyToSample(QString fileName);
  void      loadPlyToISO(QString fileName);

	void      savePly(QString fileName, CMesh& mesh);
	void      loadImage(QString fileName);
  void      loadXYZN(QString fileName);
  void      loadCameraModel(QString fileName);

  bool      isModelEmpty();
  bool      isSamplesEmpty();
  bool      isOriginalEmpty();
  bool      isSkeletonEmpty();
  bool      isIsoPointsEmpty();
  bool      isFieldPointsEmpty();
  bool      isViewCandidatesEmpty();
  bool      isScannedMeshEmpty();
  bool      isScannedResultsEmpty();
  bool      isNBVGridsEmpty();
  bool      isNBVCandidatesEmpty();

  void                    setCurrentTemperalSample(CMesh *mesh);

  CMesh*                  getCurrentSamples();
  CMesh*                  getCurrentTemperalSamples();
  CMesh*                  getCurrentModel();
  CMesh*                  getCurrentOriginal();
  CMesh*                  getCurrentIsoPoints();
  CMesh*                  getCurrentFieldPoints();
  Slices*                 getCurrentSlices();
  Skeleton*               getCurrentSkeleton();
  
  CMesh*                  getCameraModel();
  Point3f&                getCameraPos();
  Point3f&                getCameraDirection();
  double                  getCameraResolution();
  double                  getCameraHorizonDist();
  double                  getCameraVerticalDist();
  double                  getCameraMaxDistance();
  double                  getCameraMaxAngle();
  vector<NBVGrid>*        getAllNBVGrids();
  CMesh*                  getViewGridPoints();
  CMesh*                  getNbvCandidates();
  vector<ScanCandidate>*  getInitCameraScanCandidates();
  vector<ScanCandidate>*  getAllScanCandidates();
  vector<ScanCandidate>*  getSelectedScanCandidates();
  CMesh*                  getCurrentScannedMesh();
  vector<CMesh* >*        getScannedResults(); 


	void      recomputeBox();
	double    getInitRadiuse();

  void      removeOutliers();

	void      downSamplesByNum(bool use_random_downsample = true);
	void      subSamples();

	void      normalizeROSA_Mesh(CMesh& mesh, bool is_original = false);
	Box3f     normalizeAllMesh();

  void     eraseRemovedSamples();
  void     clearData();
  void     recomputeQuad();

	void     loadSkeletonFromSkel(QString fileName);
	void     saveSkeletonAsSkel(QString fileName);
  void     saveFieldPoints(QString fileName);
  void     saveViewGrids(QString fileName);


  void switchSampleToOriginal();
  void switchSampleToISO();
  void replaceMesh(CMesh& src_mesh, CMesh& target_mesh, bool isOriginal);
  void replaceMesh2(CMesh& src_mesh, CMesh& target_mesh, bool isIso);



private:
	void clearCMesh(CMesh& mesh);
  void initDefaultScanCamera();

public:
  CMesh                  model;
  CMesh                  original;
  Point3f                original_center_point;
  CMesh                  samples;
  CMesh*                 temperal_sample;
  CMesh                  iso_points;
  CMesh                  field_points;
  CMesh                  camera_model;
  std::vector<NBVGrid>   view_grids;
  CMesh                  view_grid_points;
  CMesh                  nbv_candidates;
  Point3f                camera_pos;
  Point3f                camera_direction;
  double                 camera_horizon_dist;
  double                 camera_vertical_dist;
  double                 camera_resolution;
  double                 camera_max_distance;
  double                 camera_max_angle;
  vector<ScanCandidate>  init_scan_candidates;
  vector<ScanCandidate>  scan_candidates;
  vector<ScanCandidate>  selected_scan_candidates;
  CMesh                  current_scanned_mesh; 
  vector<CMesh *>        scanned_results;  
                  
	Skeleton        skeleton;
  Slices          slices;

	RichParameterSet*   para;
	double              init_radius;
	QString             curr_file_name;

  Box3f whole_space_box;
};


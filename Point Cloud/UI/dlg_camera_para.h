#pragma once
#include "Algorithm/normal_extrapolation.h"
#include "Algorithm/pointcloud_normal.h"
#include "../GeneratedFiles/ui_camera_para.h"
#include "DataMgr.h"
#include "ParameterMgr.h"
#include "glarea.h"

#include <QtGui>
#include <QtGui/QFrame>
#include <QtGui/QWidget>
#include <iostream>

using namespace std;

class CameraParaDlg : public QFrame
{
  Q_OBJECT

public:
  CameraParaDlg(QWidget *p, ParameterMgr * _paras, GLArea * _area);
  ~CameraParaDlg();

  void initConnects();
  void setFrameConent();

  private slots:
    bool initWidgets();
    void virtualScan();
    void initialScan();
    void NBVCandidatesScan();
    void NBVCandidatesScanByHand();
    void loadRealScans();
    void loadRealInitialScan();
    void showInitCameras(bool is_show);
    void showCameraBorder(bool is_show);
    void useOtherInsideSegment(bool _val);
    void useConfidenceSeparation(bool _val);
    void needMoreOverlaps(bool _val);
    void useNbvTest1(bool _val);
    void useMaxConfidencePropagation(bool _val);
    void showCandidateIndex();

    void updateTableViewNBVCandidate();
    void updateTabelViewScanResults();
    void showSelectedScannCandidates(QModelIndex index);
    void showSelectedScannedMesh(QModelIndex index);
    void mergeScannedMeshWithOriginal();
    void mergeScannedMeshWithOriginalByHand();
    void getViewPruneConfidenceThreshold(double _val);
    void getMergeConfidenceThreshold(double _val);
    void getCameraHorizonDist(double _val);
    void getCameraVerticalDist(double _val);
    void getCameraDistToModel(double _val);
    void getCameraResolution(int _val);
    void getGridResolution(double _val);
    void getMaxRaySteps(double _val);
    void getIsoBottomDelta(double _val);
    void getCameraFarDistance(double _val);
    void getCameraNearDistance(double _val);
    void getPredictedModelSize(double _val);
    void getOptimalPlaneWidth(double _val);
    void getPropagateIndex(double _val);
    void getRayResolutionPara(double _val);
    void getNbvIterationCount(int _val);
    void getNBVTopN(int _val);
    void getNBVConfidenceThreshold(double _val);

    void buildGrid();
    void propagate();
    void propagateOnePoint();
    void gridSegment();
    void extractViewCandidates();
    void extractViewIntoBins();
    void runViewClustering();
    void runViewPrune();
    void runSetIsoBottomConfidence();
    void runUpdateViewDirections();

    void runSetupInitialScanns();
    void runStep1WLOP();
    void runStep2PoissonConfidence();
    void runStep2PoissonConfidenceViaOiginal();
    void runStep3NBVcandidates();
    void runWlopOnScannedMesh();
    void runStep4NewScans();
    void runOneKeyNbvIteration();

    //add samples
    void runRemoveSampleOutliers();
    void runICP();
    void runRemoveSamplesWithLowConfidence();
    void runAddSamplesToOiriginal();

    void getRotateCenterX(double _val);
    void getRotateCenterY(double _val);
    void getRotateCenterZ(double _val);
    void getRotateNormalX(double _val);
    void getRotateNormalY(double _val);
    void getRotateNormalZ(double _val);
    void getRotateStep(double _val);
    void getRotateAngle(double _val);
    void needSnapShotEachIteration(bool _val);
    void getSnapShotIndex(double _val);

    void rotateStep();
    void rotateAnimation();
    void sliceAnimation();
    void loadPoissonSurface();
    void moveTranslation();
    
    /*** visibility based NBV ***/
    void visibilityFirstScan();
    void visibilityPropagate();
    void visibilityCandidatesCluster();
    void visibilityCandidatesPick();
    void visibilityCandidateScan();
    void visibilityUpdate();
    void visibilityMerge();
    void runVisibilityOneKeyNbvIteration();
    void runVisibilitySmooth();
    void computeCurrentVisibility();
    /*** visibility based NBV ***/

    /*** pvs based NBV ***/
    void pvsFirstScan();
    void pvsBuildPVS();
    void pvsUpdatePVS();
    void pvsSearchNewBoundaries();
    void pvsSearchNewBoundariesByBallpivoting();
    void pvsComputeCandidates();
    void pvsSelectCandidate();
    void pvsNBVScan();
    void pvsMerge();
    void pvsOneKeyNbvIteration();
    /*** pvs based NBV ***/

    void getModelSize();

private:
  Ui::camera_paras * ui;
  ParameterMgr * m_paras;
  GLArea * area;
};
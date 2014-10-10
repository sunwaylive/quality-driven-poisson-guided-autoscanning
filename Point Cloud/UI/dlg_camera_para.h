#pragma once
#include "Algorithm/normal_extrapolation.h"
#include "Algorithm/pointcloud_normal.h"
#include "../GeneratedFiles/ui_camera_para.h"
#include "DataMgr.h"
#include "ParameterMgr.h"
#include "glarea.h"
#include "OneKeyNBVBack.h"

#include <QtGui>
#include <QtGui/QFrame>
#include <QtGui/QWidget>
#include <iostream>
#include <QtCore/QDebug> 
#include <QThread>

using namespace std;

class CameraParaDlg : public QFrame
{
  Q_OBJECT

public:
  CameraParaDlg(QWidget *p, ParameterMgr * _paras, GLArea * _area);
  ~CameraParaDlg();
  
  friend OneKeyNBVBack;
  void initConnects();
  void setFrameConent();
signals:
  void sig_runOneKeyNbvIterationBack(QString, GLArea*);

  private slots:
    bool initWidgets();
    void virtualScan();
    void initialScan();
    void NBVCandidatesScan();
    void NBVCandidatesScanByHand();
    void loadRealInitialScan();
    void loadToOriginal();
    void loadToModel();

    void showInitCameras(bool is_show);
    void showCameraBorder(bool is_show);
    void useOtherInsideSegment(bool _val);
    void useConfidenceSeparation(bool _val);
    void needMoreOverlaps(bool _val);
    void useMaxConfidencePropagation(bool _val);
    void showCandidateIndex();

    void updateTableViewNBVCandidate();
    void updateTabelViewScanResults();
    void showSelectedScannCandidates(QModelIndex index);
    void showSelectedScannedMesh(QModelIndex index);
    void mergeScannedMeshWithOriginal();
    void mergeScannedMeshWithOriginalUsingHoleConfidence();
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
    void runStep2CombinedPoissonConfidence();
    void runStep2HolePoissonConfidence();
    void runStep2PoissonConfidenceViaOiginal();
    void runStep3NBVcandidates();
    void runStep4NewScans();
    void runOneKeyNbvIteration();
    void runOneKeyNbvIterationBack();
        
    void getModelSize();

private:
  Ui::camera_paras * ui;
  ParameterMgr * m_paras;
  GLArea * area;
  OneKeyNBVBack m_nbv;
  QReadWriteLock nbv_mutex;
};
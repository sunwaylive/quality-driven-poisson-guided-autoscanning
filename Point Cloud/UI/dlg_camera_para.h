#pragma once

#include <QtGui>
#include <QtGui/QFrame>
#include <QtGui/QWidget>
#include <iostream>

#include "../GeneratedFiles/ui_camera_para.h"
#include "DataMgr.h"
#include "ParameterMgr.h"
#include "glarea.h"

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
  void showInitCameras(bool is_show);
  void useOtherInsideSegment(bool _val);
  void useConfidenceSeperation(bool _val);
  void useAverageConfidence(bool _val);
  void useNbvTest1(bool _val);
  void useMaxConfidencePropagation(bool _val);

  void updateTableViewNBVCandidate();
  void updateTabelViewScanResults();
  void showSelectedScannCandidates(QModelIndex index);
  void showSelectedScannedMesh(QModelIndex index);
  void mergeScannedMeshWithOriginal();
  void getCameraHorizonDist(double _val);
	void getCameraVerticalDist(double _val);
  void getCameraMaxDist(double _val);
  void getCameraDistToModel(double _val);
  void getGridResolution(double _val);
  void getMaxRaySteps(double _val);
  void buildGrid();
  void propagate();
  void propagateOnePoint();
  void gridSegment();

private:
	Ui::camera_paras * ui;
	ParameterMgr * m_paras;
	GLArea * area;
};
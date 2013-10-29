#pragma once

#include <QtGui>
#include <QtGui/QFrame>
#include <QtGui/QWidget>
#include <iostream>

#include "Algorithm/Poisson.h"
#include "../GeneratedFiles/ui_poisson_para.h"
#include "ParameterMgr.h"
#include "glarea.h"

using namespace std;

class PoissonParaDlg : public QFrame
{
	Q_OBJECT
public:
	PoissonParaDlg(QWidget *p, ParameterMgr * _paras, GLArea * _area);
	~PoissonParaDlg();
	void initConnects();
	
	void setFrameConent();

private slots:
	bool initWidgets();

	void getRadiusValues(double _val);
  void getPoissonDepth(double _val);
  void getPoissonIsoInterval(double _val);

	void runPoissonAndExtractLeafs();
  void runPoissonAndExtractNodes();
  void runPoissonAndExtractMC();
  void runSlice();

  void removeNonIsoPoints();
  void labelIsoPoints();
  void labelSmooth();
  void labelBoundaryPoints();
  void computeViewCandidates();
  void viewCandidatesClustering();
	void clearLabel();

  void showSlices(bool _val);
  void showSlicesTransparent(bool _val);
  void showSlicesX(bool _val);
  void showSlicesY(bool _val);
  void showSlicesZ(bool _val);

  void useConfidence1(bool _val);
  void useConfidence2(bool _val);
  void useConfidence3(bool _val);
  void useConfidence4(bool _val);
  void computeOriginalConfidence();
  void computeSamplesConfidence();
  void computeIsoConfidence();




		
private:
	Ui::poisson_paras * ui;
	ParameterMgr * m_paras;
	GLArea * area;
};
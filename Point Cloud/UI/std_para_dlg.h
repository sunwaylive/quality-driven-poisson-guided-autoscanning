#pragma once

#include <iostream>
#include <QDialog>
#include <QtGui>
#include <QtGui/QFrame>
#include <QtGui/QWidget>
#include <QtGui/QDockWidget>
#include <QtGui/QGridLayout>

#include "ParameterMgr.h"
#include "UI/dlg_wlop_para.h"
#include "UI/dlg_normal_para.h"
#include "UI/dlg_poisson_para.h"
#include "UI/dlg_camera_para.h"

#include "glarea.h"

using namespace std;

class StdParaDlg : public QDockWidget
{
  Q_OBJECT
public:
  StdParaDlg(ParameterMgr* _paras, GLArea * _area, QWidget* parent = 0);
  ~StdParaDlg();

  bool showWlopParaDialog();
  bool showNormalParaDlg();
  bool showUpsamplingParaDlg();
  bool showPoissonParaDlg();
  bool showCameraParaDlg();

private:
  void init();
  void createFrame();
  void loadWlopFrame();
  void loadNormalFrame();
  void loadPoissonFrame();
  void loadCameraFrame();

  private slots:
    void closeClick();

private:
  WlopParaDlg          * para_wlop;
  NormalParaDlg        * para_normal;
  PoissonParaDlg       * para_poisson;
  CameraParaDlg        * para_camera; 

  ParameterMgr * paras;
  QFrame * mainFrame;
  GLArea * gla;
};
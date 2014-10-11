#include "UI/std_para_dlg.h"

StdParaDlg::StdParaDlg(ParameterMgr* _paras, GLArea * _area, QWidget* parent /* = 0 */) 
  : QDockWidget(QString("Plugin"), parent)
{
  paras = _paras;
  gla = _area;

  init(); // it's important
}

void StdParaDlg::init()
{
  para_camera = NULL;
  mainFrame = NULL;
}

bool StdParaDlg::showPoissonParaDlg()
{
  createFrame();
  loadPoissonFrame();
  return true;
}

bool StdParaDlg::showCameraParaDlg()
{
  createFrame();
  loadCameraFrame();
  return true;
}

void StdParaDlg::createFrame()
{
  if(mainFrame) delete mainFrame;
  QFrame *newFrame = new QFrame;
  setWidget(newFrame);
  setSizePolicy(QSizePolicy::Minimum,QSizePolicy::Minimum);
  mainFrame = newFrame;	
}

void StdParaDlg::loadPoissonFrame()
{
  assert(mainFrame);
  mainFrame->hide();
  QGridLayout *gridLayout = new QGridLayout(mainFrame);
  mainFrame->setLayout(gridLayout);
  setWindowTitle("Poisson Parameters");

  para_poisson = new PoissonParaDlg(this, paras, gla);
  para_poisson->setFrameConent();

  gridLayout->setRowMinimumHeight(2,380);
  gridLayout->setColumnMinimumWidth(1,180);

  gridLayout->addWidget(para_poisson,1,0,13,12);

  mainFrame->showNormal();
  mainFrame->adjustSize();
  //set the minimum size so it will shrink down to the right size
  this->setMinimumSize(mainFrame->sizeHint());
  this->showNormal();
  this->adjustSize();
}

void StdParaDlg::loadCameraFrame()
{
  assert(mainFrame);
  mainFrame->hide();
  QGridLayout *gridLayout = new QGridLayout(mainFrame);
  mainFrame->setLayout(gridLayout);
  setWindowTitle("Camera Parameters");

  para_camera = new CameraParaDlg(this, paras, gla);

  para_camera->setFrameConent();

  gridLayout->setRowMinimumHeight(2, 380);
  gridLayout->setColumnMinimumWidth(1, 240);

  gridLayout->addWidget(para_camera, 1, 0, 13, 12);

  mainFrame->showNormal();
  mainFrame->adjustSize();
  //set the minimum size so it will shrink down to the right size
  this->setMinimumSize(mainFrame->sizeHint());
  this->showNormal();
  this->adjustSize();
}

void StdParaDlg::closeClick()
{
  //gla->paraMgr.setGlobalParameter("Skeleton Mode", BoolValue(false));
  cout << "close." << endl;
  close();
}

StdParaDlg::~StdParaDlg()
{
  cout << "De-construct StdParaDlg." << endl;

  // just set it to NULL
  gla = NULL;

  if (para_camera)
    delete para_camera;
  para_camera = NULL;

  if(mainFrame)
    delete mainFrame;
  mainFrame = NULL;
}
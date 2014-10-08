#pragma once

#include "Algorithm/normal_extrapolation.h"
#include "glarea.h"
#include "ParameterMgr.h"
#include "CMesh.h"

#include <QString>
#include <iostream>
#include <QtCore/QDebug>

using namespace std;

class CameraParaDlg;
class GLArea;

class OneKeyNBVBack : public QObject
{
  Q_OBJECT
signals:
  void updateTableViewNBVCandidate();
  void updateTabelViewScanResults();
  void mergeScannedMeshWithOriginal();

public:
  OneKeyNBVBack(){}
  OneKeyNBVBack( QString f, GLArea *area );
  ~OneKeyNBVBack();
  
public slots:
  void oneKeyNBV(QString file_location, GLArea *area);
};
#pragma once
#include "gl/glew.h"
//切换32-64位，只需要更换dll，以及选择正确的QT版本并重新编译，这两个步骤
#include <QWidget>
#include <QColor>
#include <QImage>
#include <QPoint>
#include <QtGui>
#include <QtOpenGL/QGLWidget>
#include <QString>

#include <wrap/gl/trimesh.h>
#include <wrap/gui/trackball.h>
#include <vcg/space/point3.h>
#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export.h>

#include "Algorithm/pointcloud_normal.h"
#include "Algorithm/NormalSmoother.h"
#include "DataMgr.h"
#include "GLDrawer.h"
#include "CMesh.h"
#include "ParameterMgr.h"
#include "Algorithm/PointCloudAlgorithm.h"
#include "Algorithm/Poisson.h"
#include "wrap/gui/coordinateframe.h"
#include "Algorithm/Camera.h"
#include "Algorithm/NBV.h"

#include <iostream>
using std::cout;
using std::endl;
using vcg::Point3f;
using namespace vcg;
using std::vector;

class GLArea : public QGLWidget
{
public:
	Q_OBJECT

public:
	GLArea(QWidget *parent = 0);
	~GLArea(void);

	void initializeGL();
	void resizeGL(int w, int h);
	void paintGL(); 
	void updateUI(){emit needUpdateStatus();}

	void loadDefaultModel();
	void openByDrop(QString fileName);
	void initAfterOpenFile();
	void initView();
	void initSetting();

	void runNormalSmoothing();
	void runPoisson();
	void runCamera();
	void runNBV();

	void cleanPickPoints();

	void saveView(QString fileName);
	void loadView(QString fileName);
  void saveNBV(QString fileName);
	void outputColor(ostream& out, QColor& color);
	QColor inputColor(istream& in);
	void readRGBNormal(QString fileName);

	void removePickPoint();
  void removeOutliers();
  void moveAllCandidates(bool is_forward);
  void removeBadCandidates();

signals:
	void needUpdateStatus();

private:
	void runPointCloudAlgorithm(PointCloudAlgorithm& algorithm);


private:
	void drawNeighborhoodRadius();
  void drawNBVBall();
	void initLight();
	void lightOnOff(bool _val);

  double nbv_ball_slice;


private: // For Light Control Ball: Shift + Ctrl + mouse drag
	bool activeDefaultTrackball; 
	bool isDefaultTrackBall()   { return activeDefaultTrackball; }
	vcg::Trackball trackball_light;
	void drawLightBall();

private: // For pick points function
	int x1, y1, x2, y2;
	bool doPick;
	vector<int> pickList;
	int pickPoint(int x, int y, vector<int> &result, int width=4, int height=4, bool only_one = true);

	bool isDragging;
	bool isRightPressed;
	void drawPickRect();

	vector<int> friendPickList;
	vector<int> fatherPickList;
	vector<int> RGBPickList;

	void addPointByPick();
	void changePointByPick();
	int RGB_counter;
	void addRBGPick(int pick_index);

private: // For snapshot
	int tileCol, tileRow, totalCols, totalRows;
	QImage snapBuffer;
	bool takeSnapTile;
	SnapshotSetting ss;

	void pasteTile();
	void setView(); 
	void recoverView();

	void setTiledView(GLdouble fovY, float viewRatio, float fAspect, GLdouble zNear, GLdouble zFar,  float cameraDist);
	QSize curSiz;
	float fov;
	float clipRatioFar;
	float clipRatioNear;
	float nearPlane;
	float farPlane;

	QString default_snap_path;
	QString current_snap_path;
	double snapDrawScal;
	bool is_paintGL_locked;
  bool is_figure_shot;

	//Point3f rotate_normal;
	//Point3f rotate_pos;

  void drawCandidatesConnectISO();

	vcg::GlTrimesh<CMesh> glw; //绘制网格的对象

public:
	void saveSnapshot();
  void figureSnapShot();
	void changeColor(QString paraName);

  //rotate
  void rotatingAnimation();
  float rotate_angle;
  bool need_rotate;
  Point3f rotate_normal;
  Point3f rotate_pos;
  double rotate_delta;
  //void printPickPointInfo();
  //void pickRotateCenter();
  //void pickRotateNormal();

  bool initial_light_have_set;

public:
	void poissonTest();

private:
	vcg::Trackball trackball;
	vcg::Box3f gl_box;

private:
	void wheelEvent(QWheelEvent *e);
	void mouseMoveEvent(QMouseEvent *e);
	void mousePressEvent(QMouseEvent *e);
	void mouseReleaseEvent(QMouseEvent *e); 
	void keyReleaseEvent ( QKeyEvent *e);

private:
	QMutex paintMutex;

public:
	DataMgr dataMgr;
	GLDrawer glDrawer;	

  NormalSmoother     norSmoother;
  Poisson            poisson;
  vcc::Camera        camera;
  NBV                nbv;
	RichParameterSet* para;
};


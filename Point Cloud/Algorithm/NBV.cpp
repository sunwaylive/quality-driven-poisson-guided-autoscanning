#include "NBV.h"

NBV::NBV(RichParameterSet *_para)
{
  cout<<"NBV constructed!"<<endl;
  para = _para;
  original = NULL;
  iso_points = NULL;
  model = NULL;
  grid_resolution = 1.0f / 10; 
}

NBV::~NBV()
{
  original = NULL;
  iso_points = NULL;
  model = NULL;
}

void
NBV::run()
{
  if (para->getBool("Run Build Grid"))
  {
    buildGrid();
    return;
  }

  if (para->getBool("Run Propagate"))
  {
    propagate();
    return;
  }
}

void
NBV::setInput(DataMgr *pData)
{
  if (!pData->isOriginalEmpty() && !pData->isModelEmpty())
  {
    CMesh *_original = pData->getCurrentOriginal();
    CMesh *_model = pData->getCurrentModel();

    if (NULL == _original)
    {
      cout<<"ERROR: NBV original == NULL !"<<endl;
      return;
    }
    if (NULL == _model)
    {
      cout<<"ERROR: NBV model == NULL !"<<endl;
      return;
    }
    model = _model;
    original = _original;
  }else
  {
    cout<<"ERROR: NBV::setInput empty!"<<endl;
  }

  all_nbv_grid_centers = pData->getAllNBVGridCenters();
  all_nbv_grids = pData->getAllNBVGrids();
  iso_points = pData->getCurrentIsoPoints();
}

void
NBV::clear()
{
  original = NULL;
}

void
NBV::buildGrid()
{
  //fix: this should be model->bbox.max
   Point3f bbox_max = iso_points->bbox.max;
   Point3f bbox_min = iso_points->bbox.min;
   //get the whole 3D space that a camera may exist
   double camera_max_dist = global_paraMgr.camera.getDouble("Camera Max Dist");
   whole_space_box_max = bbox_max + Point3f(camera_max_dist, camera_max_dist, camera_max_dist);
   whole_space_box_min = bbox_min - Point3f(camera_max_dist, camera_max_dist, camera_max_dist);
   //compute the size of the 3D space
   Point3f dif = whole_space_box_max - whole_space_box_min;
   //divide the box into grid
   
   x_max = static_cast<int> (dif.X() / grid_resolution);
   y_max = static_cast<int> (dif.Y() / grid_resolution);
   z_max = static_cast<int> (dif.Z() / grid_resolution);
   //pre allocate the memory
   int max_index = x_max * y_max * z_max;
   all_nbv_grid_centers->vert.resize(max_index);
   all_nbv_grids->resize(max_index);
   //increase from whole_space_box_min
   for (int i = 0; i < x_max; ++i)
   {
     for (int j = 0; j < y_max; ++j)
     {
       for (int k = 0; k < z_max; ++k)
       {
         //add the grid
         int index = i * y_max * z_max + j * z_max + k;
         NBVGrid grid;
         grid.x_idx = i;
         grid.y_idx = j;
         grid.z_idx = k;
         (*all_nbv_grids)[index] = grid;

         //add the center point of the grid
         CVertex t;
         t.m_index = index;
         t.is_grid_center = true;
         t.P()[0] = whole_space_box_min.X() + k * grid_resolution;
         t.P()[1] = whole_space_box_min.Y() + j * grid_resolution;
         t.P()[2] = whole_space_box_min.Z() + i * grid_resolution;
         all_nbv_grid_centers->vert[index] = t;
         all_nbv_grid_centers->bbox.Add(t.P());
       }
     }
   }
}

void
NBV::propagate()
{
  //traverse all points on the iso surface
  for (int i = 0; i < iso_points->vert.size(); ++i)
  {
    CVertex &t = iso_points->vert[i];
    //get the x,y,z index of each iso_points
    int t_indexX = static_cast<int>( ceil((t.P()[0] - whole_space_box_min.X()) / grid_resolution ));
    int t_indexY = static_cast<int>( ceil((t.P()[1] - whole_space_box_min.Y()) / grid_resolution ));
    int t_indexZ = static_cast<int>( ceil((t.P()[2] - whole_space_box_min.Z()) / grid_resolution ));
    int n_indexX, n_indexY, n_indexZ;
    //1. for each point, propagate to all discrete directions
    //get the sphere traversal resolution
    double camera_max_dist = global_paraMgr.camera.getDouble("Camera Max Dist");
    //compute the delta of a,b so as to traverse the whole sphere
    double angle_delta = grid_resolution / camera_max_dist;
    //loop for a, b
    double a = 0.0f, b = 0.0f;
    double l = 0.0f;
    double x = 0.0f, y = 0.f, z = 0.0f;
    //for DDA
    //int stepX = 0, stepY = 0, stepZ = 0;
    int max_steps = static_cast<int>(camera_max_dist / grid_resolution);
    double length = 0.0f;
    double deltaX, deltaY, deltaZ;
    for (; a < PI; a += angle_delta)
    {
      l = sin(a); y = cos(a);
      for (; b < 2 * PI; b += angle_delta)
      {
        //now the propagate direction is Point3f(x, y, z)
        x = l * sin(b); z = l * cos(b);
        //reset the next grid indexes
        n_indexX = t_indexX; n_indexY = t_indexY; n_indexZ = t_indexZ;
        //2. compute the next grid indexes
        //determine the stepX stepY stepZ
        //stepX = getStep(x); stepY = getStep(y); stepZ = getStep(z);
        //compute all the grids that a ray intersect
        //get the maximum among x,y,z
        length = getAbsMax(x, y, z);

        deltaX = x / length; 
        deltaY = y / length;
        deltaZ = z / length;
        for (int k = 0; k <= max_steps; ++k)
        {
          n_indexX = round(n_indexX + deltaX);
          n_indexY = round(n_indexY + deltaY);
          n_indexZ = round(n_indexZ + deltaZ);
          //do what we need in the next grid
          

        }
      }
    }
    

    //3.record the values and count the direction bins
  }
}

int
NBV::getStep(double s)
{
  if (0.0f == abs(s)) return 0;
  else if (s > 0.0f)  return 1;
  else                return -1;
}

double
NBV::getAbsMax(double x, double y, double z)
{
  return std::max(abs(x), std::max(abs(y), abs(z)));
}

int 
NBV::round(double x)
{
  return static_cast<int>(x + 0.5);
}
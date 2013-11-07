#include "NBVGrid.h"

NBVGrid::NBVGrid(int x_index, int y_index, int z_index)
{
  x_idx = x_index;
  y_idx = y_index;
  z_idx = z_index;
  direction_count.resize(8);
}

NBVGrid::~NBVGrid()
{

}
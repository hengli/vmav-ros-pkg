#include "dynocmap/OccupancyTile.h"

namespace px
{

OccupancyTile::OccupancyTile()
 : m_resolution(0.0)
 , m_tileGridWidth(0)
 , m_tileGridCenter(Eigen::Vector3i::Zero())
{

}

OccupancyTile::OccupancyTile(double resolution, int tileGridWidth,
                             const Eigen::Vector3i& tileGridCenter)
 : m_resolution(resolution)
 , m_tileGridWidth(tileGridWidth)
 , m_tileGridCenter(tileGridCenter)
{

}

double&
OccupancyTile::resolution(void)
{
    return m_resolution;
}

double
OccupancyTile::resolution(void) const
{
    return m_resolution;
}

double
OccupancyTile::tileWidth(void) const
{
    return m_tileGridWidth * m_resolution;
}

int&
OccupancyTile::tileGridWidth(void)
{
    return m_tileGridWidth;
}

int
OccupancyTile::tileGridWidth(void) const
{
    return m_tileGridWidth;
}

Eigen::Vector3d
OccupancyTile::tileCenter(void) const
{
    return m_tileGridCenter.cast<double>() * m_resolution;
}

Eigen::Vector3i&
OccupancyTile::tileGridCenter(void)
{
    return m_tileGridCenter;
}

Eigen::Vector3i
OccupancyTile::tileGridCenter(void) const
{
    return m_tileGridCenter;
}

std::vector<OccupancyCell>&
OccupancyTile::cells(void)
{
    return m_cells;
}

const std::vector<OccupancyCell>&
OccupancyTile::cells(void) const
{
    return m_cells;
}

std::vector<OccupancyCell>&
OccupancyTile::obstacles(void)
{
    return m_obstacles;
}

const std::vector<OccupancyCell>&
OccupancyTile::obstacles(void) const
{
    return m_obstacles;
}

}

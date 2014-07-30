#ifndef OCCUPANCYTILE_H
#define OCCUPANCYTILE_H

#include <Eigen/Core>
#include <vector>

#include "dynocmap/OccupancyCell.h"

namespace px
{

class OccupancyTile
{
public:
    OccupancyTile();
    OccupancyTile(double resolution, int tileGridWidth,
                  const Eigen::Vector3i& tileGridCenter);

    double& resolution(void);
    double resolution(void) const;

    double tileWidth(void) const;

    int& tileGridWidth(void);
    int tileGridWidth(void) const;

    Eigen::Vector3d tileCenter(void) const;

    Eigen::Vector3i& tileGridCenter(void);
    Eigen::Vector3i tileGridCenter(void) const;

    std::vector<OccupancyCell>& cells(void);
    const std::vector<OccupancyCell>& cells(void) const;

    std::vector<OccupancyCell>& obstacles(void);
    const std::vector<OccupancyCell>& obstacles(void) const;

private:
    double m_resolution;
    int m_tileGridWidth;
    Eigen::Vector3i m_tileGridCenter;

    std::vector<OccupancyCell> m_cells;
    std::vector<OccupancyCell> m_obstacles;
};

}

#endif

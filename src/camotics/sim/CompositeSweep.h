/******************************************************************************\

  CAMotics is an Open-Source simulation and CAM software.
  Copyright (C) 2011-2019 Joseph Coffland <joseph@cauldrondevelopment.com>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

\******************************************************************************/

#pragma once


#include "Sweep.h"

#include <cbang/SmartPointer.h>

#include <vector>


namespace CAMotics {
  class CompositeSweep : public Sweep { // 复杂曲面。
    std::vector<cb::SmartPointer<Sweep> > children; // 构成复杂曲面的子曲面。
    std::vector<double> zOffsets; // z轴偏移量。

  public:
      // 一个成员函数，用于向当前复杂曲面对象添加一个具体的曲面对象和对应的z轴偏移量。
    void add(const cb::SmartPointer<Sweep> &sweep, double zOffset = 0);

    // From Sweep
    // 一个继承自Sweep类的虚函数，用于根据给定的起点和终点，计算CompositeSweep对象在扫描路径上的包围盒（bounding box）的集合，
    // 包围盒是一个可以完全包含目标形状的最小矩形。
    void getBBoxes(const cb::Vector3D &start, const cb::Vector3D &end,
                   std::vector<cb::Rectangle3D> &bboxes,
                   double tolerance = 0.01) const;
    // 一个继承自Sweep类的虚函数，用于根据给定的起点和终点，计算CompositeSweep对象在扫描路径上的某一点的深度，
    // 深度是指该点到扫描路径平面的垂直距离。
    double depth(const cb::Vector3D &start, const cb::Vector3D &end,
               const cb::Vector3D &p) const;
  };
}

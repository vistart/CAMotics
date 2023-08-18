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

#include <cbang/geom/Rectangle.h>

#include <vector>


namespace GCode {class Move;}

namespace CAMotics {
  class Sweep { // 曲面
  public:
    virtual ~Sweep() {} // Compiler needs this
    // 取得当前曲面的外边框，同时指定z轴的长度和z轴偏移量。
    // 求解后边框的结果为bboxes，radius为指定半径。
    // start 和 end 为大致边框范围的起止点。
    void getBBoxes(const cb::Vector3D &start, const cb::Vector3D &end,
                   std::vector<cb::Rectangle3D> &bboxes, double radius,
                   double length, double zOffset,
                   double tolerance = 0.01) const;

    // 取得当前曲面的外边框，不特殊对待z轴。
    // 求解后边框的结果为bboxes。
    // start 和 end 为大致边框范围的起止点。
    // 该函数固定返回0.
    virtual void getBBoxes(const cb::Vector3D &start, const cb::Vector3D &end,
                           std::vector<cb::Rectangle3D> &bboxes,
                           double tolerance = 0.01) const = 0;
    // 取得指定方向的矩形是否相交。
    // 该函数固定返回 false。
    virtual bool intersects(const GCode::Move &move,
                            const cb::Rectangle3D &box) const {return false;}

    // 取得深度。
    // 该函数固定返回 0.
    virtual double depth(const cb::Vector3D &start, const cb::Vector3D &end,
                       const cb::Vector3D &p) const = 0;
  };
}

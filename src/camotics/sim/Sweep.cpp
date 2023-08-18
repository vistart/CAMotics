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

#include "Sweep.h"

#include <gcode/Move.h>

using namespace std;
using namespace cb;
using namespace CAMotics;

// 取得当前曲面的外边框，同时指定z轴的长度和z轴偏移量。
// 求解后边框的结果为bboxes，radius为指定半径。
// start 和 end 为大致边框范围的起止点。
void Sweep::getBBoxes(const Vector3D &start, const Vector3D &end,
                      vector<Rectangle3D> &bboxes, double radius,
                      double length, double zOffset, double tolerance) const {
  const unsigned maxLen = radius * 16; // 设定最大长度为半径16倍。
  double len = start.distance(end); // 取得大致起止范围长度。
  unsigned steps = (len <= maxLen) ? 1 : (len / maxLen); // 如果大致长度比半径16倍小，则步长为1，否则为二者的比值。
  double stride = 1.0 / steps; // 条带宽度为步长的倒数。
  Vector3D p1 = start; // p1 为起点
  Vector3D p2; // p2 为临时范围点

  for (unsigned i = 0; i < steps; i++) {
    for (unsigned j = 0; j < 3; j++) // 确定临时点坐标，每次调大一点
      p2[j] = start[j] + (end[j] - start[j]) * stride * (i + 1);
    // 确定三维坐标上下限
    double minX = std::min(p1.x(), p2.x()) - radius - tolerance;
    double minY = std::min(p1.y(), p2.y()) - radius - tolerance;
    double minZ = std::min(p1.z(), p2.z()) + zOffset - tolerance;
    double maxX = std::max(p1.x(), p2.x()) + radius + tolerance;
    double maxY = std::max(p1.y(), p2.y()) + radius + tolerance;
    double maxZ = std::max(p1.z(), p2.z()) + length + tolerance;
    // 每次确定的范围加入到边框列表中。
    bboxes.push_back
      (Rectangle3D(Vector3D(minX, minY, minZ),
                       Vector3D(maxX, maxY, maxZ)));

    p1 = p2; // 当前的重点为下一轮循环的起点
  }
}

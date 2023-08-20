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

#include "Workpiece.h"

using namespace std;
using namespace cb;
using namespace CAMotics;


Workpiece::Workpiece(const Rectangle3D &r) : //表示一个工件。这个类继承了Rectangle3D类，表示一个三维矩形。这个类有以下特点：
  Rectangle3D(r), center(r.getCenter()) { // 构造函数，接受一个Rectangle3D对象作为参数，用来初始化工件的位置和尺寸，并计算工件的中心点和半径的平方。
  Vector3D halfDim = r.getDimensions() / 2;
  halfDim2 = halfDim * halfDim;
}


double Workpiece::depth(const Vector3D &p) const { // depth方法，接受一个Vector3D对象作为参数，表示一个空间中的点。这个方法返回这个点到工件表面最近的距离的平方，如果这个点在工件内部，则返回正值，否则返回负值。
  double d2 = p.distanceSquared(closestPointOnSurface(p));
  return Rectangle3D::contains(p) ? d2 : -d2;
}

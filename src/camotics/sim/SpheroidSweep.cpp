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

#include "SpheroidSweep.h"

#include <gcode/Move.h>

#include <algorithm>

using namespace std;
using namespace cb;
using namespace CAMotics;

// 实现了SpheroidSweep类的构造函数和depth方法。这个类表示一个椭球形的工具扫过的形状，用来模拟切割过程。这个类继承了Sweep类，表示一个抽象的扫过形状。这个类的构造函数和depth方法的流程如下：
SpheroidSweep::SpheroidSweep(double radius, double length) : // 构造函数：接受两个双精度浮点数作为参数，分别表示工具的半径和长度。这个函数用来初始化radius和length，并根据长度计算scale和radius2。如果长度为负数，则表示工具是一个球形，否则表示工具是一个椭球形。如果工具是一个椭球形，则需要将球形沿着z轴缩放，以达到椭球形的效果。scale是一个三维向量，表示工具的缩放比例。radius2是一个双精度浮点数，表示工具的半径的平方，用来计算距离。
  radius(radius), length(length == -1 ? radius : length) {

  if (2 * radius != length) scale = Vector3D(1, 1, 2 * radius / length);
  radius2 = radius * radius;
}


void SpheroidSweep::getBBoxes(const Vector3D &start,
                              const Vector3D &end,
                              vector<Rectangle3D> &bboxes,
                              double tolerance) const {
  Sweep::getBBoxes(start, end, bboxes, radius, length, -radius, tolerance);
}


namespace {
  inline double sqr(double x) {return x * x;}
}


double SpheroidSweep::depth(const Vector3D &_A, const Vector3D &_B, // depth方法：接受三个三维向量作为参数。这个方法用来计算空间中的一点到工具扫过的表面最近的距离的平方，如果这个点在表面内部，则返回正值，否则返回负值。这个方法做了以下步骤：
                            const Vector3D &_P) const {
  const double r = radius;
//首先，将参数向量赋值给局部变量A、B、P，分别表示工具移动的起始点、终止点和空间中的一点。
  Vector3D A = _A;
  Vector3D B = _B;
  Vector3D P = _P;

  // Handle oblong spheroids by scaling the z-axis
  // 然后，判断工具是否是一个椭球形。如果是，则将A、B、P乘以scale，以达到椭球形的效果。
  if (2 * radius != length) {
    A *= scale;
    B *= scale;
    P *= scale;
  }

  // Check z-height
  // 接着，判断P的z坐标是否在A和B的z坐标之间。如果不是，则返回-1，表示没有碰撞。
  if (P.z() < min(A.z(), B.z()) || max(A.z(), B.z()) + 2 * r < P.z())
    return -1;
// 然后，计算AB、PA等向量，并根据一些公式计算出epsilon、gamma、rho、sigma等变量。这些变量用来求解二次方程，得到碰撞点在AB线段上的比例beta。
  const Vector3D AB = B - A;
  const Vector3D PA = A - P;

  // epsilon * beta^2 + gamma * beta + rho = 0
  const double epsilon = AB.dot(AB);
  const double gamma = AB.dot(PA + Vector3D(0, 0, r));
  const double rho = PA.dot(PA) + 2 * r * (A.z() - P.z());
  const double sigma = sqr(gamma) - epsilon * rho;

  // Check if solution is valid
  // 接着，判断方程是否有解，并且解是否在0到1之间。如果不是，则返回-1，表示没有碰撞。
  if (epsilon == 0 || sigma < 0) return -1;

  const double beta = (-gamma - sqrt(sigma)) / epsilon; // Quadradic equation

  // Check that it's on the line segment
  if (beta < 0 || 1 < beta) return -1;
 // 最后，返回1，表示有碰撞。
  return 1;
}

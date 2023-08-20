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

#include "ConicSweep.h"

#include <cbang/log/Logger.h>

#include <limits>

using namespace std;
using namespace cb;
using namespace CAMotics;

// 实例化圆锥曲面。length为圆锥形高度，radius1为圆锥顶部半径，radius2为圆锥底部半径。若radius2=-1，则按radius1，即圆柱面。
ConicSweep::ConicSweep(double length, double radius1, double radius2) :
  l(length), rt(radius1), rb(radius2 == -1 ? radius1 : radius2),
  Tm((rt - rb) / l) {
}

// 获得圆锥曲面的外边框。start为边框起点，end为边框终点。bboxes为一系列边框列表。tolerance为每次误差容忍值。
void ConicSweep::getBBoxes(const Vector3D &start, const Vector3D &end,
                           vector<Rectangle3D> &bboxes,
                           double tolerance) const {
  Sweep::getBBoxes(start, end, bboxes, rt < rb ? rb : rt, l, 0, tolerance);
}


namespace {
  inline double sqr(double x) {return x * x;} // 计算平方
}

// 计算一个空间中的点P到一个圆锥曲面的深度。这个圆锥曲面是由一个线段AB和一个圆锥形工具扫过而成的，其中线段AB的起点为A，终点为B，圆锥形工具的高度为l，顶部半径为rt，底部半径为rb。这个函数的过程如下：
double ConicSweep::depth(const Vector3D &A, const Vector3D &B,
                         const Vector3D &P) const {
  const double Ax = A.x(), Ay = A.y(), Az = A.z();
  const double Bx = B.x(), By = B.y(), Bz = B.z();
  const double Px = P.x(), Py = P.y(), Pz = P.z();

  // Check z-height
  // 首先，检查点P的z坐标是否在圆锥曲面的范围内，即是否在min(Az, Bz)和max(Az, Bz) + l之间，其中Az和Bz分别表示A和B的z坐标。如果不在范围内，则返回-1，表示点P不与圆锥曲面相交。
  if (Pz < min(Az, Bz) || max(Az, Bz) + l < Pz) return -1;

  // epsilon * beta^2 + gamma * beta + rho = 0
  // 然后，计算一个参数epsilon，它表示线段AB在xy平面上的投影与圆锥形工具在xy平面上的投影之间的关系。如果epsilon等于0，则说明线段AB与z轴平行，并且圆锥形工具是一个圆柱形。如果epsilon不等于0，则说明线段AB与z轴不平行，并且圆锥形工具是一个真正的圆锥形。
  double epsilon = sqr(Bx - Ax) + sqr(By - Ay) - sqr(Tm * (Bz - Az));

  // If this is a straight up and down move of a cylindrical tool choose
  // a fake arbitrarily small epsilon.
  // TODO improve cylindrical computation speed by computing special case
  if (epsilon == 0 && Bz != Az && Tm == 0) epsilon = 0.000000001;

  // 接着，计算另外两个参数gamma和rho，它们表示点P与线段AB之间的关系。gamma是点P到线段AB的垂直距离的平方，rho是点P到线段AB的起点A的距离的平方。
  const double gamma = (Ax - Px) * (Bx - Ax) + (Ay - Py) * (By - Ay) +
    (sqr(Tm) * (Az - Pz) - Tm * rb) * (Az - Bz);
  const double rho = sqr(Ax - Px) + sqr(Ay - Py) - sqr(Tm * (Az - Pz) - rb);
  // 然后，计算一个参数sigma，它表示点P是否在圆锥曲面上或内部。sigma等于gamma的平方减去epsilon乘以rho。如果sigma小于0，则说明点P不在圆锥曲面上或内部，返回-1。如果sigma大于等于0，则说明点P可能在圆锥曲面上或内部。
  const double sigma = sqr(gamma) - epsilon * rho;

  // Check if solution is valid
  if (epsilon == 0 || sigma < 0) return -1;

  // 接着，计算一个参数beta，它表示点P在线段AB上的投影所对应的比例。beta等于负gamma减去sigma的平方根除以epsilon。这是一个二次方程的解。
  const double beta = (-gamma - sqrt(sigma)) / epsilon; // Quadradic equation

  // Check if z-heights make sense
  // 接着，根据beta计算出点P在线段AB上的投影Q的z坐标Qz。Qz等于Bz减去Az乘以beta再加上Az。
  const double Qz = (Bz - Az) * beta + Az;

  // Contact point is outside of z-height range.
  // 然后，检查Qz是否在Pz和Pz + l之间，即是否在圆锥曲面的高度范围内。如果不在，则说明点P不与圆锥曲面相交，并且需要进一步检查点P是否与圆锥形工具的顶部或底部盘相交。
  if (Pz < Qz || Qz + l < Pz) {
    // Check if point is cut by bottom disc
    if (rb) { // 如果rb不等于0，则检查点P是否与底部盘相交。首先计算一个参数beta，它表示点P在线段AB上与底部盘所在平面相交的投影所对应的比例。beta等于Pz减去Az除以Bz减去Az。
        const double beta = (Pz - Az) / (Bz - Az); // E is on AB at z-height

        // 然后判断beta是否在0和1之间，即是否在线段AB上。如果是，则计算出相交点E在xy平面上的坐标Ex和Ey，并计算出E到P在xy平面上的距离的平方d2。如果d2小于等于rb的平方，则说明点P与底部盘相交，并返回1。
      if (0 <= beta && beta <= 1) {
        // Compute squared distance to E on XY plane
        const double Ex = beta * (Bx - Ax) + Ax;
        const double Ey = beta * (By - Ay) + Ay;
        const double d2 = sqr(Ex - Px) + sqr(Ey - Py);

        if (d2 <= rb * rb) return 1;
      }
    }

    // Check if point is cut by top disc
    if (rt) { // 如果rt不等于0，则检查点P是否与顶部盘相交。首先计算一个参数beta，它表示点P在线段AB上与顶部盘所在平面相交的投影所对应的比例。beta等于Pz减去Az减去l除以Bz减去Az。
      const double beta = (Pz - Az - l) / (Bz - Az); // E is on AB at z-height

      if (0 <= beta && beta <= 1) { // 然后判断beta是否在0和1之间，即是否在线段AB上。如果是，则计算出相交点E在xy平面上的坐标Ex和Ey，并计算出E到P在xy平面上的距离的平方d2。如果d2小于等于rt的平方，则说明点P与顶部盘相交，并返回1。
        // Compute squared distance to E on XY plane
        const double Ex = beta * (Bx - Ax) + Ax;
        const double Ey = beta * (By - Ay) + Ay;
        const double d2 = sqr(Ex - Px) + sqr(Ey - Py);

        if (d2 <= rt * rt) return 1;
      }
    }

    return -1; // 如果以上都不满足，则返回-1，表示点P不与圆锥曲面相交。
  }

  // Check that it's on the line segment
  // 然后，检查beta是否在0和1之间，即是否在线段AB上。如果不在，则返回-1，表示点P不在圆锥曲面上或内部。
  if (beta < 0 || 1 < beta) return -1;

  return 1;
}

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


#include <camotics/contour/FieldFunction.h>


namespace CAMotics {
  class Workpiece : public cb::Rectangle3D, public FieldFunction { // 表示一个工件。这个类继承了两个父类，分别是cb::Rectangle3D和FieldFunction。cb::Rectangle3D类表示一个三维矩形，FieldFunction类表示一个场函数，用来计算空间中的点到工件表面的距离。这个类有以下特点：
    cb::Vector3D center;
    cb::Vector3D halfDim2;

  public:
    Workpiece(const cb::Rectangle3D &r = cb::Rectangle3D()); // 构造函数，接受一个cb::Rectangle3D对象作为参数，用来初始化工件的位置和尺寸，并计算工件的中心点和半径的平方。

    cb::Rectangle3D getBounds() const {return *this;} // getBounds方法，返回工件的边界矩形，即父类cb::Rectangle3D本身。
    bool isValid() const {return getVolume();} // isValid方法，判断工件是否有效，即是否有体积。
    using cb::Rectangle3D::contains; // contains方法，判断一个点是否在工件内部，即调用父类cb::Rectangle3D的contains方法。

    // From FieldFunction
    double depth(const cb::Vector3D &p) const; // depth方法，接受一个cb::Vector3D对象作为参数，表示一个空间中的点。这个方法返回这个点到工件表面最近的距离的平方，如果这个点在工件内部，则返回正值，否则返回负值。这个方法实现了父类FieldFunction的纯虚函数。
  };
}

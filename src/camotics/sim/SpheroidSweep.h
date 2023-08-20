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

namespace CAMotics {
  class SpheroidSweep : public Sweep { // 表示一个椭球形的工具扫过的形状。这个类继承了Sweep类，表示一个抽象的扫过形状，用来模拟切割过程。这个类有以下特点：
    double radius; // radius成员变量，是一个双精度浮点数。它表示工具的半径，单位是米。
    double length; // length成员变量，是一个双精度浮点数。它表示工具的长度，单位是米。如果长度为负数，则表示工具是一个球形，否则表示工具是一个椭球形。

    cb::Vector3D scale; // scale成员变量，是一个三维向量。它表示工具的缩放比例，用来将球形变为椭球形。
    double radius2; // radius2成员变量，是一个双精度浮点数。它表示工具的半径的平方，用来计算距离。

  public:
    SpheroidSweep(double radius, double length = -1); // 构造函数，接受两个双精度浮点数作为参数，分别表示工具的半径和长度。这个函数用来初始化radius和length，并根据长度计算scale和radius2。

    // From Sweep
    void getBBoxes(const cb::Vector3D &start, const cb::Vector3D &end, // getBBoxes方法，重写了父类Sweep的纯虚函数。这个方法接受两个三维向量和一个向量的引用作为参数。这个方法用来根据工具从起始点到终止点的移动，计算出一系列包围盒，并将它们存储到参数向量中。包围盒是一种用来描述物体空间范围的矩形结构。
                   std::vector<cb::Rectangle3D> &bboxes,
                   double tolerance) const;
    double depth(const cb::Vector3D &start, const cb::Vector3D &end, // depth方法，重写了父类Sweep的纯虚函数。这个方法接受三个三维向量作为参数。这个方法用来计算空间中的一点到工具扫过的表面最近的距离的平方，如果这个点在表面内部，则返回正值，否则返回负值。

                 const cb::Vector3D &p) const;
  };
}

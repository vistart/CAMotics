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


#include "Workpiece.h"
#include "ToolSweep.h"

#include <camotics/contour/FieldFunction.h>

#include <cbang/SmartPointer.h>


namespace CAMotics {
  class CutWorkpiece : public FieldFunction  // 表示一个被切割的工件。这个类继承了FieldFunction类，表示一个场函数，用来计算空间中的点到工件表面的距离。这个类有以下特点：
    cb::SmartPointer<ToolSweep> toolSweep; // toolSweep成员变量，是一个ToolSweep对象的智能指针。ToolSweep对象表示一个工具扫过的形状，用来模拟切割过程。
    Workpiece workpiece; // workpiece成员变量，是一个Workpiece对象。Workpiece对象表示一个原始的工件，是一个三维矩形。

  public:
    CutWorkpiece(const cb::SmartPointer<ToolSweep> &toolSweep, // 构造函数，接受一个ToolSweep对象的智能指针和一个Workpiece对象作为参数，用来初始化toolSweep和workpiece。
                 const Workpiece &workpiece);

    const cb::SmartPointer<ToolSweep> &getToolSweep() const {return toolSweep;} // getToolSweep方法，返回toolSweep的常量引用。
    const Workpiece &getWorkpiece() const {return workpiece;} // getWorkpiece方法，返回workpiece的常量引用。

    bool isValid() const; // getWorkpiece方法，返回workpiece的常量引用。
    cb::Rectangle3D getBounds() const; // getBounds方法，返回工件的边界矩形。

    // From FieldFunction
    bool cull(const cb::Rectangle3D &r) const; // cull方法，重写了父类FieldFunction的虚函数，接受一个矩形作为参数，判断它是否与工件不相交。如果不相交，则返回true，表示可以剪除这个区域，提高计算效率。
    double depth(const cb::Vector3D &p) const; // depth方法，重写了父类FieldFunction的虚函数，接受一个三维向量作为参数，表示一个空间中的点。这个方法返回这个点到工件表面最近的距离的平方，如果这个点在工件内部，则返回正值，否则返回负值。
  };
}

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

#include "AABBTree.h"
#include <gcode/ToolPath.h>

#include <camotics/contour/FieldFunction.h>

#include <cbang/SmartPointer.h>

#include <vector>
#include <limits>
#include <cinttypes>


namespace GCode {class ToolTable;}

namespace CAMotics {
  class Sweep;

  class ToolSweep : public FieldFunction, public AABBTree { // 表示一个工具扫过的形状和移动的查找器。这个类继承了FieldFunction类和AABBTree类，分别表示一个空间中的场函数和一个轴对齐的包围盒树。这个类有以下特点：
    cb::SmartPointer<GCode::ToolPath> path; // path成员变量，是一个GCode::ToolPath对象的智能指针。GCode::ToolPath对象表示一个工具路径，包含了一系列的移动指令和工具信息。
    std::vector<cb::SmartPointer<Sweep> > sweeps; // sweeps成员变量，是一个Sweep对象的智能指针的向量。Sweep对象表示一个抽象的扫过形状，用来模拟切割过程。这个向量根据path中的工具编号和工具表创建不同类型的Sweep对象。

    double startTime = 0; // startTime成员变量，是一个双精度浮点数。它表示工具路径的起始时间，单位是秒。
    double endTime = 0; // endTime成员变量，是一个双精度浮点数。它表示工具路径的结束时间，单位是秒。

    cb::SmartPointer<MoveLookup> change; // change成员变量，是一个MoveLookup对象的智能指针。MoveLookup对象表示一个存储和查询GCode::Move对象的结构，表示G代码中的移动指令。这个对象用来表示工具路径在某个时间段内的变化。

  public:
    ToolSweep(const cb::SmartPointer<GCode::ToolPath> &path, // 构造函数，接受一个GCode::ToolPath对象的智能指针和两个双精度浮点数作为参数，分别表示工具路径、起始时间和结束时间。这个函数用来初始化path、startTime、endTime，并根据path中的工具编号和工具表创建sweeps向量，并将其添加到AABBTree中。
              double startTime = 0,
              double endTime = std::numeric_limits<double>::max());
// 两个set方法，分别用来设置startTime和endTime。
    void setStartTime(double startTime) {this->startTime = startTime;}
    void setEndTime(double endTime) {this->endTime = endTime;}
// 两个get方法，分别用来获取change和sweeps。
    const cb::SmartPointer<MoveLookup> &getChange() const {return change;}
    void setChange(const cb::SmartPointer<MoveLookup> &change)
    {this->change = change;}

    // From FieldFunction
    bool cull(const cb::Rectangle3D &r) const; // cull方法，重写了父类FieldFunction的纯虚函数。这个方法接受一个矩形作为参数，表示空间中的一个区域。这个方法用来判断该区域是否与工具扫过的形状相交，如果不相交，则返回true，否则返回false。
    double depth(const cb::Vector3D &p) const; // depth方法，重写了父类FieldFunction的纯虚函数。这个方法接受一个三维向量作为参数，表示空间中的一点。这个方法用来计算该点到工具扫过的表面最近的距离的平方，如果该点在表面内部，则返回正值，否则返回负值。

    static cb::SmartPointer<Sweep> getSweep(const GCode::Tool &tool); // 静态方法getSweep，接受一个GCode::Tool对象作为参数。这个方法用来根据工具的形状创建并返回不同类型的Sweep对象。
  };
}

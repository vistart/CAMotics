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

#include <gcode/ToolTable.h>
#include <gcode/ToolPath.h>
#include <gcode/plan/PlannerConfig.h>

#include <camotics/render/RenderMode.h>
#include <camotics/contour/Surface.h>

#include <cbang/SmartPointer.h>
#include <cbang/json/Serializable.h>

#include <string>


namespace cb {namespace JSON {class Sink;}}

namespace CAMotics {
  class Workpiece;

  class Simulation : public cb::JSON::Serializable { // 表示一个切割模拟的参数和结果。这个类继承了cb::JSON::Serializable类，表示一个可以序列化和反序列化为JSON格式的对象。这个类有以下特点：

  public:
    cb::SmartPointer<GCode::ToolPath> path; // path成员变量，是一个GCode::ToolPath对象的智能指针。GCode::ToolPath对象表示一个G代码的工具路径，包含了一系列的移动指令和工具信息。
    cb::SmartPointer<GCode::PlannerConfig> planConf; // planConf成员变量，是一个GCode::PlannerConfig对象的智能指针。GCode::PlannerConfig对象表示一个G代码的规划器配置，包含了一些控制移动速度和加速度的参数。
    cb::SmartPointer<Surface> surface; // surface成员变量，是一个Surface对象的智能指针。Surface对象表示一个三维的表面，由一组顶点和三角形组成。

      Workpiece workpiece; // workpiece成员变量，是一个Workpiece对象。Workpiece对象表示一个原始的工件，是一个三维矩形。
    double resolution; // resolution成员变量，是一个双精度浮点数。它表示模拟的分辨率，单位是米。
    double time; // time成员变量，是一个双精度浮点数。它表示模拟的时间，单位是秒。
    RenderMode mode; // mode成员变量，是一个RenderMode枚举类型。它表示模拟的渲染模式，可以是实体模式、线框模式或者混合模式。
    unsigned threads; // threads成员变量，是一个无符号整数。它表示模拟使用的线程数量。

    Simulation(const cb::SmartPointer<GCode::ToolPath> &path,
               const cb::SmartPointer<GCode::PlannerConfig> &planConf,
               const cb::SmartPointer<Surface> &surface,
               const Workpiece &workpiece, double resolution, double time,
               RenderMode mode, unsigned threads) : // 构造函数，接受以上所有成员变量作为参数，用来初始化它们。
      path(path), planConf(planConf), surface(surface), workpiece(workpiece),
      resolution(resolution), time(time), mode(mode), threads(threads) {}
    ~Simulation(); // 析构函数，释放内存。

    const GCode::ToolTable &getTools() const {return path->getTools();} // getTools方法，返回path中的工具表的常量引用。工具表是一个存储了工具编号和工具形状的映射表。

    std::string computeHash() const; // computeHash方法，返回模拟的哈希值。哈希值是一种用来标识和比较数据的字符串，通常由一些数字和字母组成。

    // From JSON::Serializable
    // read和write方法，重写了父类cb::JSON::Serializable的虚函数。这两个方法用来将模拟对象序列化和反序列化为JSON格式。JSON格式是一种轻量级的数据交换格式，通常由一些键值对组成。
    using cb::JSON::Serializable::read;
    using cb::JSON::Serializable::write;
    void read(const cb::JSON::Value &value);
    void write(cb::JSON::Sink &sink) const;
  };
}

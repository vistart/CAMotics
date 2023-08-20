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

#include <cbang/SmartPointer.h>


namespace GCode {class ToolPath;}

namespace CAMotics {
  namespace Project {class Project;}
  class Surface;
  class Simulation;
  class Task;


  class CutSim { // 表示一个切割模拟器。这个类用来计算GCode的工具路径和表面，并对表面进行简化。这个类有以下特点：
    cb::SmartPointer<Task> task; // 表示一个切割模拟器。这个类用来计算GCode的工具路径和表面，并对表面进行简化。这个类有以下特点：

  public:
    CutSim(); // 构造函数，初始化task为空。
    ~CutSim(); // 析构函数，释放task指向的内存。

    cb::SmartPointer<GCode::ToolPath>
    computeToolPath(const Project::Project &project); // computeToolPath方法，接受一个Project对象作为参数，返回一个GCode::ToolPath对象的智能指针。这个方法用来根据项目的设置和文件，计算出GCode的工具路径。

      cb::SmartPointer<Surface> computeSurface(const Simulation &sim); // computeSurface方法，接受一个Simulation对象作为参数，返回一个Surface对象的智能指针。这个方法用来根据模拟的参数和工具路径，计算出切割后的表面。
    void reduceSurface(const cb::SmartPointer<Surface> &surface); // reduceSurface方法，接受一个Surface对象的智能指针作为参数。这个方法用来对表面进行简化，减少顶点和三角形的数量，提高渲染效率。

    void interrupt() // interrupt方法，用来中断当前正在执行的任务。
  };
}

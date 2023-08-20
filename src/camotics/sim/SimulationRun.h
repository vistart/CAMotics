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


#include "Simulation.h"

#include <cbang/SmartPointer.h>


namespace CAMotics {
  class ToolSweep;
  class GridTree;
  class Surface;
  class MoveLookup;
  class Task;


  class SimulationRun { // 表示一个切割模拟的运行过程。这个类用来根据一个Simulation对象的参数，计算出一个Surface对象的结果。这个类有以下特点：
    Simulation sim; // sim成员变量，是一个Simulation对象。Simulation对象表示一个切割模拟的参数和结果，用来序列化和反序列化为JSON格式。
    cb::SmartPointer<ToolSweep> sweep; // sweep成员变量，是一个ToolSweep对象的智能指针。ToolSweep对象表示一个工具扫过的形状，用来模拟切割过程。
    cb::SmartPointer<GridTree> tree; // tree成员变量，是一个GridTree对象的智能指针。GridTree对象表示一个网格树，用来存储和查询表面的数据。

    double lastTime = 0; // lastTime成员变量，是一个双精度浮点数。它表示模拟的最后一次更新的时间，单位是秒。

  public:
    SimulationRun(const Simulation &sim); // 构造函数，接受一个Simulation对象作为参数，用来初始化sim，并根据sim中的工具路径和工具形状创建sweep。
    ~SimulationRun(); // 析构函数，释放内存。

    Simulation &getSimulation() {return sim;} // getSimulation方法，返回sim的引用。

    cb::SmartPointer<MoveLookup> getMoveLookup() const; // getMoveLookup方法，返回sweep中的移动查找器的智能指针。移动查找器是一个存储和查询GCode::Move对象的结构，表示G代码中的移动指令。

    void setEndTime(double endTime); // setEndTime方法，接受一个双精度浮点数作为参数，表示模拟的结束时间。这个方法用来设置sim中的时间，并根据时间调整sweep中的移动查找器。

    cb::SmartPointer<Surface> compute(Task &task); // compute方法，接受一个Task对象作为参数。这个方法用来根据sweep和workpiece计算出表面，并返回一个Surface对象的智能指针。这个方法会创建并更新tree，并调用其compute方法进行计算，并传入task作为参数。Task对象表示一个异步的任务，用来执行模拟的计算。
  };
}

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


#include <camotics/Task.h>

#include <cbang/SmartPointer.h>


namespace CAMotics {
  class Simulation;
  class SimulationRun;
  class Surface;


  class SurfaceTask : public Task { // 表示一个计算表面的任务。这个类继承了Task类，表示一个异步的任务，用来执行模拟的计算。这个类有以下特点：
    cb::SmartPointer<SimulationRun> simRun; // 一个simRun成员变量，是一个SimulationRun对象的智能指针。SimulationRun对象表示一个切割模拟的运行过程，用来根据一个Simulation对象的参数，计算出一个Surface对象的结果。
    cb::SmartPointer<Surface> surface; // 一个surface成员变量，是一个Surface对象的智能指针。Surface对象表示一个三维的表面，由一组顶点和三角形组成。

  public:
    SurfaceTask(const Simulation &sim); // 两个构造函数，分别接受一个Simulation对象和一个SimulationRun对象的智能指针作为参数。这两个函数用来创建并初始化simRun，并根据参数选择不同的方式。如果传入的是Simulation对象，则创建一个新的SimulationRun对象并赋值给simRun。如果传入的是SimulationRun对象，则直接赋值给simRun。
    SurfaceTask(const cb::SmartPointer<SimulationRun> &simRun);
    ~SurfaceTask(); // 析构函数，释放内存。

    const cb::SmartPointer<SimulationRun> &getSimRun() const {return simRun;} // getSimRun方法，返回simRun的常量引用。
    const cb::SmartPointer<Surface> &getSurface() const {return surface;} // getSurface方法，返回surface的常量引用。

    // From Task
    void run(); // run方法，重写了父类Task的虚函数。这个方法用来调用simRun的compute方法，并将其返回值赋值给surface。
  };
}

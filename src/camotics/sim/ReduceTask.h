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
  class Surface;

  class ReduceTask : public Task { // 表示一个简化表面的任务。这个类继承了Task类，表示一个异步的任务，用来执行模拟的计算。这个类有以下特点：
    cb::SmartPointer<Surface> surface; // surface成员变量，是一个Surface对象的智能指针。Surface对象表示一个三维的表面，由一组顶点和三角形组成。

  public:
    ReduceTask(const cb::SmartPointer<Surface> &surface); // 构造函数，接受一个Surface对象的智能指针作为参数，用来初始化surface。

    const cb::SmartPointer<Surface> &getSurface() const {return surface; // getSurface方法，返回surface的常量引用。

    // From Task
    void run(); // run方法，重写了父类Task的虚函数。这个方法用来对surface进行简化，减少顶点和三角形的数量，提高渲染效率。这个方法会调用surface的reduce方法进行简化。
  };
}

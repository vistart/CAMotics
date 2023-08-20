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

#include "ReduceTask.h"

#include <camotics/contour/Surface.h>

#include <cbang/Catch.h>
#include <cbang/time/Timer.h>
#include <cbang/time/TimeInterval.h>
#include <cbang/log/Logger.h>

using namespace std;
using namespace cb;
using namespace CAMotics;

// 实现了ReduceTask类的成员函数。这个类表示一个简化表面的任务，用来对表面进行简化，减少顶点和三角形的数量，提高渲染效率。这个类继承了Task类，表示一个异步的任务，用来执行模拟的计算。这个类的成员函数有以下功能：
ReduceTask::ReduceTask(const SmartPointer<Surface> &surface) : // 构造函数：接受一个Surface对象的智能指针作为参数，用来初始化surface。Surface对象表示一个三维的表面，由一组顶点和三角形组成。
  surface(surface) {}


void ReduceTask::run() { // run函数：重写了父类Task的虚函数。这个函数用来对surface进行简化，并记录简化的时间和效果。这个函数做了以下步骤：
  LOG_INFO(1, "Reducing mesh"); // 打印一条日志信息，表示开始简化网格。

  double startTime = Timer::now(); // 获取当前的时间和表面的三角形数量，作为简化前的数据。
  double startCount = surface->getTriangleCount(); // 调用surface的reduce方法进行简化，传入自身作为参数。这个方法会根据一些算法和条件，删除一些不必要或冗余的顶点和三角形，同时保持表面的形状和质量。

  surface->reduce(*this);

  unsigned count = surface->getTriangleCount(); // 获取简化后的表面的三角形数量，并计算出简化的百分比。
  double r = (double)(startCount - count) / startCount * 100;

  LOG_INFO(1, "Time: " << TimeInterval(Timer::now() - startTime // 打印一条日志信息，表示结束简化网格，并显示简化所花费的时间，以及表面的三角形数量和简化百分比。
           << String::printf(" Triangles: %u Reduction: %0.2f%%", count, r));
}

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

#include "SurfaceTask.h"

#include <camotics/sim/SimulationRun.h>
#include <camotics/contour/Surface.h>

#include <cbang/String.h>
#include <cbang/time/Timer.h>
#include <cbang/time/TimeInterval.h>
#include <cbang/log/Logger.h>

using namespace cb;
using namespace CAMotics;

SurfaceTask::SurfaceTask(const Simulation &sim) :
  simRun(new SimulationRun(sim)) {}


SurfaceTask::SurfaceTask(const SmartPointer<SimulationRun> &simRun) :
  simRun(simRun) {}


SurfaceTask::~SurfaceTask() {}


// 实现了SurfaceTask类的run方法。这个类表示一个计算表面的任务，用来根据一个SimulationRun对象的参数和结果，生成一个Surface对象。这个方法接受一个Task对象作为参数，表示一个异步的任务，用来执行模拟的计算。这个方法的流程如下：
void SurfaceTask::run() {
  double startTime = Timer::now(); // 首先，获取当前的时间，并将其赋值给startTime，作为计算开始的时间。

  surface = simRun->compute(*this); // 然后，调用simRun的compute方法，并将其返回值赋值给surface。simRun是一个SimulationRun对象，表示一个切割模拟的运行过程，用来根据一个Simulation对象的参数，计算出一个Surface对象的结果。compute方法接受一个Task对象作为参数，并返回一个Surface对象的智能指针。Surface对象表示一个三维的表面，由一组顶点和三角形组成。

  // Time
  if (shouldQuit()) { // 接着，判断是否应该退出任务。如果是，则打印一条日志信息，表示渲染被中断，并返回。
    LOG_INFO(1, "Render aborted");
    return;
  }

  // Done
  double delta = Timer::now() - startTime; // 如果不是，则获取当前的时间，并减去startTime，得到计算所花费的时间。然后获取surface中的三角形数量，并计算出每秒生成的三角形数量。最后打印一条日志信息，表示计算结束，并显示计算所花费的时间、三角形数量和每秒生成的三角形数量。
  unsigned triangles = surface->getTriangleCount();
  LOG_INFO(1, "Time: " << TimeInterval(delta)
           << " Triangles: " << triangles
           << " Triangles/sec: " << String::printf("%0.2f", triangles / delta));
}

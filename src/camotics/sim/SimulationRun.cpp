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

#include "SimulationRun.h"
#include "Simulation.h"

#include <camotics/contour/TriangleSurface.h>
#include <camotics/contour/GridTree.h>
#include <camotics/render/Renderer.h>
#include <camotics/sim/CutWorkpiece.h>

#include <cbang/log/Logger.h>
#include <cbang/time/TimeInterval.h>
#include <cbang/time/Timer.h>

using namespace cb;
using namespace CAMotics;


SimulationRun::SimulationRun(const Simulation &sim) : sim(sim), lastTime(-1) {}


SimulationRun::~SimulationRun() {}


SmartPointer<MoveLookup> SimulationRun::getMoveLookup() const {
  if (!sweep.isNull() && !sweep->getChange().isNull())
    return sweep->getChange();
  return sweep;
}


void SimulationRun::setEndTime(double endTime) {sim.time = endTime;}


SmartPointer<Surface> SimulationRun::compute(Task &task) { // 这个方法接受一个Task对象作为参数，表示一个异步的任务，用来执行模拟的计算。这个方法的具体流程如下：
  Rectangle3D bbox; // 首先，声明一个矩形变量bbox，用来存储模拟的边界。

  double start = Timer::now(); // 然后，获取当前的时间和模拟的时间，并取其中较小的一个作为模拟的结束时间。打印一条日志信息，表示开始计算表面。
  double simTime = std::min(sim.path->getTime(), sim.time);

  LOG_INFO(1, "Computing surface at " << TimeInterval(simTime));

  // Build full sweep once OR for each file
  if (sweep.isNull()) { // 接着，判断sweep是否为空。如果为空，则说明是第一次进行模拟，需要创建一个ToolSweep对象，并将其赋值给sweep。ToolSweep对象表示一个工具扫过的形状，用来模拟切割过程。这个对象根据sim中的工具路径创建，并覆盖整个时间段。然后，根据sim中的工件获取其边界，并将其扩大一点作为bbox。接着，创建一个GridTree对象，并将其赋值给tree。GridTree对象表示一个网格树，用来存储和查询表面的数据。这个对象根据bbox和sim中的分辨率创建一个网格。
    // GCode::Tool sweep
    sweep = new ToolSweep(sim.path); // Build sweep for entire time period

    // Bounds, increased a little
    bbox = sim.workpiece.getBounds().grow(sim.resolution * 0.9);

    // Grid
    tree = new GridTree(Grid(bbox, sim.resolution));

  } else { // 如果sweep不为空，则说明是继续进行模拟，需要更新sweep中的移动查找器。移动查找器是一个存储和查询GCode::Move对象的结构，表示G代码中的移动指令。首先计算出最小和最大的时间，分别表示模拟的起始和结束时间。然后创建一个ToolSweep对象，并将其赋值给change。这个对象根据sim中的工具路径和最小和最大时间创建，并只覆盖这个时间段。然后调用sweep的setChange方法，将change设置为sweep中的移动查找器。接着，根据change获取其边界，并将其扩大一点作为bbox。
    double minTime = simTime;
    double maxTime = simTime;

    if (lastTime < minTime) minTime = lastTime;
    if (maxTime < lastTime) maxTime = lastTime;

    SmartPointer<MoveLookup> change = new ToolSweep(sim.path, minTime, maxTime);
    sweep->setChange(change);
    bbox = change->getBounds().grow(sim.resolution * 1.1);
  }

  // Set target time
  sweep->setEndTime(simTime); // 然后，调用sweep的setEndTime方法，将模拟的结束时间设置为simTime。

  // Setup cut simulation
  CutWorkpiece cutWP(sweep, sim.workpiece); // 接着，创建一个CutWorkpiece对象，并将其赋值给cutWP。CutWorkpiece对象表示一个被切割的工件，用来计算空间中的点到工件表面的距离。这个对象根据sweep和sim中的工件创建。

  // Render
  Renderer renderer(task); // 然后，创建一个Renderer对象，并将其赋值给renderer。Renderer对象用来渲染cutWP到tree中，并传入task作为参数。Task对象表示一个异步的任务，用来执行模拟的计算。
  renderer.render(cutWP, *tree, bbox, sim.threads, sim.mode);

  if (task.shouldQuit()) { // 接着，判断task是否应该退出。如果是，则释放sweep和tree，并返回空指针。
    sweep.release();
    tree.release();
    return 0;
  }

  LOG_DEBUG(1, "Render time " << TimeInterval(Timer::now() - start)); // 如果不是，则打印一条日志信息，表示渲染所花费的时间。

  // Extract surface
  lastTime = simTime; //  最后，更新lastTime为simTime，并返回一个TriangleSurface对象的智能指针。TriangleSurface对象表示一个三维的表面，由一组顶点和三角形组成。这个对象根据tree创建。
  return new TriangleSurface(*tree);
}

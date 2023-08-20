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

#include "CutSim.h"

#include "ToolPathTask.h"
#include "SurfaceTask.h"
#include "ReduceTask.h"
#include "AABBTree.h"

using namespace std;
using namespace cb;
using namespace CAMotics;

// 实现了CutSim类的成员函数。这个类表示一个切割模拟器，用来计算GCode的工具路径和表面，并对表面进行简化。这个类的成员函数有以下功能：
CutSim::CutSim() {} // 析构函数：释放task指向的内存。
CutSim::~CutSim() {} // 析构函数：释放task指向的内存。


SmartPointer<GCode::ToolPath>
CutSim::computeToolPath(const Project::Project &project)  //  computeToolPath函数：接受一个Project对象作为参数，返回一个GCode::ToolPath对象的智能指针。这个函数用来根据项目的设置和文件，计算出GCode的工具路径。为了完成这个任务，它创建了一个ToolPathTask对象，并将其赋值给task，并调用其run方法执行计算，并返回其getPath方法得到的结果。

task = new ToolPathTask(project);
  task->run();
  return task.cast<ToolPathTask>()->getPath();
}


SmartPointer<Surface> CutSim::computeSurface(const Simulation &sim) { // computeSurface函数：接受一个Simulation对象作为参数，返回一个Surface对象的智能指针。这个函数用来根据模拟的参数和工具路径，计算出切割后的表面。为了完成这个任务，它创建了一个SurfaceTask对象，并将其赋值给task，并调用其run方法执行计算，并返回其getSurface方法得到的结果。
  task = new SurfaceTask(sim);
  task->run();
  return task.cast<SurfaceTask>()->getSurface();
}



void CutSim::reduceSurface(const SmartPointer<Surface> &surface)  //  reduceSurface函数：接受一个Surface对象的智能指针作为参数。这个函数用来对表面进行简化，减少顶点和三角形的数量，提高渲染效率。为了完成这个任务，它创建了一个ReduceTask对象，并将其赋值给task，并调用其run方法执行简化。
  task = new ReduceTask(surface);
  task->run();
}


void CutSim::interrupt() { //  interrupt函数：用来中断当前正在执行的任务。如果task不为空，则调用其interrupt方法。
  if (!task.isNull()) task->interrupt();
}

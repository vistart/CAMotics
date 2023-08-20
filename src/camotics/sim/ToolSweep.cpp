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

#include "ToolSweep.h"

#include "Sweep.h"
#include "ConicSweep.h"
#include "CompositeSweep.h"
#include "SpheroidSweep.h"

#include <gcode/ToolTable.h>

#include <cbang/log/Logger.h>
#include <cbang/time/TimeInterval.h>

#include <algorithm>

using namespace std;
using namespace cb;
using namespace CAMotics;

// 表示一个工具扫过的形状和移动的查找器，用来模拟切割过程。这个类继承了FieldFunction类和AABBTree类，分别表示一个空间中的场函数和一个轴对齐的包围盒树。这个类的各个方法的流程如下：
ToolSweep::ToolSweep(const SmartPointer<GCode::ToolPath> &path, //  构造函数：接受一个GCode::ToolPath对象的智能指针和两个双精度浮点数作为参数，分别表示工具路径、起始时间和结束时间。这个函数用来初始化path、startTime、endTime，并根据path中的工具编号和工具表创建sweeps向量，并将其添加到AABBTree中。sweeps向量是一个Sweep对象的智能指针的向量，Sweep对象表示一个抽象的扫过形状，用来模拟切割过程。AABBTree是一个轴对齐的包围盒树，用来存储和查询空间中的对象。
                     double startTime, double endTime) :
  path(path), startTime(startTime), endTime(endTime) {

  if (endTime < startTime) {
    swap(startTime, endTime);
    swap(this->startTime, this->endTime);
  }

  unsigned boxes = 0;

  if (!path->empty()) {
    int firstMove = path->find(startTime);
    int lastMove = path->find(endTime);

    if (lastMove == -1) lastMove = path->size() - 1;
    if (firstMove == -1) firstMove = lastMove + 1;

    double duration = path->at(lastMove).getEndTime() - startTime;

    LOG_DEBUG(1, "Times: start=" << TimeInterval(startTime) << " end="
              << TimeInterval(startTime + duration) << " duration="
              << TimeInterval(duration));
    LOG_DEBUG(1, "GCode::Moves: first=" << firstMove << " last=" << lastMove);

    GCode::ToolTable &tools = path->getTools();
    vector<Rectangle3D> bboxes;

    // Gather nodes in a list
    for (int i = firstMove; i <= lastMove; i++) {
      const GCode::Move &move = path->at(i);
      int tool = move.getTool();

      if (tool < 0) continue;
      if (sweeps.size() <= (unsigned)tool) sweeps.resize(tool + 1);
      if (sweeps[tool].isNull()) sweeps[tool] = getSweep(tools.get(tool));

      Vector3D startPt = move.getPtAtTime(startTime);
      Vector3D endPt = move.getPtAtTime(endTime);

      sweeps[tool]->getBBoxes(startPt, endPt, bboxes);

      for (unsigned j = 0; j < bboxes.size(); j++)
        insert(&move, bboxes[j]);

      boxes += bboxes.size();
      bboxes.clear();
    }
  }

  AABBTree::finalize(); // Finalize MoveLookup

  LOG_DEBUG(1, "AABBTree boxes=" << boxes << " height=" << getHeight());
}

// cull方法：重写了父类FieldFunction的纯虚函数。这个方法接受一个矩形作为参数，表示空间中的一个区域。这个方法用来判断该区域是否与工具扫过的形状相交，如果不相交，则返回true，否则返回false。这个方法主要用来优化计算效率，避免不必要的深度计算。
bool ToolSweep::cull(const Rectangle3D &r) const {
  if (change.isNull()) return false;
  return !change->intersects(r);
}


namespace {
  struct move_sort {
    bool operator()(const GCode::Move *a, const GCode::Move *b) const {
      return a->getStartTime() < b->getStartTime();
    }
  };
}

// depth方法：重写了父类FieldFunction的纯虚函数。这个方法接受一个三维向量作为参数，表示空间中的一点。这个方法用来计算该点到工具扫过的表面最近的距离的平方，如果该点在表面内部，则返回正值，否则返回负值。这个方法首先调用AABBTree中的collisions方法，找出与该点相交的移动指令，并按照时间顺序排序。然后遍历每个移动指令，并根据其工具编号和起止点，调用相应的Sweep对象中的depth方法，计算该点到该移动指令对应的扫过形状最近的距离。最后返回最大的距离值。
double ToolSweep::depth(const Vector3D &p) const {
  vector<const GCode::Move *> moves;
  collisions(p, moves);

  // Earlier moves first
  sort(moves.begin(), moves.end(), move_sort());

  double d2 = -numeric_limits<double>::max();

  for (unsigned i = 0; i < moves.size(); i++) {
    const GCode::Move &move = *moves[i];

    if (move.getEndTime() < startTime || endTime < move.getStartTime())
      continue;

    Vector3D startPt = move.getPtAtTime(startTime);
    Vector3D endPt = move.getPtAtTime(endTime);

    double sd2 = sweeps[move.getTool()]->depth(startPt, endPt, p);
    if (0 <= sd2) return sd2; // Approx 5% faster
    if (d2 < sd2) d2 = sd2;
  }

  return d2;
}

// 静态方法getSweep：接受一个GCode::Tool对象作为参数。这个方法用来根据工具的形状创建并返回不同类型的Sweep对象。GCode::Tool对象表示一个工具，包含了编号、形状、半径、长度等信息。Sweep对象有多种子类，如ConicSweep、SpheroidSweep、CompositeSweep等，分别表示圆锥形、椭球形、复合形等扫过形状。
SmartPointer<Sweep> ToolSweep::getSweep(const GCode::Tool &tool) {
  switch (tool.getShape()) {
  case GCode::ToolShape::TS_CYLINDRICAL:
    return new ConicSweep(tool.getLength(), tool.getRadius(), tool.getRadius());

  case GCode::ToolShape::TS_CONICAL:
    return new ConicSweep(tool.getLength(), tool.getRadius(), 0);

  case GCode::ToolShape::TS_BALLNOSE: {
    SmartPointer<CompositeSweep> composite = new CompositeSweep;
    composite->add
      (new SpheroidSweep(tool.getRadius(), 2 * tool.getRadius()), 0);
    composite->add(new ConicSweep(tool.getLength(), tool.getRadius(),
                                  tool.getRadius()), tool.getRadius());
    return composite;
  }

  case GCode::ToolShape::TS_SPHEROID:
    return new SpheroidSweep(tool.getRadius(), tool.getLength());

  case GCode::ToolShape::TS_SNUBNOSE:
    return new ConicSweep(tool.getLength(), tool.getRadius(),
                          tool.getSnubDiameter() / 2);
  }

  THROW("Invalid tool shape " << tool.getShape());
}

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

#include "CutWorkpiece.h"

#include <cbang/Math.h>

using namespace std;
using namespace cb;
using namespace CAMotics;

// 实现了CutWorkpiece类的成员函数。这个类表示一个被切割的工件，用来计算空间中的点到工件表面的距离。这个类继承了FieldFunction类，表示一个场函数。这个类的成员函数有以下功能：
CutWorkpiece::CutWorkpiece(const SmartPointer<ToolSweep> &toolSweep, // 构造函数：接受一个ToolSweep对象的智能指针和一个Workpiece对象作为参数，用来初始化toolSweep和workpiece。ToolSweep对象表示一个工具扫过的形状，用来模拟切割过程。Workpiece对象表示一个原始的工件，是一个三维矩形。
                           const Workpiece &workpiece) :
  toolSweep(toolSweep), workpiece(workpiece) {}


bool CutWorkpiece::isValid() const { // isValid函数：判断工件是否有效，即是否有体积。如果workpiece是有效的，则返回false。否则，检查工件的边界矩形是否有非法的值，如NaN或Inf。如果有，则返回false。否则，返回true。
  if (workpiece.isValid()) return false;

  Rectangle3D bounds = getBounds();
  for (unsigned i = 0; i < 3; i++)
    if (Math::isnan(bounds.getMin()[i]) || Math::isnan(bounds.getMax()[i]) ||
        Math::isinf(bounds.getMin()[i]) || Math::isinf(bounds.getMax()[i]))
      return false;

  return true;
}


Rectangle3D CutWorkpiece::getBounds() const { // getBounds函数：返回工件的边界矩形。如果workpiece是有效的，则返回workpiece的边界矩形。否则，如果toolSweep不为空，则返回toolSweep的边界矩形。
  Rectangle3D bb;
  if (workpiece.isValid()) bb = workpiece.getBounds();
  else if (!toolSweep.isNull()) bb = toolSweep->getBounds();
  return bb;
}


bool CutWorkpiece::cull(const Rectangle3D &r) const { // cull函数：重写了父类FieldFunction的虚函数，接受一个矩形作为参数，判断它是否与工件不相交。如果不相交，则返回true，表示可以剪除这个区域，提高计算效率。这个函数调用了toolSweep的cull函数进行判断。
  return toolSweep->cull(r);
}


double CutWorkpiece::depth(const Vector3D &p) const { // depth函数：重写了父类FieldFunction的虚函数，接受一个三维向量作为参数，表示一个空间中的点。这个函数返回这个点到工件表面最近的距离的平方，如果这个点在工件内部，则返回正值，否则返回负值。如果workpiece是无效的，则直接返回toolSweep的depth函数得到的结果。否则，返回workpiece和toolSweep的depth函数得到的结果中较小的一个。
  if (!workpiece.isValid()) return toolSweep->depth(p);
  return min(workpiece.depth(p), -toolSweep->depth(p));
}

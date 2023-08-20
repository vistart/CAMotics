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

#include "MoveLookup.h"
#include "AABB.h"

#include <gcode/Move.h>

#include <vector>


namespace {class ToolPath;}

namespace CAMotics {

  class AABBTree : public MoveLookup { // 一个AABB树。这个类继承了MoveLookup类，表示一个移动查找器，用来存储和查询GCode::Move对象。GCode::Move对象表示一个G代码中的移动指令，包含了移动的类型、速度、位置等信息。这个类有以下特点：
  protected:
    AABB *root; // root成员变量，指向一个AABB对象，表示AABB树的根节点。AABB对象表示一个轴对齐的边界盒，用来包围一组GCode::Move对象，并存储它们的迭代器范围和子节点。
    bool finalized; // finalized成员变量，表示AABB树是否已经构建完成。

  public:
    AABBTree() : root(0), finalized(false) {} // 构造函数，初始化root为0，finalized为false。
    virtual ~AABBTree(); // 析构函数，释放root指向的内存。

    const AABB *getRoot() const {return root;} // getRoot方法，返回root的常量指针。
    unsigned getHeight() const {return root ? root->getTreeHeight() : 0;} // getHeight方法，返回root的树高度，如果root为空，则返回0。

    // From MoveLookup
    cb::Rectangle3D getBounds() const; // getBounds方法，重写了父类MoveLookup的虚函数，返回AABB树的边界矩形，如果root为空，则返回空矩形。
    void insert(const GCode::Move *move, const cb::Rectangle3D &bbox); // insert方法，重写了父类MoveLookup的虚函数，接受一个GCode::Move对象的指针和一个边界矩形作为参数，将它们插入到AABB树中。如果root为空，则创建一个新的AABB对象作为root，并将参数作为其数据。否则，调用root的insert方法将参数插入到合适的子节点中，并更新root的边界矩形。最后将finalized设为false。
    bool intersects(const cb::Rectangle3D &r) const; // insert方法，重写了父类MoveLookup的虚函数，接受一个GCode::Move对象的指针和一个边界矩形作为参数，将它们插入到AABB树中。如果root为空，则创建一个新的AABB对象作为root，并将参数作为其数据。否则，调用root的insert方法将参数插入到合适的子节点中，并更新root的边界矩形。最后将finalized设为false。
    void collisions(const cb::Vector3D &p, // insert方法，重写了父类MoveLookup的虚函数，接受一个GCode::Move对象的指针和一个边界矩形作为参数，将它们插入到AABB树中。如果root为空，则创建一个新的AABB对象作为root，并将参数作为其数据。否则，调用root的insert方法将参数插入到合适的子节点中，并更新root的边界矩形。最后将finalized设为false。
                    std::vector<const GCode::Move *> &moves) const;
    void finalize(); // insert方法，重写了父类MoveLookup的虚函数，接受一个GCode::Move对象的指针和一个边界矩形作为参数，将它们插入到AABB树中。如果root为空，则创建一个新的AABB对象作为root，并将参数作为其数据。否则，调用root的insert方法将参数插入到合适的子节点中，并更新root的边界矩形。最后将finalized设为false。
  };
}

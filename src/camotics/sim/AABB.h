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

#include <gcode/Move.h>

// 源代码见 https://github.com/CauldronDevelopmentLLC/cbang/blob/master/src/cbang/geom/Rectangle.h
#include <cbang/geom/Rectangle.h>

#include <vector>


namespace CAMotics {
    // AABB 矩形，继承自 cbang 的立方体类。
  class AABB : public cb::Rectangle3D {
    AABB *left; // 左邻
    AABB *right; // 右邻
    const GCode::Move *move; // 移动方向

  public:
    AABB(AABB *nodes); // 构造函数，参数为立方体节点组，亦即
    AABB(const GCode::Move *move, const cb::Rectangle3D &bbox) :
      cb::Rectangle3D(bbox), left(0), right(0), move(move) {}
    ~AABB(); // 析构函数，作用为删除自己的左右邻居。

    const AABB *getLeft() const {return left;} // 获取自己的左邻指针。
    const AABB *getRight() const {return right;} // 获取自己的右邻指针。

    AABB *prepend(AABB *list); // 附加一组矩形为自己的左邻。
    AABB *split(unsigned count); // 切割自己，并返回切割后的一组矩形的指针。参数为切割的目标份数。

    cb::Rectangle3D getBounds() const {return *this;} // 获得边，也就是获得自己的地址。
    const GCode::Move *getMove() const {return move;} //
    bool isLeaf() const {return move;} // 判断自己是否为叶节点，也即是否为分割后的最小矩形。
    unsigned getTreeHeight() const; // 获得分割树高度。

    bool intersects(const cb::Rectangle3D &r); // 求解与另一个矩形是否相交。
    void collisions(const cb::Vector3D &p,
                    std::vector<const GCode::Move *> &moves); // 求解与一组边是否有碰撞。
  };
}

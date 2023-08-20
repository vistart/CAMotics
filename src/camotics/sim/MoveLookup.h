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

#include <cbang/geom/Rectangle.h>


namespace CAMotics {
  class MoveLookup { // 表示一个移动查找器。这个类用来存储和查询GCode::Move对象，表示G代码中的移动指令。这个类有以下特点：
  public:
    virtual ~MoveLookup() { // 虚析构函数，用来释放内存。

    virtual cb::Rectangle3D getBounds() const = 0; // 纯虚函数getBounds，返回移动查找器的边界矩形。
    virtual void insert(const GCode::Move *move, // 纯虚函数insert，接受一个GCode::Move对象的指针和一个边界矩形作为参数，将它们插入到移动查找器中。
                        const cb::Rectangle3D &bbox) = 0;
    virtual bool intersects(const cb::Rectangle3D &r) const = 0; // 纯虚函数intersects，接受一个矩形作为参数，判断它是否与移动查找器相交。
    virtual void collisions(const cb::Vector3D &p, // 纯虚函数collisions，接受一个三维向量和一个GCode::Move对象指针的向量作为参数。这个函数用来查找移动查找器中与参数向量相交的所有GCode::Move对象，并将它们的指针存储到参数向量中。

                            std::vector<const GCode::Move *> &moves) const = 0;
    virtual void finalize() {} // 虚函数finalize，用来对移动查找器进行优化或清理。这个函数默认为空。
  };
}

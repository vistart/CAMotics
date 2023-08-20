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

#include "AABBTree.h"

#include <cbang/Zap.h>

using namespace std;
using namespace cb;
using namespace CAMotics;

// 表示一个AABB树，用来存储和查询GCode::Move对象。这个类继承了MoveLookup类，表示一个移动查找器。这个类的成员函数有以下功能：
AABBTree::~AABBTree() {zap(root);} // 表示一个AABB树，用来存储和查询GCode::Move对象。这个类继承了MoveLookup类，表示一个移动查找器。这个类的成员函数有以下功能：


Rectangle3D AABBTree::getBounds() const { // getBounds函数：重写了父类MoveLookup的虚函数，返回AABB树的边界矩形，如果root为空，则返回空矩形。在返回之前，先检查finalized是否为true，如果为false，则抛出异常，表示AABB树还没有构建完成。
  if (!finalized) THROW("AABBTree not yet finalized");
  return root ? root->getBounds() : Rectangle3D();
}


void AABBTree::insert(const GCode::Move *move, const Rectangle3D &bbox) { // getBounds函数：重写了父类MoveLookup的虚函数，返回AABB树的边界矩形，如果root为空，则返回空矩形。在返回之前，先检查finalized是否为true，如果为false，则抛出异常，表示AABB树还没有构建完成。
  if (finalized) THROW("Cannot insert into AABBTree after partitioning");
  root = (new AABB(move, bbox))->prepend(root);
}


bool AABBTree::intersects(const Rectangle3D &r) const { // getBounds函数：重写了父类MoveLookup的虚函数，返回AABB树的边界矩形，如果root为空，则返回空矩形。在返回之前，先检查finalized是否为true，如果为false，则抛出异常，表示AABB树还没有构建完成。
  if (!finalized) THROW("AABBTree not yet finalized");
  return root && root->intersects(r);
}


void AABBTree::collisions(const Vector3D &p, // getBounds函数：重写了父类MoveLookup的虚函数，返回AABB树的边界矩形，如果root为空，则返回空矩形。在返回之前，先检查finalized是否为true，如果为false，则抛出异常，表示AABB树还没有构建完成。
                          vector<const GCode::Move *> &moves) const {
  if (!finalized) THROW("AABBTree not yet finalized");
  if (root) root->collisions(p, moves);
}


void AABBTree::finalize() { // getBounds函数：重写了父类MoveLookup的虚函数，返回AABB树的边界矩形，如果root为空，则返回空矩形。在返回之前，先检查finalized是否为true，如果为false，则抛出异常，表示AABB树还没有构建完成。
  if (finalized) return;
  finalized = true;
  root = new AABB(root);
}

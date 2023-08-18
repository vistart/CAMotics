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

#include "OctTree.h"

#include <cbang/Zap.h> // zap函数是一个模板函数，它用于释放指针所指向的对象，并将指针设为NULL。

using namespace std;
using namespace cb;
using namespace CAMotics;

// OctNode类的构造函数的实现，它与头文件中的声明一致，没有任何变化。
OctTree::OctNode::OctNode(const Rectangle3D &bounds, unsigned depth) :
  bounds(bounds), depth(depth) {
  for (int i = 0; i < 8; i++) children[i] = 0;
}

// OctNode类的析构函数的实现，它与头文件中的声明一致，没有任何变化。
OctTree::OctNode::~OctNode() {
  for (int i = 0; i < 8; i++)
    if (children[i]) delete children[i];
}

// OctNode类的insert方法的实现，它与头文件中的声明一致，没有任何变化。
// 用于将一个移动对象插入到节点中。移动对象是一个GCode::Move类的指针，它表示三维空间中的一次移动操作。
// 移动对象有一个边界框属性，它是一个cb::Rectangle3D类的对象，它表示移动对象所占据的最小矩形区域。
void OctTree::OctNode::insert(const GCode::Move *move,
                              const Rectangle3D &bbox) {
    // 首先，判断移动对象的边界框是否与节点所代表的空间区域相交。如果不相交，则直接返回，不做任何操作。
  if (!bounds.intersects(bbox)) return;
    // 如果相交，则继续执行以下步骤：
    // 如果节点的深度为0，或者移动对象的边界框完全包含节点所代表的空间区域，则
    // 将移动对象加入到节点所包含的移动对象集合中，并返回。
    // 这表示该节点已经是最底层的节点，或者该节点已经被移动对象完全覆盖，无需再继续划分子节点。
  if (!depth || bbox.contains(bounds)) {
    moves.insert(move);
    return;
  }
    // 如果节点的深度不为0，并且移动对象的边界框不完全包含节点所代表的空间区域，则需要创建或访问节点的八个子节点，并将移动对象插入到相应的子节点中。
  Vector3D dims = bounds.getDimensions();

  static Vector3D cubes[16] = {
    Vector3D(0.0, 0.0, 0.0), Vector3D(0.5, 0.5, 0.5),
    Vector3D(0.5, 0.0, 0.0), Vector3D(1.0, 0.5, 0.5),
    Vector3D(0.0, 0.5, 0.0), Vector3D(0.5, 1.0, 0.5),
    Vector3D(0.5, 0.5, 0.0), Vector3D(1.0, 1.0, 0.5),

    Vector3D(0.0, 0.0, 0.5), Vector3D(0.5, 0.5, 1.0),
    Vector3D(0.5, 0.0, 0.5), Vector3D(1.0, 0.5, 1.0),
    Vector3D(0.0, 0.5, 0.5), Vector3D(0.5, 1.0, 1.0),
    Vector3D(0.5, 0.5, 0.5), Vector3D(1.0, 1.0, 1.0),
  };
    // 为了创建或访问子节点，需要使用一个静态数组cubes来存储八个子区域相对于父区域的比例坐标。
    // 例如，cubes[0]和cubes[1]分别表示第一个子区域在父区域中的最小点和最大点的比例坐标。然后，遍历八个子区域，对于每个子区域：
  for (int i = 0; i < 8; i++) {
    if (!children[i]) {
        // 如果对应的子节点指针为空，则说明该子节点还没有被创建。
        // 此时，需要根据父节点的空间区域和比例坐标来计算出子节点的空间区域，并创建一个新的OctNode类对象作为子节点，
        // 并将其深度设为父节点深度减一。然后，将子节点指针指向新创建的子节点对象。
      Rectangle3D cBounds(bounds.getMin() + dims * cubes[i * 2],
                          bounds.getMin() + dims * cubes[i * 2 + 1]);
      children[i] = new OctNode(cBounds, depth - 1);
    }
    // 如果对应的子节点指针不为空，则说明该子节点已经被创建。此时，直接访问该子节点指针所指向的子节点对象。
    children[i]->insert(move, bbox);
  }
  // 无论是创建还是访问子节点，都需要调用子节点对象的insert方法，将移动对象和其边界框作为参数传递。
  // 这样就实现了递归地将移动对象插入到合适的子节点中。
}

// OctNode类的intersects方法的实现，它与头文件中的声明一致，没有任何变化。
// 用于判断OctTree类对象是否与给定的矩形区域相交。相交的意思是两个矩形区域有至少一个公共点。
// 这个方法有一个参数，就是r，它是一个cb::Rectangle3D类的对象，它表示要判断的矩形区域。这个方法的过程如下：
bool OctTree::OctNode::intersects(const Rectangle3D &r) const {
    //  首先，调用根节点对象的intersects方法，将r作为参数传递。
    // 其次，在根节点对象的intersects方法中，首先判断节点所代表的空间区域是否与r相交。如果不相交，则返回false，表示OctTree类对象与r不相交。
  if (!bounds.intersects(r)) return false;
  // 如果相交，则继续执行以下步骤：
  // 如果节点所包含的移动对象集合不为空，则返回true，表示OctTree类对象与r相交。
  // 这是因为节点所包含的移动对象集合表示该节点所覆盖的空间范围，如果该范围与r相交，则说明OctTree类对象中至少有一个移动对象与r相交。
  if (!moves.empty()) return true;

  if (depth)
    for (int i = 0; i < 8; i++)
      if (children[i] && children[i]->intersects(r))
        return true;

  return false;
}

// OctNode类的collisions方法的实现，它与头文件中的声明一致，没有任何变化。
void OctTree::OctNode::collisions(const Vector3D &p,
                                  vector<const GCode::Move *> &moves) const {
  if (!bounds.contains(p)) return;

  moves.insert(moves.end(), this->moves.begin(), this->moves.end());

  if (depth)
    for (int i = 0; i < 8; i++)
      if (children[i]) children[i]->collisions(p, moves);
}

// OctTree类的构造函数的实现，它与头文件中的声明一致，没有任何变化。
OctTree::OctTree(const Rectangle3D &bounds, unsigned depth) {
  double m = bounds.getDimensions().max();

  root = new OctNode(Rectangle3D(bounds.getMin(), bounds.getMin() +
                                 Vector3D(m, m, m)), depth);
}

// OctTree类的析构函数的实现，它与头文件中的声明一致，只有一点不同：它使用了zap函数来释放根节点对象，并将根节点指针设为NULL。这样可以简化代码，并避免内存泄漏。
OctTree::~OctTree() {
  zap(root);
}

// OctTree类的insert方法的实现，它与头文件中的声明一致，没有任何变化。
void OctTree::insert(const GCode::Move *move, const Rectangle3D &bbox) {
  this->bbox.add(bbox);
  root->insert(move, bbox);
}

// OctTree类的intersects方法的实现，它与头文件中的声明一致，没有任何变化。
bool OctTree::intersects(const Rectangle3D &r) const {
  return root->intersects(r);
}

// OctTree类的collisions方法的实现，它与头文件中的声明一致，没有任何变化。
void OctTree::collisions(const Vector3D &p,
                         vector<const GCode::Move *> &moves) const {
  root->collisions(p, moves);
}

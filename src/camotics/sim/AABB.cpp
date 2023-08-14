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

#include "AABB.h"

// 源代码见 https://github.com/CauldronDevelopmentLLC/cbang/blob/master/src/cbang/Zap.h
// 只包括一个宏定义方法 zap(x)，作用为删除对象并将指针指向0（空地址）。
#include <cbang/Zap.h>

#include <algorithm>

using namespace std;
using namespace cb;
using namespace CAMotics;


/// NOTE: Expects @param nodes to be a link list along the left child
AABB::AABB(AABB *nodes) : left(0), right(0), move(0) { // 构造函数，并将左右邻连接起来。左右邻默认为空指针。
  if (!nodes) return; // 若指定的节点组不存在，则直接返回，即构造不含左右邻的节点。

  // Compute bounds
  unsigned count = 0; // 计算边界。
  Vector3D cutV;
  for (AABB *it = nodes; it; it = it->left) { // 循环遍历每个节点：
    if (it->right) THROW("Unexpected right-hand AABB node"); // 若存在右节点，则抛出异常，即不希望存在右节点，因为我们想将自己挨个附加为当前节点的右节点。
    add(*it); // 添加当前节点。
    count++; // 计数+1
    cutV += it->getMax() + it->getMin(); // 边界就是最大值和最小值的和。
  }

  // Degenerate cases
  if (count < 3) { // 如果添加的节点数不到3个。
    if (count == 2) right = nodes->left; // 若添加了两个，则让自己的左邻作为右邻。
    left = nodes->prepend(0); // 自己的左邻为空。
    return; // 构造完毕。
  }
  // 若添加的节点数大于等于3个：
  // Decide split
  unsigned axis = getDimensions().findLargest(); // 找到每个维度中占用范围最大的轴。
  double cut = cutV[axis] / count; // 确定分割数，即按最大轴除以添加的节点数。

  // Partition nodes 分块节点
  AABB *lessThan = 0;
  AABB *greaterThan = 0;
  unsigned lessCount = 0;
  unsigned greaterCount = 0;

  for (AABB *it = nodes; it;) { // 挨个遍历每个节点。
    AABB *next = it->left;
    bool less = it->getMax()[axis] + it->getMin()[axis] < cut; // 判断当前节点的上下限值是否小于分快数。

    if (less) {lessThan = it->prepend(lessThan); lessCount++;} // 若是，将小于自己的附加为自己的左节点。
    else {greaterThan = it->prepend(greaterThan); greaterCount++;} // 否则，将大于自己的附加为自己的右节点。

    it = next; // 遍历下一个。
  }

  // Check for bad partition
  if (!lessThan) lessThan = greaterThan->split(greaterCount / 2); // 若没有小于自己的，则小于自己的就是大于自己的继续对半分割。
  if (!greaterThan) greaterThan = lessThan->split(lessCount / 2); // 若没有大于自己的，则大于自己的就是小于自己的继续对半分割。

  // Recur
  left = new AABB(lessThan); // 自己的左节点全部安排为小于自己的。
  right = new AABB(greaterThan); // 自己的右节点全部安排为大于自己的。
}


AABB::~AABB() {
  zap(left);
  zap(right);
}


AABB *AABB::prepend(AABB *list) {
  left = list;
  return this;
}


AABB *AABB::split(unsigned count) {
  if (!count) return this; // 参数为零时表示不分割，返回自己。
  if (count != 1) return left ? left->split(count - 1) : 0; // 参数不为1时，判断是否有左邻，若有则分割自己的左邻，分割数减1。若没有左邻，则返回空。
  AABB *tmp = left; //若分割数为1，则返回自己的左邻，并将自己的左邻清空。
  left = 0;
  return tmp;
}


unsigned AABB::getTreeHeight() const {
  return max(left ? left->getTreeHeight() : 0, // 若左邻存在，则获取左邻的高度；若左邻不存在，则视为0；
             right ? right->getTreeHeight() : 0) + 1; // 若右邻存在，则获取自己右邻的高度；若右邻不存在，则视为0.二者取大视为高度。
}


bool AABB::intersects(const Rectangle3D &r) {
  if (!Rectangle3D::intersects(r)) return false; // 调用 cbang 的 立体矩形判断相交的函数，若判定为不想交，则直接返回 false。

  return isLeaf() || // 若判定为相交，则再判断当前节点是否为叶节点。若是，则返回 true；若不是，则判断左节点是否存在，若存在则判断左节点是否与其相交；若是则返回 true；若不是，则判断右节点是否存在，若存在则判断右积点是否与其相交，作为最后是否相交的依据。
    (left && left->intersects(r)) || (right && right->intersects(r));
}


void AABB::collisions(const Vector3D &p,
                      vector<const GCode::Move *> &moves) {
  if (!Rectangle3D::contains(p)) return; // 调用 cbang 的 立体矩形判断是否包含边，若不包含，则返回。
  if (isLeaf()) moves.push_back(move); // 若当前节点是叶节点，则将当前节点的“移动”放入组中。
  if (left) left->collisions(p, moves); // 若左节点存在，判断左节点是否与其碰撞。
  if (right) right->collisions(p, moves); // 若右节点存在，判断右节点是否与其碰撞。
}

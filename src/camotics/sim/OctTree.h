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


#include "MoveLookup.h" // 定义了一个名为MoveLookup的抽象基类，用于表示移动对象的查找接口.

#include <set>


namespace CAMotics {
    //O ctTree的类，用于存储和查找三维空间中的移动对象。OctTree是一种数据结构，它将空间划分为八个子区域，
    // 每个子区域又可以递归地划分为八个子区域，直到达到一定的深度。这样可以有效地减少空间搜索的时间复杂度。
  class OctTree : public MoveLookup {
    cb::Rectangle3D bbox; // 用于存储OctTree类对象的边界框，即包含所有移动对象的最小矩形区域。

    class OctNode { // 定义OctTree类对象的节点。每个节点都有以下几个属性：
      cb::Rectangle3D bounds; // 表示节点所代表的空间区域。
      unsigned depth; // 表示节点在树中的深度，根节点的深度为0。

      OctNode *children[8]; // 表示节点的八个子节点的指针数组，如果某个子节点不存在，则对应的指针为NULL。
      std::set<const GCode::Move *> moves; // 表示节点所包含的移动对象的集合。GCode::Move是一个自定义的类，它表示三维空间中的一次移动操作。

    public:
      OctNode(const cb::Rectangle3D &bounds, unsigned depth); // 构造函数，它用于初始化节点对象，并将其所有子节点指针设为NULL。
      ~OctNode(); // 析构函数，它用于释放节点对象占用的内存，并递归地删除其所有子节点对象。

      void insert(const GCode::Move *move, const cb::Rectangle3D &bbox); // 用于将一个移动对象插入到节点中。如果移动对象与节点所代表的空间区域相交，则将其加入到节点所包含的移动对象集合中，并根据需要创建子节点，并将移动对象插入到相应的子节点中。
      bool intersects(const cb::Rectangle3D &r) const; // 用于判断节点所代表的空间区域是否与给定的矩形区域相交。如果相交，则返回true，否则返回false。
      void collisions(const cb::Vector3D &p,
                      std::vector<const GCode::Move *> &moves) const; // 用于查找与给定点相交或包含该点的所有移动对象，并将它们加入到一个向量中。如果节点所代表的空间区域包含该点，则将节点所包含的所有移动对象加入到向量中，并递归地在其所有子节点中进行查找。如果节点所代表的空间区域与该点相交，但不包含该点，则只在其相交的子节点中进行查找。
    };

    OctNode *root; // 用于存储OctTree类对象的根节点的指针。

  public:
    OctTree(const cb::Rectangle3D &bounds, unsigned depth); // 用于初始化OctTree类对象，并创建一个根节点对象。
    ~OctTree(); // 用于释放OctTree类对象占用的内存，并删除其根节点对象。

    // From MoveLookup
    cb::Rectangle3D getBounds() const {return bbox;} // 用于获取OctTree类对象的边界框，它是MoveLookup类的一个纯虚方法，具体实现来自MoveLookup。
    void insert(const GCode::Move *move, const cb::Rectangle3D &bbox); // 用于将一个移动对象插入到OctTree类对象中，它是MoveLookup类的一个纯虚方法，因此必须在子类中实现。它调用了根节点对象的insert方法来完成插入操作。
    bool intersects(const cb::Rectangle3D &r) const; // 用于判断OctTree类对象是否与给定的矩形区域相交。它调用了根节点对象的intersects方法来完成判断操作。
    void collisions(const cb::Vector3D &p, // 用于查找与给定点相交或包含该点的所有移动对象，并将它们加入到一个向量中。它调用了根节点对象的collisions方法来完成查找操作。
                    std::vector<const GCode::Move *> &moves) const;
  };
}

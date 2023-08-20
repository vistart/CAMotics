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

#include "Simulation.h"
#include "Workpiece.h"

#include <camotics/SHA256.h>
#include <camotics/contour/TriangleSurface.h>

#include <cbang/json/JSON.h>
#include <cbang/iostream/UpdateStreamFilter.h>
#include <cbang/net/Base64.h>

#include <boost/ref.hpp>
#include <boost/iostreams/device/null.hpp>
#include <boost/iostreams/filtering_stream.hpp>
namespace io = boost::iostreams;

using namespace std;
using namespace cb;
using namespace CAMotics;

// 实现了Simulation类的成员函数。这个类表示一个切割模拟的参数和结果，用来序列化和反序列化为JSON格式。这个类继承了cb::JSON::Serializable类，表示一个可以序列化和反序列化为JSON格式的对象。这个类的成员函数有以下功能：
Simulation::~Simulation() {} // 析构函数：释放内存。


string Simulation::computeHash() const { // computeHash函数：返回模拟的哈希值。哈希值是一种用来标识和比较数据的字符串，通常由一些数字和字母组成。这个函数使用了SHA256算法和Base64编码来生成哈希值。
  SHA256 sha256;
  UpdateStreamFilter<SHA256> digest(sha256);

  io::filtering_ostream stream;
  stream.push(boost::ref(digest));
  stream.push(io::null_sink());

  JSON::Writer writer(stream);
  write(writer);

  stream.reset();

  return Base64().encode(sha256.finalize());
}


void Simulation::read(const JSON::Value &value) { // read函数：重写了父类cb::JSON::Serializable的虚函数，接受一个JSON::Value对象作为参数，表示一个JSON格式的数据。这个函数用来将JSON格式的数据反序列化为模拟对象，并初始化其成员变量。这个函数会根据JSON数据中的键值对，创建并读取相应的对象，如工具表、工件、工具路径、表面和规划器配置。
  resolution = value.getNumber("resolution", 0);
  time = value.getNumber("time", 0);
  mode = RenderMode::parse(value.getString("render-mode", mode.toString()));

  GCode::ToolTable tools;
  if (value.has("tools")) tools.read(*value.get("tools"));

  if (value.has("workpiece")) workpiece.read(*value.get("workpiece"));
  else workpiece = Rectangle3D();

  if (value.has("path")) {
    path = new GCode::ToolPath(tools);
    path->read(*value.get("path"));
  }

  if (value.has("surface")) {
    SmartPointer<TriangleSurface> surface = new TriangleSurface;
    surface->read(*value.get("surface"));
    this->surface = surface;
  }

  if (value.has("planner")) {
    planConf = new GCode::PlannerConfig;
    planConf->read(*value.get("planner"));
  }
}


void Simulation::write(JSON::Sink &sink) const { //   write函数：重写了父类cb::JSON::Serializable的虚函数，接受一个JSON::Sink对象作为参数，表示一个JSON格式的输出流。这个函数用来将模拟对象序列化为JSON格式的数据，并写入到输出流中。这个函数会根据模拟对象的成员变量，创建并写入相应的键值对，如分辨率、时间、渲染模式、工具表、工件、工具路径、表面和规划器配置。

    sink.beginDict();

  sink.insert("resolution", resolution);
  sink.insert("time", time);
  sink.insert("render-mode", mode.toString());

  if (!path.isNull() && !path->getTools().empty()) {
    sink.beginInsert("tools");
    path->getTools().write(sink);
  }

  if (workpiece != Rectangle3D()) {
    sink.beginInsert("workpiece");
    workpiece.write(sink);
  }

  if (path.isSet()) {
    sink.beginInsert("path");
    path->write(sink);
  }

  if (surface.isSet()) {
    sink.beginInsert("surface");
    surface->write(sink);
  }

  if (planConf.isSet()) {
    sink.beginInsert("planner");
    planConf->write(sink);
  }

  sink.endDict();
}

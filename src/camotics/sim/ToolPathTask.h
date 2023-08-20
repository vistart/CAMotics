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


#include <camotics/Task.h>

#include <gcode/Units.h>
#include <gcode/ToolPath.h>
#include <gcode/ToolTable.h>
#include <gcode/ControllerImpl.h>
#include <gcode/machine/MachinePipeline.h>

#include <string>
#include <vector>
#include <sstream>


namespace cb {
  class Subprocess;
  class Thread;
  class InputSource;
}

namespace GCode {
  class Controller;
  class MachineInterface;
  class PlannerConfig;
}

namespace tplang {class TPLContext;}

namespace CAMotics {
  namespace Project {class Project;}

  class ToolPathTask : public Task { // 表示一个计算工具路径的任务。这个类继承了Task类，表示一个异步的任务，用来执行模拟的计算。这个类有以下特点：
    GCode::ToolTable tools; // tools成员变量，是一个GCode::ToolTable对象。GCode::ToolTable对象表示一个工具表，存储了工具编号和工具形状的映射。
    GCode::Units units; // units成员变量，是一个GCode::Units枚举类型。它表示G代码中使用的单位，可以是英制或者公制。
    std::vector<std::string> files; // files成员变量，是一个字符串的向量。它存储了要计算的G代码文件的名称。
    std::string simJSON; // simJSON成员变量，是一个字符串。它存储了模拟的参数和结果的JSON格式的数据。

    GCode::MachinePipeline pipeline; // pipeline成员变量，是一个GCode::MachinePipeline对象。GCode::MachinePipeline对象表示一个机器管道，用来处理G代码中的指令，并模拟机器的运动和状态。
    GCode::ControllerImpl controller; // controller成员变量，是一个GCode::ControllerImpl对象。GCode::ControllerImpl对象表示一个控制器，用来解析和执行G代码中的指令，并与机器管道交互。

    unsigned errors = 0; // errors成员变量，是一个无符号整数。它表示计算过程中出现的错误数量。
    cb::SmartPointer<GCode::ToolPath> path; // path成员变量，是一个GCode::ToolPath对象的智能指针。GCode::ToolPath对象表示一个工具路径，包含了一系列的移动指令和工具信息。
    std::ostringstream gcode; // gcode成员变量，是一个字符串流。它用来存储计算后生成的G代码。

    cb::SmartPointer<tplang::TPLContext> tplCtx; // 它有一个tplCtx成员变量，是一个tplang::TPLContext对象的智能指针。tplang::TPLContext对象表示一个TPL语言的上下文，用来解释和执行TPL语言中的指令。TPL语言是一种基于Python语法的模板语言，用来生成G代码

  public:
      // 接受一个Project::Project对象和一个GCode::PlannerConfig对象作为参数。这两个函数用来根据项目中的配置和参数初始化tools、units、files、simJSON、pipeline、controller等成员变量，并根据config选择不同的规划器配置。Project::Project对象表示一个CAMotics项目，包含了一些文件和设置信息。GCode::PlannerConfig对象表示一个规划器配置，包含了一些控制移动速度和加速度的参数。
    ToolPathTask(const Project::Project &project,
                 const GCode::PlannerConfig *config = 0);
    ~ToolPathTask(); // 析构函数，释放内存。

    unsigned getErrorCount() const {return errors;} // getErrorCount方法，返回errors的值。
    const cb::SmartPointer<GCode::ToolPath> &getPath() const {return path;} // getPath方法，返回path的常量引用。
    std::string getGCode() const {return gcode.str();} // getGCode方法，返回gcode流中的字符串。
// 所有runTPL开头的方法，分别接受不同类型的参数。这些方法用来运行TPL语言，并生成G代码。如果传入的是文件名或者输入源，则创建并初始化tplCtx，并调用其run方法执行文件或者输入源中的TPL语言。如果传入的是字符串，则调用tplCtx中已存在的runString方法执行字符串中的TPL语言。
    void runTPL(const cb::InputSource &src);
    void runTPL(const std::string &filename);
    void runTPLString(const std::string &s);
// 所有runGCode开头的方法，分别接受不同类型的参数。这些方法用来运行G代码，并生成工具路径。如果传入的是文件名或者输入源，则调用controller中已存在的executeFile或者executeStream方法执行文件或者输入源中的G代码，并将其写入到gcode流中。如果传入的是字符串，则调用controller中已存在的executeString方法执行字符串中的G代码，并将其写入到gcode流中。
    void runGCode(const cb::InputSource &src);
    void runGCode(const std::string &filename);
    void runGCodeString(const std::string &gcode);

    // From Task
    void run(); // run方法，重写了父类Task的虚函数。这个方法用来遍历files向量中的每个文件，并根据文件后缀名选择不同的运行方式。如果文件后缀名是.tpl，则调用runTPL方法运行TPL语言，并生成G代码。如果文件后缀名是.nc或者.gcode，则调用runGCode方法运行G代码，并生成工具路径。最后，将controller中的工具路径赋值给path，并打印一条日志信息，表示计算结束。
    void interrupt();// interrupt方法，重写了父类Task的虚函数。这个方法用来中断任务，并释放pipeline和tplCtx。
  };
}

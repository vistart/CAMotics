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


namespace CAMotics {
  struct Hit { // 定义了一个名为Hit的结构体，表示一个切割模拟中的碰撞信息。这个结构体有以下成员变量：
    float depth // depth：表示碰撞点到工件表面的深度，单位是米。
    float time; // time：表示碰撞发生的时间，单位是秒。
    uint32_t segment; // segment：表示碰撞发生在哪个工具路径的段落，是一个无符号的32位整数。
    uint8_t face; //  face：表示碰撞发生在工具形状的哪个面，是一个无符号的8位整数。
  };
}

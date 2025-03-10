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


#include "MachineAdapter.h"

#include <gcode/MoveStream.h>

#include <cbang/config/Options.h>


namespace GCode {
  class MoveSink : public MachineAdapter {
    MoveStream &stream;

    bool probePending = false;
    double time = 0;
    unsigned count = 0;
    cb::SmartPointer<std::string> lastFile;

  public:
    MoveSink(MoveStream &stream) : stream(stream) {}

    // From MachineInterface
    void seek(port_t port, bool active, bool error);
    void move(const Axes &position, int axes, bool rapid, double time);
    void arc(const cb::Vector3D &offset, const cb::Vector3D &target,
             double degrees, plane_t plane);
  };
}

/*
 ******************************************************************************
 * @addtogroup UAVOROSBridge UAVO to ROS Bridge Module
 * @{
 *
 * @file       rosbridge.h
 * @author     The LibrePilot Project, http://www.librepilot.org Copyright (C) 2016.
 *             Max Planck Institute for intelligent systems, http://www.is.mpg.de Copyright (C) 2016.
 * @brief      Bridges certain UAVObjects to ROS on USB VCP
 *
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Additional note on redistribution: The copyright and license notices above
 * must be maintained in each individual source file that is a derivative work
 * of this source file; otherwise redistribution is prohibited.
 */

#ifndef ROSBROIDGE_H
#define ROSBRIDGE_H

#include "ros/ros.h"
#include "boost/asio.hpp"
#include "boost/thread/mutex.hpp"

namespace librepilot {
class rosbridge_priv;

typedef struct {
    double x;
    double y;
    double z;
} offset3d;

class rosbridge {
public:
    rosbridge(int argc, char * *argv);
    ~rosbridge();
    int run(void);
    boost::posix_time::ptime *getStart(void);
    uint8_t getMySequenceNumber();
    void setMySequenceNumber(uint8_t value);
    int serialWrite(uint8_t *buffer, size_t length);
    void rosinfoPrint(const char *bla);
    std::string getNameSpace();
    void setOffset(offset3d &offset);
    offset3d getOffset();

private:
    rosbridge_priv *instance;
};
}

#endif // ifndef ROSBROIDGE_H

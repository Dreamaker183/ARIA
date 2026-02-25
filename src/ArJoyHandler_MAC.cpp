/*
Adept MobileRobots Robotics Interface for Applications (ARIA)
Copyright (C) 2004-2005 ActivMedia Robotics LLC
Copyright (C) 2006-2010 MobileRobots Inc.
Copyright (C) 2011-2015 Adept Technology, Inc.
Copyright (C) 2016-2018 Omron Adept Technologies, Inc.

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute ARIA under different terms, contact 
Adept MobileRobots for information about a commercial version of ARIA at 
robots@mobilerobots.com or 
Adept MobileRobots, 10 Columbia Drive, Amherst, NH 03031; +1-603-881-7960
*/

#include "ariaOSDef.h"
#include "ArExport.h"
#include "ArJoyHandler.h"
#include "ariaUtil.h"

/**
 * macOS stub implementation of joystick handler.
 * This provides empty implementations for macOS where joystick support
 * is not currently implemented.
 */

AREXPORT bool ArJoyHandler::init(void)
{
  myInitialized = false;
  myJoyNumber = -1;
  myPhysMax = 255;
  
  // Initialize axes and buttons to safe defaults
  myAxes.clear();
  myAxes[0] = 0; // axis 0 (unused)
  myAxes[1] = 0; // axis 1 (x)
  myAxes[2] = 0; // axis 2 (y)
  myAxes[3] = 0; // axis 3 (z)
  
  myButtons.clear();
  for (int i = 0; i < 8; i++) {
    myButtons[i] = false;
  }
  
  ArLog::log(ArLog::Normal, "ArJoyHandler: Joystick support not implemented on macOS");
  return false;
}

AREXPORT void ArJoyHandler::getData(void)
{
  // Stub implementation - no data to get on macOS
  // All axes remain at 0, all buttons remain false
}

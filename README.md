kinect\_pendulum\_demo
====================

This is a simple demo that primarily involves interacting a Qt/OpenGL/trep
application.  The application simulates inverted pendula (up to *n*-links) using
[trep].  In its simplest form the user has the ability to interact with the
system using the mouse as the interface.  There is also a simple [ROS] wrapper
that allow the user to use [ROS]/openni_launch/openni_tracker to control the
pendulum through a Kinect interface.  In the application window, there are also
controls for the user to adjust the weights in the LQR stabilizing controller
(effectively changing the rate of response of the closed-loop system), and there
are controls for adjusting a "trust" metric.  With high trust, the user is
required to help the controller stabilize the pendulum, at low trusts, the
controller can do a greater share of the work.  

This is a work in progress, and may change significantly.


License
-------

Copyright (C) 2012 Jarvis Schultz, schultzjarvis@gmail.com

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.


[trep]: https://code.google.com/p/trep/
[ROS]: http://www.ros.org/wiki/
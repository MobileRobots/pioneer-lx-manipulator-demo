# pioneer-lx-manipulator-demo

This is meant to run on the secondary computer ("top" or "torso" computer) of
a Pioneer LX Manipulator or similar robot.  It tries to connect any available
Kinova arms, a pan/tilt unit (configure via command line arguments), and to 
an ARNL server (configure via command line arguments).  It opens another
ArNetwworking server for configuration and some visualization (to run on the
same computer as the ARNL server instead, use the -serverPort argument to
specify a different server port for this server. You will need to do this if
you see an error like "ArSocket::open: could not bind... Address already in
use ... Can't open server on port 7272 yet, waiting")

For example:
   demo -host 192.168.0.32 -ptzType dpptu 
or:
   demo -host localhost -ptzType axis -serverPort 7070

When ARNL reaches a goal starting with the prefix "Arm Demo" (or just the
name "Arm Demo", it stops the tour of goals, performs
the arm demo (depending on demo mode). 
There are two goal modes:

* Reactive - put the arms in reactive (zero gravity) mode.  The PTU is
  moved to track the left hand.  You must resome touring goals manually via
  MobileEyes.
* CartesianPos - Do a sequence of cartesian positions. The PTU tracks the
  left hand.  When done, automatically resumes touring goals.

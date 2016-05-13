#ifndef KINECTARVIDEOSERVER_H
#define KINECTARVIDEOSERVER_H

#include "Aria.h"
#include "ArNetworking.h"
#include <libfreenect2/libfreenect2.hpp>

class KinectArVideoServer : public virtual ArASyncTask
{
  ArServerBase *server;
  bool shutdown;
  libfreenect2::Freenect2Device *freenect_dev;
  libfreenect2::Freenect2 freenect2;
  int resize_to_width;
  int resize_to_height;
  virtual void *runThread(void*);
  void close();
public:
  KinectArVideoServer(ArServerBase *server, int width=320, int height=240);
  virtual ~KinectArVideoServer();
};

#endif

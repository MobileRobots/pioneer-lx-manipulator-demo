/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2011 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */


#include <iostream>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include "ArVideo.h"
#include "ArVideoOpenCV.h"

#include "KinectArVideoServer.h"

#include <libfreenect2/frame_listener_impl.h>

void KinectArVideoServer::close()
{
  std::cout << "KinectArVideoServer: closing." << std::endl;
  // TODO: restarting ir stream doesn't work!
  // TODO: bad things will happen, if frame listeners are freed before dev->stop() :(
  freenect_dev->stop();
  freenect_dev->close();
}

KinectArVideoServer::~KinectArVideoServer()
{
  close();
}

KinectArVideoServer::KinectArVideoServer(ArServerBase *_server, int width, int height) : 
  server(_server), shutdown(false), freenect_dev(NULL), resize_to_width(width), resize_to_height(height)
{
}

void *KinectArVideoServer::runThread(void*)
{
  // TODO might need to move initialization to separate function

  /* Open Kinect */

/*
  int ndevs = freenect2.enumerateDevices();
  printf("KinectArVideoServer: %d devices\n", ndevs);
  std::string serial = freenect2.getDefaultDeviceSerialNumber();
  printf("KinectArVideoServer: device serial number is %s\n", serial.c_str());
*/

  freenect_dev = freenect2.openDefaultDevice();

  if(!freenect_dev)
  {
    std::cout << "KinectArVideoServer: no kinect2 device connected or failure opening the default one!" << std::endl;
    return 0;
  }

  shutdown = false;


  /* Set up libfreenect listeners, start capturing  */

  //libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
  libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Depth);
  libfreenect2::FrameMap frames;

  freenect_dev->setColorFrameListener(&listener);
  freenect_dev->setIrAndDepthFrameListener(&listener);

  assert(freenect_dev);
  if(! freenect_dev->start() )
  {
    std::cout << "KinectArVideoServer: Error starting stream from kinect!" << std::endl;
    return NULL;
  }

  std::cout << "kinect device serial: " << freenect_dev->getSerialNumber() << std::endl;
  std::cout << "kinect device firmware: " << freenect_dev->getFirmwareVersion() << std::endl;

  // TODO set up registration in libfreenect2

  ArVideoOpenCV kinectDepthSource("Kinect_Depth|libfreenect2|OpenCV");
  ArVideo::createVideoServer(server, &kinectDepthSource, "Kinect_Depth|libfreenect2|OpenCV", "freenect2|Depth|OpenCV");

  ArVideoOpenCV kinectRGBSource("Kinect_RGB|libfreenect2|OpenCV");
  ArVideo::createVideoServer(server, &kinectRGBSource, "Kinect_RGB|libfreenect2|OpenCV", "freenect2|RGB|OpenCV");

//  ArVideoOpenCV kinectThreshSource("Kinect_Depth|libfreenect2|OpenCV_threshold");
//  ArVideo::createVideoServer(&server, &kinectThreshSource, "Kinect_Depth|libfreenect2|OpenCV_threshold", "Kinect depth data with basic threshold applied");


  /* Main loop, capture images from kinect, display, copy to ArVideo sources */

  bool first = true;
  cv::Mat rgbm_small(resize_to_width, resize_to_height, CV_8UC3);
  cv::Mat depthm_small(resize_to_width, resize_to_height, CV_8UC3);
  cv::Mat rgbm_flip(resize_to_width, resize_to_height, CV_8UC3);
  cv::Mat depthm_flip(resize_to_width, resize_to_height, CV_8UC3);

  while(!shutdown)
  {
//    std::cout << "." << std::flush;
    ArTime t;
    listener.waitForNewFrame(frames); //, 30000);
    if(t.secSince() >= 28) 
    {  
      std::cout << "KinectArVideoServer: Warning: took more than 30 seconds to receive a frame from Kinect!" << std::endl;
      //shutdown = true;
      //continue;
    }
    
    
//    printf("%d\n", t.secSince());
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
//    libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

    // todo don't recreate Mat objects each time
    cv::Mat rgbm(rgb->height, rgb->width, CV_8UC4, rgb->data);
    cv::resize (rgbm, rgbm_small, cv::Size(resize_to_width, resize_to_height));
    cv::flip(rgbm_small, rgbm_flip, 1);

    cv::Mat depthm(depth->height, depth->width, CV_32FC1, depth->data);
    cv::resize(depthm, depthm_small, cv::Size(resize_to_width, resize_to_height));
    cv::flip(depthm_small, depthm_flip, 1);


//    cv::Mat depth_thresh(depth->height, depth->width, CV_32FC1, depth->data);
//    cv::threshold(depthm, depth_thresh, 0.4, 1.0, CV_THRESH_BINARY_INV);

//    cv::imshow("rgb", rgbm_small);
//    cv::imshow("ir", cv::Mat(ir->height, ir->width, CV_32FC1, ir->data) / 20000.0f);
//    cv::imshow("depth", depthm / 4500.0f);

    if(!kinectRGBSource.updateVideoDataCopy(rgbm_flip, 1, CV_BGR2RGB))
      std::cout << "KinectArVideoServer: Warning: error copying rgb data to ArVideo source" << std::endl;
    if(!kinectDepthSource.updateVideoDataCopy(depthm_flip/4500.0f, 255,
/*(1/255.0),*/ CV_GRAY2RGB))
      std::cout << "KinectArVideoServer: Warning: error copying depth data to ArVideo source" << std::endl;
//    if(!kinectThreshSource.updateVideoDataCopy(depth_thresh, 255, CV_GRAY2RGB))
//      std::cout << "Warning error copying depth thresholded data to ArVideo source" << std::endl;
    

    listener.release(frames);
    //libfreenect2::this_thread::sleep_for(libfreenect2::chrono::milliseconds(100));

//    if(first)
//    {
//      first = false;
//      cv::moveWindow("rgb",   90, 85); 
//      cv::moveWindow("depth", 90, 599);
//    }

    //ArUtil::sleep(1);
  }


  close();

  // TODO destroy ArVideo servers created

  return 0;
}

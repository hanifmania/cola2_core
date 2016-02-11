/*
   Autor(s)      : Jordi Ferrer Plana, Josep Quintana Plana, Ricard Campos
                 : Embedded in COLA2 by Enric Galceran

   e-mail        : jferrerp@eia.udg.es, josepq@eia.udg.es, rcampos@eia.udg.edu
   Branch        : Computer Vision

   Working Group : Underwater Vision Lab
   Project       : Sparus Camera Module

   Homepage      : http://llamatron.homelinux.net

   Module        : Camera Definition

   File          : CCamera.h

   Compiler      : GNU gcc(ANSI C++)
   Libraries     : - STL(Standard Template Library)

   Notes         : - Fitxer escrit amb codificació ISO-8859-1.
       - Extreta la dependencia de Qt

  -----------------------------------------------------------------------------

   Copyright(C) 2002-2004 by Jordi Ferrer Plana

   This source code is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License
   as published by the Free Software Foundation; either version 2
   of the License, or(at your option) any later version.

   This source code is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

   See GNU licenses at http://www.gnu.org/licenses/licenses.html for
   more details.

  -----------------------------------------------------------------------------
 */

#ifndef COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_IO_CCAMERA_H_
#define COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_IO_CCAMERA_H_

#include <stdlib.h>
#include <unistd.h>              // Linux I/O Files
#include <fcntl.h>               // Linux I/O File Constants
#include <sys/mman.h>            // Linux Memory Management
#include <sys/ioctl.h>           // Linux I/O Control
#include <sys/stat.h>
#include <errno.h>               // Linux Errors
#include <assert.h>              // Assertions
#include <linux/types.h>
#include <linux/videodev2.h>     // Video For Linux 2

#include <iostream>
#include <string>

#define V4L2_CID_PRIVATE_UV_RATIO   (V4L2_CID_PRIVATE_BASE + 8)  // From bttv-driver.c

typedef struct SBufferPointer
{
  void   *Start;     // Init of buffer
  size_t  Length;    // Size of allocated memory
} TBufferPointer;

namespace cola2 {
namespace io {

class CCamera
{
 private:
  int Width, Height;
  static const int Depth;
  static const unsigned int NumBuffers;
  static const unsigned int MinBuffers;

  int Fd;
  int Size;

  std::string DeviceFile;
  int FrameCount;
  TBufferPointer *BufferPointer;
  struct v4l2_requestbuffers Buffer;
  struct v4l2_buffer DataBuffer;

  bool verbose;  // Controls the on-screen debug messages usage
  std::string pixelFormat;
  std::string fieldType;

  CCamera();

  //! Obtain capacities of the device
  void queryCapabilities(void);

  //! Enumerate inputs
  void enumInputs(void);

  //! Enumerate controls
  void enumControls(void);

  //! Enumerate menus
  void enumMenu(struct v4l2_queryctrl *Queryctrl);

  //! Obtain the video format
  void getVideoFormat(void);

  //! Define the video format
  void setVideoFormat(bool color);

  //! Define the video standard
  void setVideoStandard(bool color);

  //! Definir the active input
  void setVideoInput(int InputNumber);

  //! Set Exposure Time
  void setExposure(int exposure);

  //! Enumerate the formats
  void enumFormats(void);

  //! Unmap Buffers
  void unmapBuffers(TBufferPointer *BufferPointer, unsigned int Size);

  //! Free buffers
  void freeKernelBuffers(struct v4l2_requestbuffers *Buffer);

  //! Enqueue the buffer
  void enqueueBuffer(int Index);

  //! Dequeue one buffer
  void dequeueBuffer(struct v4l2_buffer *Buffer);

  //! Obtain the buffers of capturate one image
  void requestBuffers(TBufferPointer **BufferPointer, struct v4l2_requestbuffers *Buffer, unsigned int Count);

  //! Set the overlay
  void setOverlay(void);

  //! Streaming on
  void videoStreamOn(void);

  //! Streaming off
  void videoStreamOff(void);

  //! Get the streaming capabilities
  void getStreamingCapabilities(void);

  //! Print BufferPointer
  void printBufferPointer(TBufferPointer *Buffer);

  //! Print Buffer
  void printBuffer(struct v4l2_buffer *Buffer);

  //! Init data
  void initData(void);

  //! Free an object
  void free(void);

  //! Create an object
  void allocate(const std::string &ConfigFileName, int videoInput, int width, int height, int videoBrightness,
                int videoContrast, int videoSaturation, int videoHue, bool color, std::string pixFmt,
                std::string fieldType_, bool verbose);

 public:
  //! Builder with the name of the configuration file.
  CCamera(const std::string &ConfigFileName = "/dev/video0", int videoInput = 0, int width = 384, int height = 288,
          int videoBrightness = 32768, int videoContrast = 32768, int videoSaturation = 32768, int videoHue = 32768,
          bool color = false, std::string pixFmt = "V4L2_PIX_FMT_RGB32",
          std::string fieldType_ = "V4L2_FIELD_INTERLACED", bool verbose = false);

  //! Destroyer.
  virtual ~CCamera(void);

  //! Start/Restart the camera with a configuration camera.
  void init(const std::string &ConfigFileName = "/dev/video0", int videoInput = 0, int width = 384, int height = 288,
            int videoBrightness = 32768, int videoContrast = 32768, int videoSaturation = 32768, int videoHue = 32768,
            bool color = false, std::string pixFmt = "V4L2_PIX_FMT_RGB32",
            std::string fieldType_ = "V4L2_FIELD_INTERLACED", bool verbose = false);

  //! Adquire the data buffer.
  unsigned char* grab(void);

  //! Freeing the buffer after the process of the image.
  void grabRelease(void);

  //! Check if the initialization was good.
  bool isInit(void) const;

  //! Return the with of the image.
  int width(void) const;

  //! Return the height of the image.
  int height(void) const;

  //! Return the number of bytes for pixel
  int bytesPerPixel(bool color) const;

  //! Return the size of the buffer
  int size(void) const;

  //! Return the name of the adquired frame
  int grabFramesCount(void) const;

  //! Return the number of devices
  int nDevices(void) const;

  //! Definir the camera brightness
  void setVideoBrightness(int InputBrightness);

  //! Definir the camera contrast
  void setVideoContrast(int InputContrast);

  //! Define the camera Saturation
  void setVideoSaturation(int InputSaturation);

  //! Set UV ratio
  void setVideoUvRatio(int UVRatio);

  //! Define the camera Hue
  void setVideoHue(int InputHue);

  //! Set Auto White Balance
  void setAutoWhiteBalance(bool AutoWhite);

  //! Set Auto Gain
  void setAutoGain(bool AutoGain);
};

const int CCamera::Depth = 1;                // estava a 1
const unsigned int CCamera::NumBuffers = 1;  // estava a 4
const unsigned int CCamera::MinBuffers = 1;  // estava a 2

}  // namespace io
}  // namespace cola2

#endif  // COLA2_LIB_INCLUDE_COLA2_LIB_COLA2_IO_CCAMERA_H_

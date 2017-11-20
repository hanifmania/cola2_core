
/*
 * Copyright (c) 2017 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

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

#include "cola2_lib/cola2_io/CCamera.h"

namespace cola2 {
namespace io {

CCamera::CCamera()
{
}

//! Obtain capacities of the device
void CCamera::queryCapabilities(void)
{
  struct v4l2_capability Cap;

  if (verbose)
  {
    std::cout << "[CCamera] * Query capabilities:" << std::endl;
    // qDebug() <<(QString("3. * Query capabilities:"));
  }

  if (ioctl(Fd, VIDIOC_QUERYCAP, &Cap) == -1)
  {
    std::cout << "[CCamera] ioctl error: VIDIOC_QUERYCAP" << std::endl;
    // qDebug() <<(QString("3. ioctl error: VIDIOC_QUERYCAP"));
    exit(EXIT_FAILURE);
  }
  else
  {
    std::cout << "[CCamera] * Driver      : '" << Cap.driver << "'" << std::endl;
    std::cout << "[CCamera] * Card        : '" << Cap.card << "'" << std::endl;
    std::cout << "[CCamera] * Bus Info    : '" << Cap.bus_info << "'" << std::endl;
    std::cout << "[CCamera] * Version     : '" << Cap.version << "'" << std::endl;
    std::cout << "[CCamera] * Driver Capabilities" << std::endl;
    std::cout << "[CCamera]     - Video Capture     : " <<((Cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) != 0) << std::endl;
    std::cout << "[CCamera]     - Video Output      : " <<((Cap.capabilities & V4L2_CAP_VIDEO_OUTPUT) != 0) << std::endl;
    std::cout << "[CCamera]     - Video Overlay     : " <<((Cap.capabilities & V4L2_CAP_VIDEO_OVERLAY) != 0) << std::endl;
    std::cout << "[CCamera]     - VBI Video Capture : " <<((Cap.capabilities & V4L2_CAP_VBI_CAPTURE) != 0) << std::endl;
    std::cout << "[CCamera]     - VBI Video Output  : " <<((Cap.capabilities & V4L2_CAP_VBI_OUTPUT) != 0) << std::endl;
    std::cout << "[CCamera]     - RDS Capture       : " <<((Cap.capabilities & V4L2_CAP_RDS_CAPTURE) != 0) << std::endl;
    std::cout << "[CCamera]     - Tuner             : " <<((Cap.capabilities & V4L2_CAP_TUNER) != 0) << std::endl;
    std::cout << "[CCamera]     - Audio             : " <<((Cap.capabilities & V4L2_CAP_AUDIO) != 0) << std::endl;
    std::cout << "[CCamera]     - Read Write        : " <<((Cap.capabilities & V4L2_CAP_READWRITE) != 0) << std::endl;
    std::cout << "[CCamera]     - Asynchronous IO   : " <<((Cap.capabilities & V4L2_CAP_ASYNCIO) != 0) << std::endl;
    std::cout << "[CCamera]     - Streaming         : " <<((Cap.capabilities & V4L2_CAP_STREAMING) != 0) << std::endl;
    // qDebug() <<(QString("3. * Driver      : '") + QString((char *)Cap.driver) + QString("'"));
    // qDebug() <<(QString("3. * Card        : '") + QString((char *)Cap.card) + QString("'"));
    // qDebug() <<(QString("3. * Bus Info    : '") + QString((char *)Cap.bus_info) + QString("'"));
    // qDebug() <<(QString("3. * Version     : '") + QString::number(Cap.version) + QString("'"));
    // qDebug() <<(QString("3. * Driver Capabilities"));
    // qDebug() <<(QString("3.     - Video Capture     : ") + QString::number((Cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) != 0));
    // qDebug() <<(QString("3.     - Video Output      : ") + QString::number((Cap.capabilities & V4L2_CAP_VIDEO_OUTPUT) != 0));
    // qDebug() <<(QString("3.     - Video Overlay     : ") + QString::number((Cap.capabilities & V4L2_CAP_VIDEO_OVERLAY) != 0));
    // qDebug() <<(QString("3.     - VBI Video Capture : ") + QString::number((Cap.capabilities & V4L2_CAP_VBI_CAPTURE) != 0));
    // qDebug() <<(QString("3.     - VBI Video Output  : ") + QString::number((Cap.capabilities & V4L2_CAP_VBI_OUTPUT) != 0));
    // qDebug() <<(QString("3.     - RDS Capture       : ") + QString::number((Cap.capabilities & V4L2_CAP_RDS_CAPTURE) != 0));
    // qDebug() <<(QString("3.     - Tuner             : ") + QString::number((Cap.capabilities & V4L2_CAP_TUNER) != 0));
    // qDebug() <<(QString("3.     - Audio             : ") + QString::number((Cap.capabilities & V4L2_CAP_AUDIO) != 0));
    // qDebug() <<(QString("3.     - Read Write        : ") + QString::number((Cap.capabilities & V4L2_CAP_READWRITE) != 0));
    // qDebug() <<(QString("3.     - Asynchronous IO   : ") + QString::number((Cap.capabilities & V4L2_CAP_ASYNCIO) != 0));
    // qDebug() <<(QString("3.     - Streaming         : ") + QString::number((Cap.capabilities & V4L2_CAP_STREAMING) != 0));
  }
}

//! Enumerate inputs
void CCamera::enumInputs(void)
{
  struct v4l2_input Input;

  if (verbose)
  {
    std::cout << "[CCamera] * Enum Inputs" << std::endl;
    // qDebug() <<(QString("3. * Enum Inputs"));

    Input.index = 0;
    while (ioctl(Fd, VIDIOC_ENUMINPUT, &Input) == 0)
    {
      std::cout << "[CCamera] * Device Inputs : " << Input.index << std::endl;
      std::cout << "[CCamera]     - Name      : " << Input.name << std::endl;
      std::cout << "[CCamera]     - Type      : " << Input.type << std::endl;
      std::cout << "[CCamera]     - Audio Set : " << Input.audioset << std::endl;
      std::cout << "[CCamera]     - Tuner     : " << Input.tuner << std::endl;
      std::cout << "[CCamera]     - Standard  : " << static_cast<unsigned int>(Input.std) << std::endl;
      std::cout << "[CCamera]     - Status    : " << Input.status << std::endl;
      // qDebug() <<(QString("3. * Device Inputs : ") + QString::number(Input.index));
      // qDebug() <<(QString("3.     - Name      : ") + QString((char *)Input.name));
      // qDebug() <<(QString("3.     - Type      : ") + QString::number(Input.type));
      // qDebug() <<(QString("3.     - Audio Set : ") + QString::number(Input.audioset));
      // qDebug() <<(QString("3.     - Tuner     : ") + QString::number(Input.tuner));
      // qDebug() <<(QString("3.     - Standard  : ") + QString::number((unsigned int)Input.std));
      // qDebug() <<(QString("3.     - Status    : ") + QString::number(Input.status));
      Input.index++;
    }
  }
}

//! Enumerate controls
void CCamera::enumControls(void)
{
  if (verbose)
  {
    struct v4l2_queryctrl Queryctrl;

    std::cout << "[CCamera] * Controls del dispositiu:" << std::endl;
    // qDebug() <<(QString("3. * Controls del dispositiu:"));

    Queryctrl.id = V4L2_CID_BASE;
    while ((Queryctrl.id < V4L2_CID_LASTP1) && (ioctl(Fd, VIDIOC_QUERYCTRL, &Queryctrl) == 0))
    {
      if (!(Queryctrl.flags & V4L2_CTRL_FLAG_DISABLED))
      {
        std::cout << "[CCamera]     - Control: '" << Queryctrl.name << "'" << std::endl;
        // qDebug() <<(QString("3.     - Control: '") + QString((char *)Queryctrl.name) + QString("'"));
        if (Queryctrl.type == V4L2_CTRL_TYPE_MENU)
          enumMenu(&Queryctrl);
      }
      Queryctrl.id++;
    }

    Queryctrl.id = V4L2_CID_PRIVATE_BASE;
    while (ioctl(Fd, VIDIOC_QUERYCTRL, &Queryctrl) == 0)
    {
      if (!(Queryctrl.flags & V4L2_CTRL_FLAG_DISABLED))
      {
        std::cout << "[CCamera]     - Control: '" << Queryctrl.name << "'" << std::endl;
        // qDebug() <<(QString("3.     - Control: '") + QString((char *)Queryctrl.name) + QString("'"));

        if (Queryctrl.type == V4L2_CTRL_TYPE_MENU)
          enumMenu(&Queryctrl);
      }
      Queryctrl.id++;
    }
  }
}

//! Enumerate menus
void CCamera::enumMenu(struct v4l2_queryctrl *Queryctrl)
{
  struct v4l2_querymenu Querymenu;
  int i;

  std::cout << "[CCamera] * Menu items:" << std::endl;
  // qDebug() <<(QString("3. * Menu items:"));

  Querymenu.id = Queryctrl->id;
  for (i = Queryctrl->minimum; i <= Queryctrl->maximum; i++)
  {
    Querymenu.index = i;
    //      if (ioctl(Fd, VIDIOC_QUERYMENU, &Querymenu) == 0)
    // qDebug() <<(QString("3.    ") + QString((char *) Querymenu.name));
    //      else
    //      {
    if (ioctl(Fd, VIDIOC_QUERYMENU, &Querymenu) != 0)
    {
      std::cerr << "[CCamera - Error] ioctl error: VIDIOC_QUERYMENU" << std::endl;
      // qDebug() <<(QString("3. ioctl error: VIDIOC_QUERYMENU"));
      exit(EXIT_FAILURE);
    }
    else
    {
      std::cout << "[CCamera]    " << Querymenu.name << std::endl;
    }
  }
}

//! Obtain the video format
void CCamera::getVideoFormat(void)
{
  struct v4l2_format Format;

  std::cout << "[CCamera] * Get Video Format:" << std::endl;
  // qDebug() <<(QString("3. * Get Video Format:"));

  Format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(Fd, VIDIOC_G_FMT, &Format) == -1)
  {
    std::cout << "[CCamera] ioctl error: VIDIOC_G_FMT" << std::endl;
    // qDebug() <<(QString("3. ioctl error: VIDIOC_G_FMT"));
    exit(EXIT_FAILURE);
  }
  else
  {
    std::cout << "[CCamera]     - Type         : " << Format.type << std::endl;
    std::cout << "[CCamera]     - Width        : " << Format.fmt.pix.width << std::endl;
    std::cout << "[CCamera]     - Height       : " << Format.fmt.pix.height << std::endl;
    std::cout << "[CCamera]     - Pixel format : " << Format.fmt.pix.pixelformat << std::endl;
    std::cout << "[CCamera]     - Field        : " << Format.fmt.pix.field << std::endl;
    std::cout << "[CCamera]     - Bytes/Line   : " << Format.fmt.pix.bytesperline << std::endl;
    std::cout << "[CCamera]     - Image size   : " << Format.fmt.pix.sizeimage << std::endl;
    std::cout << "[CCamera]     - Color space  : " << Format.fmt.pix.colorspace << std::endl;
    // qDebug() <<(QString("3.     - Type         : ") + QString::number(Format.type));
    // qDebug() <<(QString("3.     - Width        : ") + QString::number(Format.fmt.pix.width));
    // qDebug() <<(QString("3.     - Height       : ") + QString::number(Format.fmt.pix.height));
    // qDebug() <<(QString("3.     - Pixel format : ") + QString::number(Format.fmt.pix.pixelformat));
    // qDebug() <<(QString("3.     - Field        : ") + QString::number(Format.fmt.pix.field));
    // qDebug() <<(QString("3.     - Bytes/Line   : ") + QString::number(Format.fmt.pix.bytesperline));
    // qDebug() <<(QString("3.     - Image size   : ") + QString::number(Format.fmt.pix.sizeimage));
    // qDebug() <<(QString("3.     - Color space  : ") + QString::number(Format.fmt.pix.colorspace));
  }
}

//! Define the video format
void CCamera::setVideoFormat(bool color)
{
  struct v4l2_format Format;

  if (verbose)
  {
    std::cout << "[CCamera] * Set Video Format : " << std::endl;
    // qDebug() <<(QString("3. * Set Video Format : "));
  }
  Format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(Fd, VIDIOC_G_FMT, &Format) == -1)
  {
    std::cerr << "[CCamera - Error] ioctl error: VIDIOC_G_FMT" << std::endl;
    // qDebug() <<(QString("3. ioctl error: VIDIOC_G_FMT"));
    exit(EXIT_FAILURE);
  }
  else
  {
    Format.fmt.pix.width = Width;
    Format.fmt.pix.height = Height;
    if (color)
    {
      // Format.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB32;  //COLOR RGB32
      // Format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;  //COLOR YUYV
      if (pixelFormat.compare("V4L2_PIX_FMT_YUYV") == 0)
      {
        Format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
      }
      else if (pixelFormat.compare("V4L2_PIX_FMT_RGB32") == 0)
      {
        Format.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB32;
      }
      else
      {
        std::cerr << "Unknown(or unimplemented) Pixel Format!" << std::endl;
        exit(EXIT_FAILURE);
      }
      // assert(!(pixelFormat.compare("V4L2_PIX_FMT_YUYV") &(Width < 2 || Width & 1)));
    }
    else
    {
      Format.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;  // GREY
    }

    if (fieldType.compare("V4L2_FIELD_ALTERNATE") == 0)
    {
      Format.fmt.pix.field = V4L2_FIELD_ALTERNATE;
    }
    else
    {
      if (fieldType.compare("V4L2_FIELD_INTERLACED") == 0)
      {
        Format.fmt.pix.field = V4L2_FIELD_INTERLACED;
      }
      else
      {
        if (fieldType.compare("V4L2_FIELD_BOTTOM") == 0)
        {
          Format.fmt.pix.field = V4L2_FIELD_BOTTOM;
        }
        else
        {
          if (fieldType.compare("V4L2_FIELD_TOP") == 0)
          {
            Format.fmt.pix.field = V4L2_FIELD_TOP;
          }
          else
          {
            std::cerr << "Unknown(or unimplemented) Pixel Format!" << std::endl;
            exit(EXIT_FAILURE);
          }
        }
      }
    }
    // Format.fmt.pix.field = V4L2_FIELD_ALTERNATE;
    // Format.fmt.pix.field = V4L2_FIELD_INTERLACED;
    if (ioctl(Fd, VIDIOC_S_FMT, &Format) == -1)
    {
      // qDebug() <<(QString("3. ioctl error: VIDIOC_S_FMT"));
      std::cerr << "[CCamera - Error] ioctl error: VIDIOC_S_FMT" << std::endl;
      exit(EXIT_FAILURE);
    }
    else if (verbose)
    {
      std::cout << "[CCamera] Ok!" << std::endl;
      // qDebug() <<(QString("3. Ok!"));
    }
  }
}

//! Define the video standard
void CCamera::setVideoStandard(bool color)
{
  v4l2_std_id Std;

  if (verbose)
  {
    std::cout << "[CCamera] * Set video standard: " << std::endl;
    // qDebug() <<(QString("3. * Set video standard: "));
  }

  if (color)
  {
    Std = V4L2_STD_PAL;
  }
  else
  {
    Std = V4L2_STD_PAL_Nc;
  }

  if (ioctl(Fd, VIDIOC_S_STD, &Std) == -1)
  {
    std::cerr << "[CCamera - Error] ioctl error: VIDIOC_S_STD" << std::endl;
    // qDebug() <<(QString("3. ioctl error: VIDIOC_S_STD"));
    exit(EXIT_FAILURE);
  }
  else if (verbose)
  {
    std::cout << "[CCamera] Ok!" << std::endl;
    // qDebug() <<(QString("3. Ok!"));
  }
}

//! Definir the active input
void CCamera::setVideoInput(int InputNumber)
{
  if (ioctl(Fd, VIDIOC_S_INPUT, &InputNumber) == -1)
    std::cerr << "[CCamera - Error] * Device Set Input error!" << std::endl;
  // qDebug() <<(QString("3. * Device Set Input error!"));
}

//! Set Exposure Time
void CCamera::setExposure(int exposure)
{
  struct v4l2_control s;
  s.id = V4L2_CID_EXPOSURE;
  s.value = exposure;

  if (ioctl(Fd, VIDIOC_S_CTRL, &s) == -1)
    std::cerr << "[CCamera - Error] * Device Set Exposure error!" << std::endl;
  // qDebug() <<(QString("3. * Device Set Exposure error!"));
}

//! Enumerate the formats
void CCamera::enumFormats(void)
{
  struct v4l2_fmtdesc  FormatDesc;

  std::cout << "[CCamera] * Enum Formats:" << std::endl;
  // qDebug() <<(QString("3. * Enum Formats:"));

  FormatDesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  FormatDesc.index = 0;
  while (ioctl(Fd, VIDIOC_ENUM_FMT, &FormatDesc) == 0)
  {
    std::cout << "[CCamera] * Enumerar format  : " << FormatDesc.index << std::endl;
    std::cout << "[CCamera]     - Type         : " << FormatDesc.type << std::endl;
    std::cout << "[CCamera]     - Flags        : " << FormatDesc.flags << std::endl;
    std::cout << "[CCamera]     - Description  : '" << FormatDesc.description << "'" << std::endl;
    std::cout << "[CCamera]     - Pixel format : " << FormatDesc.pixelformat << std::endl;
    // qDebug() <<(QString("3. * Enumerar format  : ") + QString::number(FormatDesc.index));
    // qDebug() <<(QString("3.     - Type         : ") + QString::number(FormatDesc.type));
    // qDebug() <<(QString("3.     - Flags        : ") + QString::number(FormatDesc.flags));
    // qDebug() <<(QString("3.     - Description  : '") + QString((char *)FormatDesc.description) + QString("'"));
    // qDebug() <<(QString("3.     - Pixel format : ") + QString::number(FormatDesc.pixelformat));
    FormatDesc.index++;
  }
}

//! Unmap Buffers
void CCamera::unmapBuffers(TBufferPointer *BufferPointer, unsigned int Size)
{
  unsigned int i;

  if (BufferPointer != NULL)
  {
    for (i = 0; i < Size; i++)
    {
      if (BufferPointer[i].Start != MAP_FAILED)
        munmap(BufferPointer[i].Start, BufferPointer[i].Length);
    }
  }
}

//! Free buffers
void CCamera::freeKernelBuffers(struct v4l2_requestbuffers *Buffer)
{
  // int Count;
  //
  // Count = Buffer->count;
  Buffer->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  Buffer->memory = V4L2_MEMORY_MMAP;
  Buffer->count = 0;
  if (ioctl(Fd, VIDIOC_REQBUFS, Buffer) == -1)
    std::cerr << "[CCamera - Error] Can't free allocated memory" << std::endl;
  // qDebug() <<(QString("3. Can't free allocated memory(") + QString::number(Count) + QString(")"));
}

//! Enqueue the buffer
void CCamera::enqueueBuffer(int Index)
{
  struct v4l2_buffer DriverBuffer;

  if (verbose)
  {
    std::cout << "[CCamera] * Encuar buffer: " << Index << std::endl;
    // qDebug() <<(QString("3. * Encuar buffer: ") + QString::number(Index));
  }

  DriverBuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  DriverBuffer.index = Index;

  if (ioctl(Fd, VIDIOC_QUERYBUF, &DriverBuffer) == -1)
  {
    std::cerr << "[CCamera - Error] ioctl error: VIDIOC_QUERYBUF" << std::endl;
    // qDebug() <<(QString("3. ioctl error: VIDIOC_QUERYBUF"));
    exit(EXIT_FAILURE);
  }
  else
  {
    // DriverBuffer.bytesused = Width * Height * Depth;
    // DriverBuffer.field = V4L2_FIELD_ALTERNATE;
    // DriverBuffer.field = V4L2_FIELD_TOP;

    if (ioctl(Fd, VIDIOC_QBUF, &DriverBuffer) == -1)
    {
      switch (errno)
      {
      case EAGAIN:
        std::cerr << "[CCamera - Error] No hi ha buffers a la cua!" << std::endl;
        // qDebug() <<(QString("3. No hi ha buffers a la cua!"));
        break;

      case EINVAL:
        std::cerr << "[CCamera-Error] Tipus o numero de buffer incorrecte, o no hi ha buffers allotjats!" << std::endl;
        // qDebug() <<(QString("3. Tipus o numero de buffer incorrecte, o no hi ha buffers allotjats!"));
        break;

      case ENOMEM:
        std::cerr << "[CCamera - Error] Memoria insuficient" << std::endl;
        // qDebug() <<(QString("3. Memoria insuficient"));
        break;

      default:
        std::cerr << "[CCamera - Error] ioctl error: Unknown error!" << std::endl;
        // qDebug() <<(QString("3. ioctl error: Unknown error!"));
        break;
      }

      exit(EXIT_FAILURE);
    }
    else if (verbose)
    {
      std::cout << "[CCamera] * Buffer enqueued!" << std::endl;
      // qDebug() <<(QString("3. * Buffer enqueued!"));
    }
  }
}

//! Dequeue one buffer
void CCamera::dequeueBuffer(struct v4l2_buffer *Buffer)
{
  Buffer->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(Fd, VIDIOC_DQBUF, Buffer) == -1)
  {
    std::cerr << "[CCamera - Error] ioctl error: VIDIOC_DQBUF" << std::endl;
    // qDebug() <<(QString("3. ioctl error: VIDIOC_DQBUF"));
  }
  else if (verbose)
  {
    std::cout << "[CCamera] * Dequeued Buffer" << std::endl;
    // qDebug() <<(QString("3. * Dequeued Buffer"));
    printBuffer(Buffer);
  }
}

//! Obtain the buffers of capturate one image
void CCamera::requestBuffers(TBufferPointer **BufferPointer, struct v4l2_requestbuffers *Buffer, unsigned int Count)
{
  unsigned int i;

  if (verbose)
  {
    std::cout << "[CCamera] * Request " << Count << " buffers" << std::endl;
    //   qDebug() <<(QString("3. * Request ") + QString::number(Count) + QString(" buffers"));
  }

  Buffer->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  Buffer->memory = V4L2_MEMORY_MMAP;
  Buffer->count = Count;

  if (ioctl(Fd, VIDIOC_REQBUFS, Buffer) == -1)
  {
    if (errno == EINVAL)
      std::cerr << "[CCamera - Error] Video capturing or mmap-streaming is not supported" << std::endl;
    // qDebug() <<(QString("3. Video capturing or mmap-streaming is not supported"));
    else
      std::cerr << "[CCamera - Error] ioctl error: VIDIOC_REQBUFS" << std::endl;
    // qDebug() <<(QString("3. ioctl error: VIDIOC_REQBUFS"));

    exit(EXIT_FAILURE);
  }

  if (Buffer->count < MinBuffers)
  {
    std::cerr << "[CCamera-Error] Not enough buffer memory " << MinBuffers << "/" << Buffer->count << ")" << std::endl;
    //      qDebug() <<(QString("3. Not enough buffer memory ") +
    //               QString::number(MinBuffers) + QString("/") +
    //               QString::number(Buffer->count) + QString(")"));

    freeKernelBuffers(Buffer);
    *BufferPointer = NULL;

    exit(EXIT_FAILURE);
  }

  *BufferPointer = static_cast<TBufferPointer*>(calloc(Buffer->count, sizeof(**BufferPointer)));
  assert(*BufferPointer != NULL);

  for (i = 0; i < Buffer->count; i++)
  {
    struct v4l2_buffer buffer;

    buffer.type = Buffer->type;
    buffer.index = i;

    if (verbose)
    {
      std::cout << "[CCamera]  * Query buffer: " << i << std::endl;
      //      qDebug() <<(QString("3.  * Query buffer: ") + QString::number(i));
    }
    if (ioctl(Fd, VIDIOC_QUERYBUF, &buffer) == -1)
    {
      std::cerr << "[CCamera - Error] ioctl error: VIDIOC_QUERYBUF" << std::endl;
      // qDebug() <<(QString("3. ioctl error: VIDIOC_QUERYBUF"));

      // Eliminar buffers
      unmapBuffers(*BufferPointer, Buffer->count);
      freeKernelBuffers(Buffer);
      *BufferPointer = NULL;

      exit(EXIT_FAILURE);
    }
    else if (verbose)
    {
      std::cout << "[CCamera] Ok!" << std::endl;
      // qDebug() <<(QString("3. Ok!"));
    }

    if (verbose)
    {
      std::cout << "[CCamera]  * mmap buffer: " << i << std::endl;
      //      qDebug() <<(QString("3.  * mmpa buffer: ") + QString::number(i));
    }

    (*BufferPointer)[i].Length = buffer.length;
    (*BufferPointer)[i].Start = mmap(NULL, buffer.length,
                                     PROT_READ | PROT_WRITE,  // Necessari
                                     MAP_SHARED,              // Recomanat
                                     Fd, buffer.m.offset);

    if ((*BufferPointer)[i].Start == MAP_FAILED)
    {
      unmapBuffers(*BufferPointer, i);
      freeKernelBuffers(Buffer);
      *BufferPointer = NULL;

      std::cerr << "[CCamera - Error] mmap memory error" << std::endl;
      // qDebug() <<(QString("3. mmap memory error"));

      exit(EXIT_FAILURE);
    }
    else if (verbose)
    {
      std::cout << "[CCamera]     - Index  : " << i << std::endl;
      //         qDebug() <<(QString("3.     - Index  : ") + QString::number(i));
      printBufferPointer(&(*BufferPointer)[i]);
    }
  }
}

//! Set the overlay
void CCamera::setOverlay(void)
{
  int Ov = 0;

  if (verbose)
  {
    std::cout << "[CCamera] * Overlay:" << std::endl;
    // qDebug() <<(QString("3. * Overlay:"));
  }
  if (ioctl(Fd, VIDIOC_OVERLAY, &Ov) == -1 )
  {
    std::cout << "[CCamera] ioctl error : VIDIOC_OVERLAY" << std::endl;
    // qDebug() <<(QString("3. ioctl error : VIDIOC_OVERLAY"));
    exit(EXIT_FAILURE);
  }
  else
  {
    std::cout << "[CCamera]    - Overlay : " << Ov << std::endl;
    // qDebug() <<(QString("3.    - Overlay : ") + QString::number(Ov));
  }
}

//! Streaming on
void CCamera::videoStreamOn(void)
{
  int Argp;

  Argp = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(Fd, VIDIOC_STREAMON, &Argp) == -1)
  {
    std::cout << "[CCamera] ioctl error: VIDIOC_STREAMON" << std::endl;
    // qDebug() <<(QString("3. ioctl error: VIDIOC_STREAMON"));
    exit(EXIT_FAILURE);
  }
  else
  {
    std::cout << "[CCamera] * Stream on: " << Argp << std::endl;
    // qDebug() <<(QString("3. * Stream on: ") + QString::number(Argp));
  }
}

//! Streaming off
void CCamera::videoStreamOff(void)
{
  int Argp;

  Argp = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(Fd, VIDIOC_STREAMOFF, &Argp) == -1)
  {
    std::cout << "[CCamera] ioctl error: VIDIOC_STREAMOFF" << std::endl;
    // qDebug() <<(QString("3. ioctl error: VIDIOC_STREAMOFF"));
    exit(EXIT_FAILURE);
  }
  else
  {
    std::cout << "[CCamera] * Stream off: " << Argp << std::endl;
    // qDebug() <<(QString("3. * Stream off: ") + QString::number(Argp));
  }
}

//! Get the streaming capabilities
void CCamera::getStreamingCapabilities(void)
{
  struct v4l2_requestbuffers RBuff;

  if (verbose)
  {
    std::cout << "[CCamera] * Get streaming capabilities:" << std::endl;
    // qDebug() <<(QString("3. * Get streaming capabilities:"));
  }

  RBuff.count = NumBuffers;
  RBuff.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  RBuff.memory = V4L2_MEMORY_MMAP;
  if (ioctl(Fd, V4L2_MEMORY_USERPTR, &RBuff) == -1)
  {
    std::cout << "[CCamera] ioctl error : V4L2_MEMORY_USERPTR" << std::endl;
    // qDebug() <<(QString("3. ioctl error : V4L2_MEMORY_USERPTR"));
    exit(EXIT_FAILURE);
  }
  else
  {
    std::cout << "[CCamera] Ok!" << std::endl;
    // qDebug() <<(QString("3. Ok!"));
  }
}

//! Print BufferPointer
void CCamera::printBufferPointer(TBufferPointer *Buffer)
{
  std::cout << "[CCamera]     - Start  : 0x" << Buffer->Start << std::endl;
  std::cout << "[CCamera]     - Length : " << Buffer->Length << std::endl;
  // qDebug() <<(QString("3.     - Start  : 0x%1").arg((unsigned int)Buffer->Start, 0, 16));
  // qDebug() <<(QString("3.     - Length : ") + QString::number((unsigned int)Buffer->Length));
}

//! Print Buffer
void CCamera::printBuffer(struct v4l2_buffer *Buffer)
{
  std::cout << "[CCamera]     - Index        : " << Buffer->index << std::endl;
  std::cout << "[CCamera]     - Type         : " << Buffer->type << std::endl;
  std::cout << "[CCamera]     - Bytes Used   : " << Buffer->bytesused << std::endl;
  std::cout << "[CCamera]     - Flags        : " << Buffer->flags << std::endl;
  std::cout << "[CCamera]     - Field        : " << Buffer->field << std::endl;

  std::cout << "[CCamera]     - Time Stamp   :" << std::endl;
  std::cout << "[CCamera]         - Tv Sec   : " << Buffer->timestamp.tv_sec << std::endl;
  std::cout << "[CCamera]         - Tv USec  : " << Buffer->timestamp.tv_usec << std::endl;
  std::cout << "[CCamera]     - Time Code    :" << std::endl;
  std::cout << "[CCamera]         - Type     : " << Buffer->timecode.type << std::endl;
  std::cout << "[CCamera]         - Flags    : " << Buffer->timecode.flags << std::endl;
  std::cout << "[CCamera]         - Frames   : " << Buffer->timecode.frames << std::endl;
  std::cout << "[CCamera]         - Seconds  : " << Buffer->timecode.seconds << std::endl;
  std::cout << "[CCamera]         - Minutes  : " << Buffer->timecode.minutes << std::endl;
  std::cout << "[CCamera]         - Hours    : " << Buffer->timecode.hours << std::endl;
  std::cout << "[CCamera]         - Userbits : '" << Buffer->timecode.userbits << "'" << std::endl;

  std::cout << "[CCamera]     - Sequence     : " << Buffer->sequence << std::endl;
  std::cout << "[CCamera]     - Memory       : " << Buffer->memory << std::endl;
  std::cout << "[CCamera]     - Offset       : " << Buffer->m.offset << std::endl;
  std::cout << "[CCamera]     - Length       : " << Buffer->length << std::endl;
  //   qDebug() <<(QString("3.     - Index        : ") + QString::number(Buffer->index));
  //   qDebug() <<(QString("3.     - Type         : ") + QString::number(Buffer->type));
  //   qDebug() <<(QString("3.     - Bytes Used   : ") + QString::number(Buffer->bytesused));
  //   qDebug() <<(QString("3.     - Flags        : ") + QString::number(Buffer->flags));
  //   qDebug() <<(QString("3.     - Field        : ") + QString::number(Buffer->field));
  //
  //   qDebug() <<(QString("3.     - Time Stamp   :"));
  //   qDebug() <<(QString("3.         - Tv Sec   : ") + QString::number(Buffer->timestamp.tv_sec));
  //   qDebug() <<(QString("3.         - Tv USec  : ") + QString::number(Buffer->timestamp.tv_usec));
  //
  //   qDebug() <<(QString("3.     - Time Code    :"));
  //   qDebug() <<(QString("3.         - Type     : ") + QString::number(Buffer->timecode.type));
  //   qDebug() <<(QString("3.         - Flags    : ") + QString::number(Buffer->timecode.flags));
  //   qDebug() <<(QString("3.         - Frames   : ") + QString::number(Buffer->timecode.frames));
  //   qDebug() <<(QString("3.         - Seconds  : ") + QString::number(Buffer->timecode.seconds));
  //   qDebug() <<(QString("3.         - Minutes  : ") + QString::number(Buffer->timecode.minutes));
  //   qDebug() <<(QString("3.         - Hours    : ") + QString::number(Buffer->timecode.hours));
  //   qDebug() <<(QString("3.         - Userbits : '") + QString((char *)Buffer->timecode.userbits) + QString("'"));
  //
  //   qDebug() <<(QString("3.     - Sequence     : ") + QString::number(Buffer->sequence));
  //   qDebug() <<(QString("3.     - Memory       : ") + QString::number(Buffer->memory));
  //   qDebug() <<(QString("3.     - Offset       : ") + QString::number(Buffer->m.offset));
  //   qDebug() <<(QString("3.     - Length       : ") + QString::number(Buffer->length));
}

//! Init data
void CCamera::initData(void)
{
  Size = Width * Height * Depth;
  FrameCount = 0;
}

//! Free an object
void CCamera::free(void)
{
  if (Fd > 0)
  {
    videoStreamOff();

    // Alliberar els Buffers
    unmapBuffers(BufferPointer, Buffer.count);
    freeKernelBuffers(&Buffer);

    if (close(Fd) != 0)
    {
      Fd = -1;
      std::cerr << "[CCamera - Error] file error: closing" << std::endl;
      //         qDebug() <<(QString("3. file error: closing"));
      exit(EXIT_FAILURE);
    }
  }
}

//! Create an object
void CCamera::allocate(const std::string &ConfigFileName, int videoInput, int width, int height, int videoBrightness,
                       int videoContrast, int videoSaturation, int videoHue, bool color, std::string pixFmt,
                       std::string fieldType_, bool verbose)
{
  int i;

  Width = width;
  Height = height;
  pixelFormat = pixFmt;
  fieldType = fieldType_;

  DeviceFile = ConfigFileName;

  initData();

  if (Fd < 0)
  {
    // Obrir el dispositiu
    if ((Fd = open(DeviceFile.c_str(), O_RDWR)) >= 0)
    {
      if (verbose) {
        std::cout << "[CCamera] * File Name : '" << DeviceFile << "'" << std::endl;
        std::cout << "[CCamera] * File Handle : '" << Fd << std::endl;
        // qDebug() <<(QString("3. * File Name : '") + DeviceFile + QString("'"));
        // qDebug() <<(QString("3. * File Handle : ") + QString::number(Fd));
      }
      // setOverlay();

      if (verbose)
      {
        queryCapabilities();
        enumInputs();
        enumControls();

        getVideoFormat();
        enumFormats();
      }
      setVideoFormat(color);
      if (verbose)
      {
        getVideoFormat();
      }

      // input 0 --> television
      // input 1 --> Composite 1
      // input 2 --> S-Video
      setVideoInput(videoInput);  // in DazzleDVC100, Composite1 is 0

      // Set Brightess
      setVideoBrightness(videoBrightness);

      // Set Contrast
      setVideoContrast(videoContrast);

      // Set Saturation
      setVideoSaturation(videoSaturation);

      // Set Hue
      setVideoHue(videoHue);

      // Set Auto White Balance
      setAutoWhiteBalance(true);

      // Set Auto Gain
      setAutoGain(true);

      requestBuffers(&BufferPointer, &Buffer, NumBuffers);

      if (BufferPointer == NULL)
      {
        std::cout << "[CCamera] * Null BufferPointer!" << std::endl;
        // qDebug() <<(QString("3. * Null BufferPointer!"));
        exit(EXIT_FAILURE);
      }

      for (i = 0; i < static_cast<int>(NumBuffers); i++)
      {
        enqueueBuffer(i);
        if (verbose)
        {
          printBufferPointer(&(BufferPointer[i]));
        }
      }

      setVideoStandard(color);

      if (verbose)
      {
        std::cout << "[CCamera] * Grey Format code: 0xGREY" << std::endl;
        std::cout << "[CCamera] * RGB3 Format code: 0xRGB3" << std::endl;
        std::cout << "[CCamera] * RGB4 Format code: 0xRGB4" << std::endl;
        std::cout << "[CCamera] * RGBO Format code: 0xRGB0" << std::endl;
        // qDebug() <<(QString("3. * Grey Format code: 0x%1").arg(v4l2_fourcc('G', 'R', 'E', 'Y'), 0, 16));
        // qDebug() <<(QString("3. * RGB3 Format code: 0x%1").arg(v4l2_fourcc('R', 'G', 'B', '3'), 0, 16));
        // qDebug() <<(QString("3. * RGB4 Format code: 0x%1").arg(v4l2_fourcc('R', 'G', 'B', '4'), 0, 16));
        // qDebug() <<(QString("3. * RGBO Format code: 0x%1").arg(v4l2_fourcc('R', 'G', 'B', 'O'), 0, 16));
      }

      videoStreamOn();
    }
    else
    {
      std::cout << "[CCamera] file error: opening file '" << DeviceFile << "'" << std::endl;
      // qDebug() <<(QString("3. file error: opening file '") + DeviceFile + QString("'") );
    }
  }
  else
  {
    std::cout << "[CCamera] * Camera is init!" << std::endl;
    // qDebug() <<(QString("3. * Camera is init!"));
  }
}

//! Builder with the name of the configuration file.
CCamera::CCamera(const std::string &ConfigFileName, int videoInput, int width, int height, int videoBrightness,
                 int videoContrast, int videoSaturation, int videoHue, bool color, std::string pixFmt,
                 std::string fieldType_, bool verbose)
{
  Fd = -1;
  BufferPointer = NULL;
  DeviceFile = "";

  allocate(ConfigFileName, videoInput, width, height, videoBrightness, videoContrast, videoSaturation, videoHue, color,
           pixFmt, fieldType_, verbose);    // Crear objecte
}

//! Destroyer.
CCamera::~CCamera(void)
{
  free();
}

//! Start/Restart the camera with a configuration camera.
void CCamera::init(const std::string &ConfigFileName, int videoInput, int width, int height, int videoBrightness,
                   int videoContrast, int videoSaturation, int videoHue, bool color, std::string pixFmt,
                   std::string fieldType_, bool verbose)
{
  allocate(ConfigFileName, videoInput, width, height, videoBrightness, videoContrast, videoSaturation, videoHue, color,
           pixFmt, fieldType_, verbose);    // Create the object
}

//! Adquire the data buffer.
unsigned char* CCamera::grab(void)
{
  dequeueBuffer(&DataBuffer);

  if (verbose)
    printBufferPointer(&(BufferPointer[DataBuffer.index]));

  return(unsigned char *)BufferPointer[DataBuffer.index].Start;
}

//! Freeing the buffer after the process of the image.
void CCamera::grabRelease(void)
{
  enqueueBuffer(DataBuffer.index);
  FrameCount++;
}

//! Check if the initialization was good.
bool CCamera::isInit(void) const
{
  return Fd >= 0;
}

//! Return the with of the image.
int CCamera::width(void) const
{
  return Width;
}

//! Return the height of the image.
int CCamera::height(void) const
{
  return Height;
}

//! Return the number of bytes for pixel
int CCamera::bytesPerPixel(bool color) const
{
  if (color)
    return 4;
  else
    return 1;
}

//! Return the size of the buffer
int CCamera::size(void) const
{
  return Size;
}

//! Return the name of the adquired frame
int CCamera::grabFramesCount(void) const
{
  return FrameCount;
}

//! Return the number of devices
int CCamera::nDevices(void) const
{
  std::string devname;
  struct stat devstat;
  int i;
  int ndevices = 0;

  for (i = 0; i < 64; i++)
  {
    devname = "/dev/video%d" + i;
    if (stat(devname.c_str(), &devstat) == 0)
    {
      ndevices++;
    }
  }
  return ndevices;
}

//! Definir the camera brightness
void CCamera::setVideoBrightness(int InputBrightness)
{
  struct v4l2_control s;
  s.id = V4L2_CID_BRIGHTNESS;
  s.value = InputBrightness;

  std::cout << "[CCamera]     - SetVideoBrightness = " << InputBrightness << std::endl;

  if (ioctl(Fd, VIDIOC_S_CTRL, &s) == -1)
    std::cerr << "[CCamera - Error] * Device Set Brightness error!" << std::endl;
  // qDebug() <<(QString("3. * Device Set Brightness error!"));
}

//! Definir the camera contrast
void CCamera::setVideoContrast(int InputContrast)
{
  struct v4l2_control s;
  s.id = V4L2_CID_CONTRAST;
  s.value = InputContrast;

  std::cout << "[CCamera]     - SetVideoContrast = " << InputContrast << std::endl;

  if (ioctl(Fd, VIDIOC_S_CTRL, &s) == -1)
    std::cerr << "[CCamera - Error] * Device Set Contrast error!" << std::endl;
  // qDebug() <<(QString("3. * Device Set Contrast error!"));
}

//! Define the camera Saturation
void CCamera::setVideoSaturation(int InputSaturation)
{
  struct v4l2_control s;
  s.id = V4L2_CID_SATURATION;
  s.value = InputSaturation;

  std::cout << "[CCamera]     - SetVideoSaturation = " << InputSaturation << std::endl;

  if (ioctl(Fd, VIDIOC_S_CTRL, &s) == -1)
    std::cerr << "[CCamera - Error] * Device Set Saturation error!" << std::endl;
  // qDebug() <<(QString("3. * Device Set Saturation error!"));
}

//! Set UV ratio
void CCamera::setVideoUvRatio(int UVRatio)
{
  struct v4l2_control s;
  s.id = V4L2_CID_PRIVATE_UV_RATIO;
  s.value = UVRatio;

  std::cout << "[CCamera]     - SetVideoUvRatio = " << UVRatio << std::endl;

  if (ioctl(Fd, VIDIOC_S_CTRL, &s) == -1)
    std::cerr << "[CCamera - Error] * Device Set UV Ratio error!" << std::endl;
  // qDebug() <<(QString("3. * Device Set Hue error!"));
}


//! Define the camera Hue
void CCamera::setVideoHue(int InputHue)
{
  struct v4l2_control s;
  s.id = V4L2_CID_HUE;
  s.value = InputHue;

  std::cout << "[CCamera]     - SetVideoHue = " << InputHue << std::endl;

  if (ioctl(Fd, VIDIOC_S_CTRL, &s) == -1)
    std::cout << "[CCamera - Error] * Device Set Hue error!" << std::endl;
  // qDebug() <<(QString("3. * Device Set Hue error!"));
}

//! Set Auto White Balance
void CCamera::setAutoWhiteBalance(bool AutoWhite)
{
  struct v4l2_control s;
  s.id = V4L2_CID_AUTO_WHITE_BALANCE;
  s.value = AutoWhite;

  if (ioctl(Fd, VIDIOC_S_CTRL, &s) == -1)
    std::cerr << "[CCamera - Error] * Device Set Auto White Balance error!" << std::endl;
  // qDebug() <<(QString("3. * Device Set Auto White Balance error!"));
}

//! Set Auto Gain
void CCamera::setAutoGain(bool AutoGain)
{
  struct v4l2_control s;
  s.id = V4L2_CID_AUTOGAIN;
  s.value = AutoGain;

  if (ioctl(Fd, VIDIOC_S_CTRL, &s) == -1)
    std::cerr << "[CCamera - Error] * Device Set Auto Gain error!" << std::endl;
  // qDebug() <<(QString("3. * Device Set Auto Gain error!"));
}

}  // namespace io
}  // namespace cola2


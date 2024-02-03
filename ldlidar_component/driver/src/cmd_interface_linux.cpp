//  Copyright 2022 Walter Lucetti
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
////////////////////////////////////////////////////////////////////////////////

#include "cmd_interface_linux.hpp"

#include <sys/file.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <string.h>
#include <memory.h>
#include <libudev.h>

#include <iostream>

#define MAX_ACK_BUF_LEN 2304000

CmdInterfaceLinux::CmdInterfaceLinux()
: _rxThread(nullptr),
  _rxCount(0),
  _readCallback(nullptr)
{
  _comHandle = -1;
}


CmdInterfaceLinux::~CmdInterfaceLinux()
{
  Close();
}


bool CmdInterfaceLinux::Open(std::string & port_name)
{
  int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

  _comHandle = open(port_name.c_str(), flags);
  if (-1 == _comHandle) {
    std::cerr << "CmdInterfaceLinux::Open open error!" << std::endl;
    return false;
  }

  // get port options
  struct termios options;
  if (-1 == tcgetattr(_comHandle, &options)) {
    Close();
    std::cerr << "CmdInterfaceLinux::Open tcgetattr error!" << std::endl;
    return false;
  }

  options.c_cflag |= (tcflag_t)(CLOCAL | CREAD | CS8 | CRTSCTS);
  options.c_cflag &= (tcflag_t) ~(CSTOPB | PARENB | PARODD);
  options.c_lflag &= (tcflag_t) ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG |
    IEXTEN);  // |ECHOPRT
  options.c_oflag &= (tcflag_t) ~(OPOST);
  options.c_iflag &= (tcflag_t) ~(IXON | IXOFF | INLCR | IGNCR | ICRNL | IGNBRK);

  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;

  cfsetispeed(&options, B230400);

  if (tcsetattr(_comHandle, TCSANOW, &options) < 0) {
    std::cerr << "CmdInterfaceLinux::Open tcsetattr error!" << std::endl;
    Close();
    return false;
  }

  tcflush(_comHandle, TCIFLUSH);

  _rxThreadExitFlag = false;
  _rxThread = new std::thread(_rxThreadProc, this);
  _isCmdOpened = true;

  return true;
}


bool CmdInterfaceLinux::Close()
{
  if (_isCmdOpened == false) {
    return true;
  }

  _rxThreadExitFlag = true;

  if (_comHandle != -1) {
    close(_comHandle);
    _comHandle = -1;
  }

  if ((_rxThread != nullptr) && _rxThread->joinable()) {
    _rxThread->join();
    delete _rxThread;
    _rxThread = NULL;
  }

  _isCmdOpened = false;

  return true;
}


bool CmdInterfaceLinux::GetCmdDevices(
  std::vector<std::pair<std::string,
  std::string>> & device_list)
{
  struct udev * udev;
  struct udev_enumerate * enumerate;
  struct udev_list_entry * devices, * dev_list_entry;
  struct udev_device * dev;

  udev = udev_new();
  if (!udev) {
    return false;
  }
  enumerate = udev_enumerate_new(udev);
  udev_enumerate_add_match_subsystem(enumerate, "tty");
  udev_enumerate_scan_devices(enumerate);
  devices = udev_enumerate_get_list_entry(enumerate);
  udev_list_entry_foreach(dev_list_entry, devices)
  {
    const char * path;

    path = udev_list_entry_get_name(dev_list_entry);
    dev = udev_device_new_from_syspath(udev, path);
    std::string dev_path = std::string(udev_device_get_devnode(dev));
    dev = udev_device_get_parent_with_subsystem_devtype(dev, "usb", "usb_device");
    if (dev) {
      std::pair<std::string, std::string> p;
      p.first = dev_path;
      p.second = udev_device_get_sysattr_value(dev, "product");
      device_list.push_back(p);
      udev_device_unref(dev);
    } else {
      continue;
    }
  }
  udev_enumerate_unref(enumerate);
  udev_unref(udev);
  return true;
}

bool CmdInterfaceLinux::ReadFromIO(uint8_t * rx_buf, uint32_t rx_buf_len, uint32_t * rx_len)
{
  static timespec timeout = {0, static_cast<int>(100 * 1e6)};
  int32_t len = -1;

  if (IsOpened()) {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(_comHandle, &read_fds);
    int r = pselect(_comHandle + 1, &read_fds, NULL, NULL, &timeout, NULL);
    if (r < 0) {
      // Select was interrupted
      if (errno == EINTR) {
        return false;
      }
    } else if (r == 0) {  // timeout
      return false;
    }

    if (FD_ISSET(_comHandle, &read_fds)) {
      len = (int32_t)read(_comHandle, rx_buf, rx_buf_len);
      if ((len != -1) && rx_len) {
        *rx_len = len;
      }
    }
  }
  return len == -1 ? false : true;
}


bool CmdInterfaceLinux::WriteToIo(const uint8_t * tx_buf, uint32_t tx_buf_len, uint32_t * tx_len)
{
  int32_t len = -1;

  if (IsOpened()) {
    len = (int32_t)write(_comHandle, tx_buf, tx_buf_len);
    if ((len != -1) && tx_len) {
      *tx_len = len;
    }
  }
  return len == -1 ? false : true;
}


void CmdInterfaceLinux::_rxThreadProc(void * param)
{
  CmdInterfaceLinux * cmd_if = reinterpret_cast<CmdInterfaceLinux *>(param);
  char * rx_buf = new char[MAX_ACK_BUF_LEN + 1];
  while (!cmd_if->_rxThreadExitFlag.load()) {
    uint32_t readed = 0;
    bool res = cmd_if->ReadFromIO(reinterpret_cast<uint8_t *>(rx_buf), MAX_ACK_BUF_LEN, &readed);
    if (res && readed) {
      cmd_if->_rxCount += readed;
      if (cmd_if->_readCallback != nullptr) {
        cmd_if->_readCallback(rx_buf, readed);
      }
    }
  }

  delete[] rx_buf;
}

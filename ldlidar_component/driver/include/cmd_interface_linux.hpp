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

#ifndef CMD_INTERFACE_LINUX_HPP_
#define CMD_INTERFACE_LINUX_HPP_

#include <inttypes.h>
#include <string.h>

#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <utility>
#include <functional>
#include <string>
#include <condition_variable>

class CmdInterfaceLinux
{
public:
  CmdInterfaceLinux();
  ~CmdInterfaceLinux();

  bool Open(std::string & port_name);
  bool Close();
  bool ReadFromIO(uint8_t * rx_buf, uint32_t rx_buf_len, uint32_t * rx_len);
  bool WriteToIo(const uint8_t * tx_buf, uint32_t tx_buf_len, uint32_t * tx_len);
  bool GetCmdDevices(std::vector<std::pair<std::string, std::string>> & device_list);
  void SetReadCallback(std::function<void(const char *, size_t length)> callback)
  {
    mReadCallback = callback;
  }
  bool IsOpened() {return mIsCmdOpened.load();}

private:
  std::thread * mRxThread;
  static void mRxThreadProc(void * param);
  int64_t mRxCount;
  int32_t mComHandle;
  std::atomic<bool> mIsCmdOpened, mRxThreadExitFlag;
  std::function<void(const char *, size_t length)> mReadCallback;
};

#endif  // CMD_INTERFACE_LINUX_HPP_

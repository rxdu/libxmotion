/*
Ping指令测试,测试总线上相应ID舵机是否就绪,广播指令只适用于总线只有一个舵机情况
*/

#include <iostream>
#include <thread>
#include <chrono>

#include "SCServo.h"

SMS_STS sm_st;

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cout << "argc error!" << std::endl;
    return 0;
  }
  std::cout << "serial:" << argv[1] << std::endl;
  if (!sm_st.begin(1000000, argv[1])) {
    std::cout << "Failed to init sms/sts motor!" << std::endl;
    return 0;
  }

  for (int i = 0; i < 255; i++) {
    int ID = sm_st.Ping(i);
    if (ID != -1) {
      std::cout << "ID:" << ID << std::endl;
      break;
    } else {
      std::cout << "Ping servo ID error: " << i << " !" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  sm_st.end();
  return 1;
}

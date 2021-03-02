//
// Created by allen on 2020/11/2.
//

#ifndef MY_PCL_TRAVPROCESS_H
#define MY_PCL_TRAVPROCESS_H
// headers
#include <iostream>
#include <vector>        //提供向量头文件
#include <algorithm>     // 算法头文件，提供迭代器
#include <sstream>
#include <iomanip>       //C++输出精度控制需要
#include <thread>

class Handler {
public:
    ~Handler() {}

    void heighthandler(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const heightnew_t *msg) {
        (void) rbuf;
        (void) chan;

        for (size_t i(0); i < xnew_size; ++i) {
            for (size_t j(0); j < ynew_size; ++j) {
                _height_mapnew(i, j) = msg->map[i][j];
            }
        }

    }
    void LCMThread() { while (true) { lcm.handle(); } }
};
lcm::LCM lcm;
std::thread _LCMThread;
size_t xnew_size = 101;
size_t ynew_size = 101;

DMat<float> _height_mapnew;
#endif //MY_PCL_TRAVPROCESS_H

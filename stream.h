//
// Created by dell on 2022/3/19.
//

#ifndef CPOD_STREAM_H
#define CPOD_STREAM_H
#include "cpod.h"
#include "vector"

vector<Point> get_incoming_data(int current_time, int pipe_fd, int length, std::ifstream& file, string delimiter);

#endif //CPOD_STREAM_H

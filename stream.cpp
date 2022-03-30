//
// Created by dell on 2022/3/19.
//

#include "cpod.h"
#include <fstream>
#include "iostream"
#include "stream.h"

vector<Point> get_incoming_data(int current_time, int length, std::ifstream& file, string delimiter) {
    vector<Point> datas;
    for(int i=0;i<length;i++) {
        string line;
        size_t pos = 0;
        std::string token;
        if(!getline(file, line)) break;
        vector<double> vals;
        while ((pos = line.find(delimiter)) != std::string::npos) {
            token = line.substr(0, pos);
            vals.push_back(atof(token.c_str()));
            line.erase(0, pos + delimiter.length());
        }
        token = line.substr(0, pos);
        vals.push_back(atof(token.c_str()));
        Point data(vals);
        data.arrival_time = current_time + i;
        datas.push_back(data);
    }
    return datas;
}

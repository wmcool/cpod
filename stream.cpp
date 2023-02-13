//
// Created by dell on 2022/3/19.
//

#include "cpod.h"
#include <fstream>
#include <sys/unistd.h>
#include "iostream"
#include "stream.h"

std::string pipe_line;

vector<Point> get_incoming_data(int current_time, int pipe_fd, int length, std::ifstream& file, string delimiter) {
    vector<Point> datas;
    string line;
    while(true) {
        size_t pos = 0;
        std::string token;
        char buffer[71];
        vector<double> vals;
        int count;
        int valread = read(pipe_fd, buffer, 71);
        if(valread > 0) {
            for(int i=0;i<valread;i++) {
                if(buffer[i] == '/n') {
//                        while ((pos = pipe_line.find(delimiter)) != std::string::npos) {
//                            token = pipe_line.substr(0, pos);
//                            vals.push_back(atof(token.c_str()));
//                            pipe_line.erase(0, pos + delimiter.length());
//                        }
                    cout << "recv:" << pipe_line << endl;
                    if(!getline(file, line)) break;
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
                    count++;
                    pipe_line = "";
                } else {
                    pipe_line += buffer[i];
                }
            }
        }
        if(count == length) break;
    }
    return datas;
}

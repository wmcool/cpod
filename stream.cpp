//
// Created by dell on 2022/3/19.
//

#include "cpod.h"
#include <fstream>
#include "sstream"
#include "iostream"

vector<Data> get_incoming_data(int current_time, int length, std::ifstream& file) {
    vector<Data> datas;
    for(int i=0;i<length;i++) {
        string line;
        while(getline(file, line)) {
            std::istringstream iss(line);
            vector<double> vals;
            double d;
            while(iss >> d) vals.push_back(d);
            Data data(vals);
            data.arrival_time = current_time + i;
            datas.push_back(data);
        }
    }
    return datas;
}

int main() {
    std::ifstream file("tao.txt");
    vector<Data> datas = get_incoming_data(0, 10, file);
    for(int i=0;i<datas.size();i++) {
        Data data = datas[i];
        cout << data.arrival_time << endl;
    }
}

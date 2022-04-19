#include <iostream>
#include "cpod.h"
#include "vector"
#include "include/cmdline.h"
#include <fstream>
#include "stream.h"
#include "include/json.hpp"
#include "include/mysocket.h"


using namespace std;

int main(int argc, char *argv[]) {
    cmdline::parser parser;
    parser.add<double>("radius", 'R', "radius", false, 1.0);
    parser.add<int>("window", 'W', "window size", false, 20);
    parser.add<int>("K", 'K', "K", false, 5);
    parser.add<int>("slide", 'S', "slide size", false, 5);
    parser.add<int>("num_window", ' ', "slide size", false, -1);
    parser.add<string>("filename", 'f', "filename", false, "");
    parser.parse_check(argc, argv);
    R = parser.get<double>("radius");
    WINDOW = parser.get<int>("window");
    K = parser.get<int>("K");
    SLIDE = parser.get<int>("slide");
    string file_name = parser.get<string>("filename");
    ifstream in(file_name);
    current_time = 0;
    int num_windows = parser.get<int>("num_window");
    int current_window = 0;
    mtree = new MTreeCorePoint();
    int sock = init_socket();
    if(in.is_open()) {
        while(true) {
            nlohmann::json j;
            if(num_windows != -1 && current_window > num_windows) break;
            current_window++;
//            cout << "Num window: " << current_window << endl;
            vector<Point> incoming_data;
            if(current_time != 0) {
                incoming_data = get_incoming_data(current_time, SLIDE, in, ",");
                current_time += SLIDE;
            } else {
                incoming_data = get_incoming_data(current_time, WINDOW, in, ",");
                current_time += WINDOW;
            }
            vector<Point> outliers = detect_outlier(incoming_data, current_time, WINDOW, SLIDE);
            if(outliers.empty()) continue;
//            cout << "Num outliers = " << outliers.size() << endl;
            for(int i=0;i<outliers.size();i++) {
                stringstream ss;
                ss << outliers[i].arrival_time;
                j["outlier"][ss.str()] = outliers[i];
            }
            string s = j.dump();
            send(sock, s.c_str(), s.size(), 0);
//            cout << s << endl;
        }
    }
    for(auto & i : all_distinct_cores) {
        delete i;
    }
    delete mtree;
    return 0;
}

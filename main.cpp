#include <iostream>
#include "cpod.h"
#include "vector"
#include "include/cmdline.h"
#include <fstream>
#include "stream.h"


using namespace std;

extern MTreeCorePoint* mtree;

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
    string file_name = "tao.txt";
    ifstream in(file_name);
    bool stop = false;
    int current_time = 0;
    int num_windows = parser.get<int>("num_window");
//    int num_windows = 5;
    int current_window = 0;
    mtree = new MTreeCorePoint();
    if(in.is_open()) {
        while(!stop) {
            if(num_windows != -1 && current_window > num_windows) break;
            current_window++;
            cout << "Num window: " << current_window << endl;
            vector<Point> incoming_data;
            if(current_time != 0) {
                incoming_data = get_incoming_data(current_time, SLIDE, in, ",");
                current_time += SLIDE;
            } else {
                incoming_data = get_incoming_data(current_time, WINDOW, in, ",");
                current_time += WINDOW;
            }
            vector<Point> outliers = detect_outlier(incoming_data, current_time, WINDOW, SLIDE);
            cout << "Num outliers = " << outliers.size() << endl;
        }
    }
    delete mtree;
    return 0;
}

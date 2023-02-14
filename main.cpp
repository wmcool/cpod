#include <iostream>
#include "cpod.h"
#include "vector"
#include "include/cmdline.h"
#include <sys/fcntl.h>
#include <fstream>
#include "stream.h"
#include "include/mysocket.h"
#include <chrono>
#include <unistd.h>
#include "include/json.hpp"
#include <ios>
#include <sys/stat.h>
#include <unistd.h>


using namespace std;

void process_mem_usage(double& vm_usage, double& resident_set)
{
    using std::ios_base;
    using std::ifstream;
    using std::string;

    vm_usage     = 0.0;
    resident_set = 0.0;

    // 'file' stat seems to give the most reliable results
    //
    ifstream stat_stream("/proc/self/stat",ios_base::in);

    // dummy vars for leading entries in stat that we don't care about
    //
    string pid, comm, state, ppid, pgrp, session, tty_nr;
    string tpgid, flags, minflt, cminflt, majflt, cmajflt;
    string utime, stime, cutime, cstime, priority, nice;
    string O, itrealvalue, starttime;

    // the two fields we want
    //
    unsigned long vsize;
    long rss;

    stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr
                >> tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt
                >> utime >> stime >> cutime >> cstime >> priority >> nice
                >> O >> itrealvalue >> starttime >> vsize >> rss; // don't care about the rest

    stat_stream.close();

    long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024; // in case x86-64 is configured to use 2MB pages
    vm_usage     = vsize / 1024.0;
    resident_set = rss * page_size_kb;
}

int main(int argc, char *argv[]) {
    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::milliseconds;
    cmdline::parser parser;
    cout << "开始运行CPOD算法..." << endl;
    parser.add<double>("radius", 'R', "radius", false, 1.0);
    parser.add<int>("window", 'W', "window size", false, 20);
    parser.add<int>("K", 'K', "K", false, 5);
    parser.add<int>("slide", 'S', "slide size", false, 5);
    parser.parse_check(argc, argv);
    R = parser.get<double>("radius");
    WINDOW = parser.get<int>("window");
    K = parser.get<int>("K");
    SLIDE = parser.get<int>("slide");
    current_time = 0;
    int current_window = 0;
    mtree = new MTreeCorePoint();
    int sock = init_socket();
    duration<double, std::milli> all_cost;
    double max_vm;
    int num_windows = 100000;

    // 初始化管道
    if((mkfifo("/tmp/nda",O_CREAT|O_EXCL)<0)&&(errno!=EEXIST))
        printf("cannot create fifo\n");
    if(errno==ENXIO){
        printf("open error; no reading process\n");
        return 0;
    }
    int fifo_fd = 0;
    fifo_fd = open("/tmp/nda",O_RDONLY, 0);
    if(fifo_fd <= 0) {
        printf("open fifo failed");
        return 0;
    }

    while(true) {
        nlohmann::json j;
        if(current_window > num_windows) break;
        current_window++;
//            cout << "Num window: " << current_window << endl;
        vector<Point> incoming_data;
        if(current_time != 0) {
            incoming_data = get_incoming_data(current_time, fifo_fd, SLIDE, ",");
            current_time += SLIDE;
        } else {
            incoming_data = get_incoming_data(current_time, fifo_fd, WINDOW, ",");
            current_time += WINDOW;
        }
//        cout << "data size:" << incoming_data.size() << endl;
//        cout << "first data: ";
//        for(int i=0;i<incoming_data[0].values.size();i++) {
//            cout << incoming_data[0].values[i];
//            if(i != incoming_data[0].values.size() - 1) cout << ",";
//        }
//        cout << endl;
        vector<Point> outliers = detect_outlier(incoming_data, current_time, WINDOW, SLIDE);
        double vm, rss;
        process_mem_usage(vm, rss);
        if(vm > max_vm) max_vm = vm;
//        cout << "VM: " << vm << "KB" << endl;
//        cout << "Num outliers = " << outliers.size() << endl;
        if(outliers.empty()) continue;
        for(int i=0;i<outliers.size();i++) {
            stringstream ss;
            ss << outliers[i].timestamp;
            cout << outliers[i].timestamp << endl;
            j["outlier"][ss.str()] = outliers[i];
        }
        string s = j.dump() + "\n";
        send(sock, s.c_str(), s.size(), 0);
//        cout << s;
//            close(sock);
    }
}


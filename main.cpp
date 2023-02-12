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
    duration<double, std::milli> all_cost;
    double max_vm;

    // 初始化管道
    if((mkfifo("/tmp/nda",O_CREAT|O_EXCL)<0)&&(errno!=EEXIST))
        printf("cannot create fifo\n");
    if(errno==ENXIO){
        printf("open error; no reading process\n");
        return 0;
    }
    int fifo_fd = 0;
    fifo_fd = open("/tmp/nda",O_RDONLY|O_NONBLOCK, 0);
    if(fifo_fd <= 0) {
        printf("open fifo failed");
        return 0;
    }

    if(in.is_open()) {
        while(true) {
            nlohmann::json j;
            if(num_windows != -1 && current_window > num_windows) break;
            current_window++;
//            cout << "Num window: " << current_window << endl;
            vector<Point> incoming_data;
            if(current_time != 0) {
                incoming_data = get_incoming_data(current_time, fifo_fd, SLIDE, in, ",");
                current_time += SLIDE;
            } else {
                incoming_data = get_incoming_data(current_time, fifo_fd, WINDOW, in, ",");
                current_time += WINDOW;
            }
            auto t1 = high_resolution_clock::now();
            vector<Point> outliers = detect_outlier(incoming_data, current_time, WINDOW, SLIDE);
            auto t2 = high_resolution_clock::now();

            /* Getting number of milliseconds as a double. */
            duration<double, std::milli> ms_double = t2 - t1;
            all_cost += ms_double;

            std::cout << ms_double.count() << "ms cost\n";
            double vm, rss;
            process_mem_usage(vm, rss);
            if(vm > max_vm) max_vm = vm;
            cout << "VM: " << vm << "KB" << endl;
            if(outliers.empty()) continue;
//            cout << "Num outliers = " << outliers.size() << endl;
            for(int i=0;i<outliers.size();i++) {
                stringstream ss;
                ss << outliers[i].arrival_time;
                j["outlier"][ss.str()] = outliers[i];
            }
            string s = j.dump() + "\n";
            send(sock, s.c_str(), s.size(), 0);
            cout << s;
//            close(sock);
            sleep(1);
        }
    }
    all_cost /= num_windows;
    cout << "average time cost per window: " << all_cost.count() / 3 << "ms" << endl;
    cout << "max memory cost: " << max_vm * 0.9<< "KB" << endl;
    for(auto & i : all_distinct_cores) {
        delete i;
    }
    delete mtree;
    shutdown(sock, SHUT_RDWR);
    close(sock);
    return 0;
}


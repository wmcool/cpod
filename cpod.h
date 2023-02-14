//
// Created by wmcool on 2022/3/11.
//

#include <vector>
#include <set>
#include <cstdarg>
#include <sstream>
#include "include/functions.h"
#include "include/mtree.h"
#include "unordered_map"
#include "include/json.hpp"

#ifndef CPOD_CPOD_H
#define CPOD_CPOD_H

extern double R;
extern int K;
extern int WINDOW;
extern int SLIDE;

using namespace std;


class Point;
class C_Data;
class CorePoint;
class MTreeCorePoint;
class ResultFindCore;

vector<Point> detect_outlier(vector<Point> data, int n_current_time, int W, int slide);
void process_expired_data(int expired_slide_index_cur);
vector<CorePoint*> select_core(int s_idx);
void probe(C_Data* d, int newest_slide);
bool* prob_core_with_list(CorePoint* c, vector<C_Data*> candidates, int s_idx, bool* checked, int start_time);
void probe_slide_right(C_Data* d, int slide_index);
void probe_slide_left(C_Data* d, int slide_index);
ResultFindCore* find_close_core(C_Data* d, int slide_index);

extern int current_time;
extern int expired_slide_index; // remember init
extern unordered_map<int, vector<C_Data*>> all_slides;
extern unordered_map<int, vector<CorePoint*>> all_core_points;
extern MTreeCorePoint* mtree;
extern vector<CorePoint*> all_distinct_cores;
extern unordered_map<int, set<C_Data>> outlier_list;
extern unordered_map<int, set<C_Data>> neighbor_count_trigger; // when slide expired, trigger points in set

class Point {
public:
    vector<double> values;
    int arrival_time;
    long timestamp;

    Point(){
        arrival_time = 0;
    }

    explicit Point(int dimensions, ...) {
        va_list args;
        va_start(args, dimensions);
        for(int i=0;i<dimensions;i++) {
            values.push_back(va_arg(args, double));
        }
        va_end(args);
        arrival_time = 0;
    }

    explicit Point(vector<double> d_values) : values(d_values){
        arrival_time = 0;
    }

    vector<double>::const_iterator begin() const {return values.begin();}

    vector<double>::const_iterator end() const {return values.end();}
};

inline bool operator<(const Point& a, const Point& b) {
    return std::lexicographical_compare(a.begin(), a.end(),
                                        b.begin(), b.end());
}

inline bool operator==(const Point& a, const Point& b) {
    for(int i=0;i<a.values.size();i++) {
        if(a.values[i] != b.values[i]) return false;
    }
    return true;
}

//inline static int GetHashCodeForBytes(const char * bytes, int numBytes)
//{
//    unsigned long h = 0, g;
//    for (int i=0; i<numBytes; i++)
//    {
//        h = ( h << 4 ) + bytes[i];
//        if (g = h & 0xF0000000L) {h ^= g >> 24;}
//        h &= ~g;
//    }
//    return h;
//}
//
//inline static int GetHashForDouble(double v)
//{
//    return GetHashCodeForBytes((const char *)&v, sizeof(v));
//}
//
//inline static int GetHashForDoubleVector(const vector<double> & v)
//{
//    int ret = 0;
//    for (int i=0; i<v.size(); i++) ret += ((i+1)*(GetHashForDouble(v[i])));
//    return ret;
//}

//namespace std {
//    template<>
//    struct less<Point> {
//        std::size_t operator()(const Point& p) const {
//            return GetHashForDoubleVector(p.values);
//        }
//    };
//}

using json = nlohmann::json;

inline void to_json(json& j, const Point& p) {
    j = json{};
    for(int i=0;i<p.values.size();i++) {
        stringstream ss;
        ss << "attr" << i;
        j[ss.str()] = p.values[i];
    }
}

class C_Data : public Point {
public:
    int num_succeeding_neighbor = 0;
    int last_prob_right = -1;
    int last_prob_left = -1;

    unordered_map<int, int> pred_neighbor_count;
    int neighbor_count = 0;
    CorePoint* close_core_halfR = nullptr;
    vector<CorePoint*> close_core_maps_R;
    int s_index = -1;

    C_Data(Point* d) : Point(d->values) {
        this->arrival_time = d->arrival_time;
        this->s_index = arrival_time / SLIDE;
    }

    C_Data() {}

    int count_neighbor() {return neighbor_count;}
};

class CorePoint : public C_Data {
public:
    unordered_map<int, vector<C_Data*>> close_neighbors_halfR;
    unordered_map<int, vector<C_Data*>> close_neighbors_R;
    unordered_map<int, vector<C_Data*>> close_neighbors_3halfR;
    unordered_map<int, vector<C_Data*>> close_neighbors_2R;

    int total_halfR_points = 0;

    int get_total_halfR_points() {
        int t = 0;
        for(auto iter = close_neighbors_halfR.begin();iter != close_neighbors_halfR.end();iter++) {
            t += iter->second.size();
        }
        return t;
    }

    int get_total_R_points() {
        int t = 0;
        for(auto iter = close_neighbors_R.begin();iter != close_neighbors_R.end();iter++) {
            t += iter->second.size();
        }
        return t;
    }

    int get_total_3halfR_points() {
        int t = 0;
        for(auto iter = close_neighbors_3halfR.begin();iter != close_neighbors_3halfR.end();iter++) {
            t += iter->second.size();
        }
        return t;
    }

    int get_total_2R_points() {
        int t = 0;
        for(auto iter = close_neighbors_2R.begin();iter != close_neighbors_2R.end();iter++) {
            t += iter->second.size();
        }
        return t;
    }

    bool is_covered_all_slides() {

    }

    CorePoint(C_Data d) {
        values = d.values;
        arrival_time = d.arrival_time;
    }

    CorePoint* copy() {
        auto* fuck = new CorePoint();
        fuck->values = values;
        fuck->arrival_time = arrival_time;
        fuck->num_succeeding_neighbor = num_succeeding_neighbor;
        fuck->last_prob_left = last_prob_left;
        fuck->last_prob_right = last_prob_right;
        fuck->pred_neighbor_count = pred_neighbor_count;
        fuck->neighbor_count = neighbor_count;
        fuck->close_core_halfR = close_core_halfR;
        fuck->close_core_maps_R = close_core_maps_R;
        fuck->s_index = s_index;
        fuck->close_neighbors_halfR = close_neighbors_halfR;
        fuck->close_neighbors_R = close_neighbors_R;
        fuck->close_neighbors_3halfR = close_neighbors_3halfR;
        fuck->close_neighbors_2R = close_neighbors_2R;
        return fuck;
    }

    CorePoint() {}

    vector<double>::const_iterator begin() const {return values.begin();}

    vector<double>::const_iterator end() const {return values.end();}
};

inline bool operator<(const CorePoint& a, const CorePoint& b) {
    return std::lexicographical_compare(a.begin(), a.end(),
                                        b.begin(), b.end());
}

class ResultFindCore {
private:
    double distance; // biggest
    vector<CorePoint*> cores;

public:
    vector<double> distance_to_cores;

    ResultFindCore(double distance, vector<CorePoint*> cores) {
        this->distance = distance;
        this->cores = std::move(cores);
    }

    ResultFindCore(double distance, vector<CorePoint*> cores, vector<double> all_distance) {
        this->distance = distance;
        this->cores = std::move(cores);
        this->distance_to_cores = std::move(all_distance);
    }

    double get_distance(){
        return this->distance;
    }

    vector<CorePoint*> get_core() {
        return this->cores;
    }
};

typedef CorePoint Data;
typedef set<Data*> DataSet;
typedef mt::functions::cached_distance_function<Data, mt::functions::euclidean_distance> CachedDistanceFunction;
typedef pair<Data*, Data*>(*PromotionFunction)(const DataSet&, CachedDistanceFunction&);

static PromotionFunction nonRandomPromotion =
        [](const DataSet& dataSet, CachedDistanceFunction&) -> pair<Data*, Data*> {
            vector<Data*> dataObjects(dataSet.begin(), dataSet.end());
            sort(dataObjects.begin(), dataObjects.end());
            return {dataObjects.front(), dataObjects.back()};
        };


typedef mt::mtree<
        Data,
        mt::functions::euclidean_distance,
        mt::functions::split_function<
                PromotionFunction,
                mt::functions::balanced_partition
        >
>
        MTree;

class MTreeCorePoint : public MTree {
private:
    struct OnExit {
        MTreeCorePoint* mt;
        OnExit(MTreeCorePoint* mt) : mt(mt) {}
        ~OnExit() { mt->_check(); }
    };

public:
    // Turning the member public
    using MTree::distance_function;

    MTreeCorePoint()
            : MTree(100, 199,
                    distance_function_type(),
                    split_function_type(nonRandomPromotion)
    )
    {}

    void add(Data* data) {
        OnExit onExit(this);
        return MTree::add(data);
    }

    bool remove(Data* data) {
        OnExit onExit(this);
        return MTree::remove(data);
    }
};


#endif //CPOD_CPOD_H

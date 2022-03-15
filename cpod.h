//
// Created by wmcool on 2022/3/11.
//

#include <vector>
#include <set>
#include <cstdarg>
#include "include/functions.h"
#include "include/mtree.h"
#include "unordered_map"

#ifndef CPOD_CPOD_H
#define CPOD_CPOD_H


#define assertEqual(A, B)             assert(A == B);
#define assertLessEqual(A, B)         assert(A <= B);
#define assertIn(ELEM, CONTAINER)     assert(CONTAINER.find(ELEM) != CONTAINER.end());
#define assertNotIn(ELEM, CONTAINER)  assert(CONTAINER.find(ELEM) == CONTAINER.end());

const double R = 1;
const int K = 50;
const int WINDOW = 10000;
const int SLIDE = 500;

using namespace std;


class Cpod;
class Data;
class C_Data;
class CorePoint;
class MTreeCorePoint;

vector<Data> detect_outlier(vector<Data> data, int n_current_time, int W, int slide);
void process_expired_data(int expired_slide_index_cur);
vector<CorePoint> select_core(int s_idx);
void probe(C_Data* d, int newest_slide);

static int current_time;
static int expired_slide_index = -1; // remember init
static unordered_map<int, vector<C_Data>> all_slides;
static unordered_map<int, vector<CorePoint>> all_core_points;
static MTreeCorePoint* mtree;
static vector<CorePoint> all_distince_cores;
static unordered_map<int, set<C_Data>> outlier_list;
static unordered_map<int, set<C_Data>> neighbor_count_trigger;

class Data {
public:
    vector<double> values;
    int arrival_time;

    Data(){}

    explicit Data(int dimensions, ...) {
        va_list args;
        va_start(args, dimensions);
        for(int i=0;i<dimensions;i++) {
            values.push_back(va_arg(args, double));
        }
        va_end(args);
    }

    explicit Data(vector<double> d_values) : values(d_values){}

    vector<double>::iterator begin() {return values.begin();}

    vector<double>::iterator end() {return values.end();}
};



class C_Data : public Data {
public:
    int num_succeeding_neighbor = 0;
    int last_prob_right = -1;
    int last_prob_left = -1;

    unordered_map<int, int> pred_neighbor_count;
    int neighbor_count = 0;
    CorePoint* close_core_halfR;
    vector<CorePoint> close_core_maps_R;
    int s_index = -1;

    C_Data(Data* d) : Data(d->values) {
        this->arrival_time = d->arrival_time;
        this->values = d->values;
        this->s_index = (arrival_time - 1) / SLIDE;
    }

    C_Data() {}

    int count_neighbor() {return neighbor_count;}
};

class CorePoint : public C_Data {
public:
    unordered_map<int, vector<C_Data>> close_neighbors_halfR;
    unordered_map<int, vector<C_Data>> close_neighbors_R;
    unordered_map<int, vector<C_Data>> close_neighbors_3halfR;
    unordered_map<int, vector<C_Data>> close_neighbors_2R;

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
};

typedef set<Data> DataSet;
typedef mt::functions::cached_distance_function<Data, mt::functions::euclidean_distance> CachedDistanceFunction;
typedef pair<Data, Data>(*PromotionFunction)(const DataSet&, CachedDistanceFunction&);

PromotionFunction nonRandomPromotion =
        [](const DataSet& dataSet, CachedDistanceFunction&) -> pair<Data, Data> {
            vector<Data> dataObjects(dataSet.begin(), dataSet.end());
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

    void add(const Data& data) {
        OnExit onExit(this);
        return MTree::add(data);
    }

    bool remove(const Data& data) {
        OnExit onExit(this);
        return MTree::remove(data);
    }
};


#endif //CPOD_CPOD_H

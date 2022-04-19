//
// Created by wmcool on 2022/3/11.
//

#include <iostream>
#include "cpod.h"

int current_time;
int expired_slide_index = -1; // remember init
unordered_map<int, vector<C_Data*>> all_slides;
unordered_map<int, vector<CorePoint*>> all_core_points;
MTreeCorePoint *mtree;
vector<CorePoint*> all_distinct_cores;
unordered_map<int, set<C_Data>> outlier_list;
unordered_map<int, set<C_Data>> neighbor_count_trigger; // when slide expired, trigger points in set

double R = 1;
int K = 50;
int WINDOW = 10000;
int SLIDE = 500;

static double euclidean_distance(Point* d1, Point* d2) {
    double sum_square = 0;
    for (int i = 0; i < d1->values.size(); i++) {
        sum_square += pow(d1->values[i] - d2->values[i], 2);
    }
    return sqrt(sum_square);
}

vector<Point> detect_outlier(vector<Point> data, int n_current_time, int W, int slide) {
    current_time = n_current_time;
    vector<C_Data*> d_to_process;
    vector<int> slide_to_process;
    if (current_time == W) { // when start, process all slide in window
        for (int i = 0; i < W / slide; i++) slide_to_process.push_back(i);
    } else { // only process the coming slide
        slide_to_process.push_back((current_time - 1) / slide);
    }
    for (int i = 0; i < data.size(); i++) {
        Point *o = &data[i];
        auto *d = new C_Data(o);
        d_to_process.push_back(d);
        if (all_slides.find(d->s_index) != all_slides.end()) {
            all_slides[d->s_index].push_back(d);
        } else {
            vector<C_Data*> data_in_slide;
            data_in_slide.push_back(d);
            all_slides[d->s_index] = data_in_slide;
        }
    }

    vector<Point> result;
    expired_slide_index = (current_time - 1) / SLIDE - WINDOW / SLIDE;
    process_expired_data(expired_slide_index);
    for (int i = 0; i < slide_to_process.size(); i++) {
        vector<CorePoint*> core_points = select_core(slide_to_process[i]);
        all_core_points[slide_to_process[i]] = core_points;
    }
    int newest_slide = (current_time - 1) / SLIDE;

    if (current_time == WINDOW) {
        for (auto iter = all_distinct_cores.begin(); iter != all_distinct_cores.end(); iter++) {
            (*iter)->total_halfR_points = (*iter)->get_total_halfR_points();
        }
    } else if (data.size() == SLIDE) {
        for (auto iter = all_distinct_cores.begin(); iter != all_distinct_cores.end(); iter++) {
            if ((*iter)->close_neighbors_halfR.find(newest_slide) != (*iter)->close_neighbors_halfR.end()) {
                (*iter)->total_halfR_points += (*iter)->close_neighbors_halfR[newest_slide].size();
            }
        }
    }

    for (int i = 0; i < d_to_process.size(); i++) {
        C_Data* d = d_to_process[i];
        if (d->close_core_halfR != nullptr && d->close_core_halfR->total_halfR_points >= K + 1) {
            continue;
        }
        if (d->neighbor_count < K) probe(d, newest_slide);
    }

    for (auto iter = outlier_list.begin(); iter != outlier_list.end(); iter++) {
        for (auto outlier = (*iter).second.begin(); outlier != (*iter).second.end(); outlier++) {
            if ((*outlier).close_core_halfR != nullptr &&
                (*outlier).close_core_halfR->total_halfR_points >= K + 1)
                continue;
            if ((*outlier).neighbor_count < K && (*outlier).s_index < newest_slide) {
                C_Data d = (*outlier);
                if ((*outlier).last_prob_right < newest_slide) probe(&d, newest_slide);
            }
            if ((*outlier).neighbor_count < K && outlier->arrival_time >= data[0].arrival_time) {
                C_Data d = (*outlier);
                result.push_back(d);
            }
        }
    }
    return result;
}

void process_expired_data(int expired_slide_index_cur) {
    for(auto & i : all_slides[expired_slide_index_cur]) {
        delete i;
    }
    all_slides.erase(expired_slide_index_cur);
    outlier_list.erase(expired_slide_index_cur);
    if (neighbor_count_trigger.find(expired_slide_index_cur) != neighbor_count_trigger.end()) {
        for (auto iter = neighbor_count_trigger[expired_slide_index_cur].begin();
             iter != neighbor_count_trigger[expired_slide_index_cur].end(); iter++) {
            if ((*iter).pred_neighbor_count.find(expired_slide_index_cur) != (*iter).pred_neighbor_count.end()) {
                C_Data d = (*iter);
                d.neighbor_count -= d.pred_neighbor_count[expired_slide_index_cur];
                d.pred_neighbor_count.erase(expired_slide_index_cur);
            }
        }
    }
    if (neighbor_count_trigger.find(expired_slide_index_cur - 1) != neighbor_count_trigger.end()) {
        neighbor_count_trigger.erase(expired_slide_index_cur - 1);
    }
    all_core_points.erase(expired_slide_index_cur);
    for (auto iter = all_distinct_cores.begin(); iter != all_distinct_cores.end(); iter++) {
        CorePoint* c = (*iter);
        if ((*c).close_neighbors_halfR.find(expired_slide_index_cur) != (*c).close_neighbors_halfR.end()) {
            (*c).total_halfR_points -= (*c).close_neighbors_halfR[expired_slide_index_cur].size();
            (*c).close_neighbors_halfR.erase(expired_slide_index_cur);
        }
        (*c).close_neighbors_R.erase(expired_slide_index_cur);
        (*c).close_neighbors_3halfR.erase(expired_slide_index_cur);
        (*c).close_neighbors_2R.erase(expired_slide_index_cur);
    }
}

vector<CorePoint*> select_core(int s_idx) {
    vector<CorePoint*> core_points;
    vector<CorePoint*> new_cores;
    for (int i = 0; i < SLIDE; i++) {
        C_Data* d = all_slides[s_idx][i];

        // scan with current cores first
        for (int j = core_points.size() - 1; j >= 0; j--) {
            CorePoint* c = core_points[j];
            double distance = euclidean_distance(d, c);
            if (distance <= R / 2) {
                if ((*c).close_neighbors_halfR.find(s_idx) != (*c).close_neighbors_halfR.end()) {
                    (*c).close_neighbors_halfR[s_idx].push_back(d);
                } else {
                    vector<C_Data*> close_neighbors;
                    close_neighbors.push_back(d);
                    (*c).close_neighbors_halfR[s_idx] = close_neighbors;
                }
                d->close_core_halfR = c;
                break;
            } else if (distance <= R) {
                if ((*c).close_neighbors_R.find(s_idx) != (*c).close_neighbors_R.end()) {
                    (*c).close_neighbors_R[s_idx].push_back(d);
                } else {
                    vector<C_Data*> close_neighbors;
                    close_neighbors.push_back(d);
                    (*c).close_neighbors_R[s_idx] = close_neighbors;
                }
                d->close_core_maps_R.push_back(c);
                break;
            }
        }

        // scan in all core points (except in current slide)
        if (d->close_core_maps_R.empty() && d->close_core_halfR == nullptr) {
            MTreeCorePoint::query query = mtree->get_nearest(*d, R, 1);
            CorePoint *c;
            double distance = numeric_limits<double>::max();
            for (MTreeCorePoint::query::iterator i = query.begin(); i != query.end(); ++i) {
                MTreeCorePoint::query::value_type r = *i;
                c = r.data;
                distance = r.distance;
            }
            if (distance <= R) {
                core_points.push_back(c);
                if (distance <= R / 2) {
                    if ((*c).close_neighbors_halfR.find(s_idx) != (*c).close_neighbors_halfR.end()) {
                        (*c).close_neighbors_halfR[s_idx].push_back(d);
                    } else {
                        vector<C_Data*> close_neighbors;
                        close_neighbors.push_back(d);
                        (*c).close_neighbors_halfR[s_idx] = close_neighbors;
                    }
                    d->close_core_halfR = c;
                } else {
                    if ((*c).close_neighbors_R.find(s_idx) != (*c).close_neighbors_R.end()) {
                        (*c).close_neighbors_R[s_idx].push_back(d);
                    } else {
                        vector<C_Data*> close_neighbors;
                        close_neighbors.push_back(d);
                        (*c).close_neighbors_R[s_idx] = close_neighbors;
                    }
                    d->close_core_maps_R.push_back(c);
                }
            } else { // create new core point
                auto* tmp = new CorePoint(d);
                c = tmp;
                all_distinct_cores.push_back(c);
                new_cores.push_back(c);
                core_points.push_back(c);
                if ((*c).close_neighbors_halfR.find(s_idx) != (*c).close_neighbors_halfR.end()) {
                    (*c).close_neighbors_halfR[s_idx].push_back(d);
                } else {
                    vector<C_Data*> close_neighbors;
                    close_neighbors.push_back(d);
                    (*c).close_neighbors_halfR[s_idx] = close_neighbors;
                }
                d->close_core_halfR = c;
            }
        }
    }
    bool *checked = new bool[SLIDE]{false};
    for (auto iter = core_points.begin(); iter != core_points.end(); iter++) {
        CorePoint* c = *iter;
        if ((*c).close_neighbors_halfR.find(s_idx) == (*c).close_neighbors_halfR.end()) {
            (*c).close_neighbors_halfR[s_idx] = vector<C_Data*>();
        }
        if ((*c).close_neighbors_R.find(s_idx) == (*c).close_neighbors_R.end()) {
            (*c).close_neighbors_R[s_idx] = vector<C_Data*>();
        }
        if ((*c).close_neighbors_3halfR.find(s_idx) == (*c).close_neighbors_3halfR.end()) {
            (*c).close_neighbors_3halfR[s_idx] = vector<C_Data*>();
        }
        if ((*c).close_neighbors_2R.find(s_idx) == (*c).close_neighbors_2R.end()) {
            (*c).close_neighbors_2R[s_idx] = vector<C_Data*>();
        }
        for (int i = 0; i < SLIDE; i++) {
            checked[i] = false;
        }
        for (auto iter2 = core_points.begin(); iter2 != core_points.end(); iter2++) {
            CorePoint* c2 = *iter2;
            if (c != c2) {
                double distance = euclidean_distance(c, c2);
                if (distance <= R * 3) { // pre core points only be inited their R-associated, so we need 3R-associated core points to init their 2R-associated
                    checked = prob_core_with_list(c, (*c2).close_neighbors_halfR[s_idx], s_idx, checked,
                                                  all_slides[s_idx][0]->arrival_time);
                    checked = prob_core_with_list(c, (*c2).close_neighbors_R[s_idx], s_idx, checked,
                                                  all_slides[s_idx][0]->arrival_time);
                }
            }
        }
    }
    for (auto iter = new_cores.begin(); iter != new_cores.end(); iter++) {
        mtree->add(*iter);
    }
    return core_points;
}

void probe(C_Data *d, int newest_slide) {
    bool counted = false;
    if (d->last_prob_right < newest_slide) { // prob right
        int slide_index = d->last_prob_right + 1;
        if (d->last_prob_right == -1) {
            slide_index = d->s_index; // 从当前slide往右探索
        }
        while (slide_index <= newest_slide && d->neighbor_count < K) {
            if (!counted) counted = true;
            probe_slide_right(d, slide_index);
            d->last_prob_right = slide_index;
            slide_index++;
        }
    }
    if (d->neighbor_count < K) {
        int slide_index = d->last_prob_left - 1;
        if (d->last_prob_left == -1) slide_index = d->s_index - 1;
        while (slide_index > expired_slide_index && slide_index >= 0 && d->neighbor_count < K) {
            if (!counted) counted = true;
            probe_slide_left(d, slide_index);
            d->last_prob_left = slide_index;
            slide_index--;
        }
    }
    if (d->neighbor_count < K) {
        if (outlier_list.find(d->s_index) != outlier_list.end()) {
            outlier_list[d->s_index].insert(d);
        } else {
            set<C_Data> s;
            s.insert(d);
            outlier_list[d->s_index] = s;
        }
    }
}

void probe_slide_right(C_Data* d, int slide_index) {
    vector<C_Data*> possible_candidates;
    ResultFindCore *rf = find_close_core(d, slide_index);
    if (rf != nullptr) {
        double distance = rf->get_distance();
        vector<CorePoint*> cores = rf->get_core();
        int case_ = 0;
        if (distance <= R / 2) {
            CorePoint* c = cores[0];
            (*d).neighbor_count += (*c).close_neighbors_halfR[slide_index].size();
            (*d).num_succeeding_neighbor += (*c).close_neighbors_halfR[slide_index].size();
            if ((*d).num_succeeding_neighbor >= K) return;
            possible_candidates.insert(possible_candidates.end(), (*c).close_neighbors_R[slide_index].begin(),
                                       (*c).close_neighbors_R[slide_index].end());
            possible_candidates.insert(possible_candidates.end(), (*c).close_neighbors_3halfR[slide_index].begin(),
                                       (*c).close_neighbors_3halfR[slide_index].end());
        } else if (distance <= R) {
            CorePoint* c = cores[0];
            possible_candidates.insert(possible_candidates.end(), (*c).close_neighbors_halfR[slide_index].begin(),
                                       (*c).close_neighbors_halfR[slide_index].end());
            possible_candidates.insert(possible_candidates.end(), (*c).close_neighbors_R[slide_index].begin(),
                                       (*c).close_neighbors_R[slide_index].end());
            possible_candidates.insert(possible_candidates.end(), (*c).close_neighbors_3halfR[slide_index].begin(),
                                       (*c).close_neighbors_3halfR[slide_index].end());
            possible_candidates.insert(possible_candidates.end(), (*c).close_neighbors_2R[slide_index].begin(),
                                       (*c).close_neighbors_2R[slide_index].end());
        } else if (distance <= R * 2) {
            case_ = 1;
            for (int i = 0; i < cores.size(); i++) {
                CorePoint* c = cores[i];
                if (rf->distance_to_cores[i] <= R * 3 / 2) {
                    possible_candidates.insert(possible_candidates.end(), (*c).close_neighbors_halfR[slide_index].begin(),
                                               (*c).close_neighbors_halfR[slide_index].end());
                }
            }
            for (auto iter = cores.begin(); iter != cores.end(); iter++) {
                CorePoint* c = (*iter);
                possible_candidates.insert(possible_candidates.end(), (*c).close_neighbors_R[slide_index].begin(),
                                           (*c).close_neighbors_R[slide_index].end());
            }
        }

        int min_arrival_time = all_slides[slide_index][0]->arrival_time;
        bool *checked;
        if (case_ == 1) checked = new bool[SLIDE]; // distance <= R * 2时，candidates可能有重复，用checked去重
        int old_num_suc_neighbor = d->num_succeeding_neighbor;
        for (auto iter = possible_candidates.begin(); iter != possible_candidates.end(); iter++) {
            C_Data* d2 = (*iter);
            if (case_ == 0 || (case_ == 1) && !checked[d2->arrival_time - min_arrival_time]) {
                if (euclidean_distance(d, d2) <= R) {
                    d->num_succeeding_neighbor += 1;
                    if (d->num_succeeding_neighbor >= K) {
                        d->pred_neighbor_count.clear();
                        d->neighbor_count += d->num_succeeding_neighbor - old_num_suc_neighbor;
                        return;
                    }
                }
                if (case_ == 1) checked[d2->arrival_time - min_arrival_time] = true;
            }
        }
        if(case_ == 1) delete checked;
        d->neighbor_count += d->num_succeeding_neighbor - old_num_suc_neighbor;
        delete rf;
    } else {
//        std::cout << "no core found" << std::endl;
    }
}

void probe_slide_left(C_Data* d, int slide_index) {
    int old_num_neighbor = d->neighbor_count;
    vector<C_Data*> possible_candidates;
    ResultFindCore *rf = find_close_core(d, slide_index);
    if (rf != nullptr) {
        double distance = rf->get_distance();
        vector<CorePoint*> cores = rf->get_core();
        int case_ = 0;
        if (distance <= R / 2) {
            CorePoint* c = cores[0];
            d->neighbor_count += c->close_neighbors_halfR[slide_index].size();
            d->num_succeeding_neighbor += c->close_neighbors_halfR[slide_index].size();
            if (d->num_succeeding_neighbor >= K) {
                d->pred_neighbor_count[slide_index] = c->close_neighbors_halfR[slide_index].size();
                if (neighbor_count_trigger.find(slide_index) != neighbor_count_trigger.end()) {
                    neighbor_count_trigger[slide_index].insert(d);
                } else {
                    set<C_Data> hs;
                    hs.insert(d);
                    neighbor_count_trigger[slide_index] = hs;
                }
                return;
            }
            possible_candidates.insert(possible_candidates.end(), c->close_neighbors_R[slide_index].begin(),
                                       c->close_neighbors_R[slide_index].end());
            possible_candidates.insert(possible_candidates.end(), c->close_neighbors_3halfR[slide_index].begin(),
                                       c->close_neighbors_3halfR[slide_index].end());
        } else if (distance <= R) {
            CorePoint* c = cores[0];
            possible_candidates.insert(possible_candidates.end(), c->close_neighbors_halfR[slide_index].begin(),
                                       c->close_neighbors_halfR[slide_index].end());
            possible_candidates.insert(possible_candidates.end(), c->close_neighbors_R[slide_index].begin(),
                                       c->close_neighbors_R[slide_index].end());
            possible_candidates.insert(possible_candidates.end(), c->close_neighbors_3halfR[slide_index].begin(),
                                       c->close_neighbors_3halfR[slide_index].end());
            possible_candidates.insert(possible_candidates.end(), c->close_neighbors_2R[slide_index].begin(),
                                       c->close_neighbors_2R[slide_index].end());
        } else if (distance <= R * 2) {
            case_ = 1;
            for (int i = 0; i < cores.size(); i++) {
                CorePoint* c = cores[i];
                if (rf->distance_to_cores[i] <= R * 3 / 2) {
                    possible_candidates.insert(possible_candidates.end(), c->close_neighbors_halfR[slide_index].begin(),
                                               c->close_neighbors_halfR[slide_index].end());
                }
            }
            for (auto iter = cores.begin(); iter != cores.end(); iter++) {
                CorePoint* c = (*iter);
                possible_candidates.insert(possible_candidates.end(), c->close_neighbors_R[slide_index].begin(),
                                           c->close_neighbors_R[slide_index].end());
            }
        }

        int min_arrival_time = all_slides[slide_index][0]->arrival_time;
        bool *checked;
        if (case_ == 1) checked = new bool[SLIDE];
        int old_num_suc_neighbor = d->num_succeeding_neighbor;

        for (auto iter = possible_candidates.begin(); iter != possible_candidates.end(); iter++) {
            C_Data* d2 = (*iter);
            if (case_ == 0 || (case_ == 1) && !checked[d2->arrival_time - min_arrival_time]) {
                if (euclidean_distance(d, d2) <= R) {
                    d->num_succeeding_neighbor += 1;
                    if (d->num_succeeding_neighbor >= K) {
                        break;
                    }
                }
                if (case_ == 1) checked[d2->arrival_time - min_arrival_time] = true;
            }
//        d.neighbor_count += d.num_succeeding_neighbor - old_num_suc_neighbor;
        }
        if(case_ == 1) delete checked;
        d->pred_neighbor_count[slide_index] = d->neighbor_count - old_num_neighbor;
        if (neighbor_count_trigger.find(slide_index) != neighbor_count_trigger.end()) {
            neighbor_count_trigger[slide_index].insert(d);
        } else {
            set<C_Data> hs;
            hs.insert(d);
            neighbor_count_trigger[slide_index] = hs;
        }
        delete rf;
    }
}

ResultFindCore *find_close_core(C_Data* d, int slide_index) {
    vector<CorePoint*> result_core;
    if (d->close_core_halfR != nullptr && d->close_core_halfR->close_neighbors_halfR.find(slide_index) != d->close_core_halfR->close_neighbors_halfR.end()) {
        result_core.push_back(d->close_core_halfR);
        auto *res = new ResultFindCore(R / 2, result_core);
        return res;
    } else if (!(*d).close_core_maps_R.empty()) {
        for (auto iter = (*d).close_core_maps_R.begin(); iter != (*d).close_core_maps_R.end(); iter++) {
            CorePoint* c = (*iter);
            if ((*c).close_neighbors_2R.find(slide_index) != c->close_neighbors_2R.end()) {
                result_core.push_back(c);
                auto *res = new ResultFindCore(R, result_core);
                return res;
            }
        }
    }

    vector<CorePoint*> in_range_R_cores;
    vector<CorePoint*> in_range_2R_cores;
    vector<double> distance_to_cores;
    if (all_core_points.find(slide_index) != all_core_points.end()) {
        vector<CorePoint*> core_points = all_core_points[slide_index];
        for (auto iter = core_points.begin(); iter != core_points.end(); iter++) {
            CorePoint* c = (*iter);
            Point* d1 = c;
            double distance = euclidean_distance(d, d1);
            if (distance <= R) {
                in_range_R_cores.push_back(c);
                break;
            } else if (distance <= R * 2) {
                in_range_2R_cores.push_back(c);
                distance_to_cores.push_back(distance);
            }
        }
    }
    if (!in_range_R_cores.empty()) {
        auto *res = new ResultFindCore(R, in_range_R_cores);
        return res;
    } else if (!in_range_2R_cores.empty()) {
        auto *res = new ResultFindCore(R * 2, in_range_2R_cores, distance_to_cores);
        return res;
    } else return nullptr;
}

bool *prob_core_with_list(CorePoint* c, vector<C_Data*> candidates, int s_idx, bool *checked, int start_time) {
    Point* d1 = c;
    if (!candidates.empty()) {
        for (auto iter = candidates.begin(); iter != candidates.end(); iter++) {
            C_Data* d2 = (*iter);
            if (!checked[(*d2).arrival_time - start_time]) {
                double distance = euclidean_distance(d1, d2);
                if (distance <= R / 2) {
                    (*c).close_neighbors_halfR[s_idx].push_back(d2);
                    (*d2).close_core_halfR = c;
                } else if (distance <= R) {
                    (*c).close_neighbors_R[s_idx].push_back(d2);
                    (*d2).close_core_maps_R.push_back(c);
                } else if (distance <= R * 1.5) {
                    (*c).close_neighbors_3halfR[s_idx].push_back(d2);
                } else if (distance <= R * 2) {
                    (*c).close_neighbors_2R[s_idx].push_back(d2);
                }
                checked[(*d2).arrival_time - start_time] = true;
            }
        }
    }
    return checked;
}
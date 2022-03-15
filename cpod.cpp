//
// Created by wmcool on 2022/3/11.
//

#include "cpod.h"
#include "distance.cpp"

vector<Data> detect_outlier(vector<Data> data, int n_current_time, int W, int slide) {
    current_time = n_current_time;
    vector<C_Data> d_to_process(data.size());
    int* slide_to_process;
    if(current_time == W) { // when start, process all slide in window
        slide_to_process = new int[W / slide];
        for(int i=0;i<W/slide;i++) slide_to_process[i] = i;
    } else { // only process the coming slide
        slide_to_process = new int[]{(current_time - 1) / slide};
    }
    for(int i=0;i<data.size();i++) {
        Data* o = &data[i];
        C_Data* d = new C_Data(o);
        d_to_process.push_back(*d);
        if(all_slides.find(d->s_index) != all_slides.end()) {
            all_slides[d->s_index].push_back(*d);
        } else {
            vector<C_Data> data_in_slide;
            data_in_slide.push_back(*d);
            all_slides[d->s_index] = data_in_slide;
        }

        vector<Data> result(data.size() / 100);
        expired_slide_index = (current_time - 1) / SLIDE - WINDOW / SLIDE;
        process_expired_data(expired_slide_index);
        for(int i=0;i<sizeof(slide_to_process) / sizeof(int);i++) {
            vector<CorePoint> core_points = select_core(slide_to_process[i]);
            all_core_points[slide_to_process[i]] = core_points;
        }
        int newest_slide = (current_time - 1) / SLIDE;

        if(current_time == WINDOW) {
            for(auto iter = all_distince_cores.begin();iter != all_distince_cores.end();iter++) {
                (*iter).total_halfR_points = (*iter).get_total_halfR_points();
            }
        } else if(data.size() == SLIDE) {
            for(auto iter = all_distince_cores.begin();iter != all_distince_cores.end();iter++) {
                if((*iter).close_neighbors_halfR.find(newest_slide) != (*iter).close_neighbors_halfR.end()) {
                    (*iter).total_halfR_points += (*iter).close_neighbors_halfR[newest_slide].size();
                }
            }
        }

        for(int i=0;i<sizeof(d_to_process) / sizeof(int);i++) {
            C_Data d = d_to_process[i];
            if(d.close_core_halfR != nullptr && d.close_core_halfR->total_halfR_points >= K + 1) {
                continue;
            }
            if(d.neighbor_count < K) probe(&d, newest_slide);
        }

        for(auto iter=outlier_list.begin();iter!=outlier_list.end();iter++) {
            for(auto outlier=(*iter).second.begin();outlier!=(*iter).second.end();outlier++) {
                if((*outlier).close_core_halfR != nullptr && (*outlier).close_core_halfR->total_halfR_points >= K + 1) continue;
                if((*outlier).neighbor_count < K && (*outlier).s_index < newest_slide) {
                    C_Data d = (*outlier);
                    if((*outlier).last_prob_right < newest_slide) probe(&d, newest_slide);
                }
                if((*outlier).neighbor_count < K) {
                    C_Data d = (*outlier);
                    result.push_back(d);
                }
            }
        }
        return result;
    }
}

void process_expired_data(int expired_slide_index_cur) {
    all_slides.erase(expired_slide_index_cur);
    outlier_list.erase(expired_slide_index_cur);
    if(neighbor_count_trigger.find(expired_slide_index_cur) != neighbor_count_trigger.end()) {
        for(auto iter=neighbor_count_trigger[expired_slide_index_cur].begin(); iter != neighbor_count_trigger[expired_slide_index_cur].end(); iter++) {
            if((*iter).pred_neighbor_count.find(expired_slide_index_cur) != (*iter).pred_neighbor_count.end()) {
                C_Data d = (*iter);
                d.neighbor_count -= d.pred_neighbor_count[expired_slide_index_cur];
                d.pred_neighbor_count.erase(expired_slide_index_cur);
            }
        }
    }
    if(neighbor_count_trigger.find(expired_slide_index_cur - 1) != neighbor_count_trigger.end()) {
        neighbor_count_trigger.erase(expired_slide_index_cur - 1);
    }
    all_core_points.erase(expired_slide_index_cur);
    for(auto iter=all_distince_cores.begin();iter!=all_distince_cores.end();iter++) {
        CorePoint c = (*iter);
        if(c.close_neighbors_halfR.find(expired_slide_index_cur) != c.close_neighbors_halfR.end()) {
            c.total_halfR_points -= c.close_neighbors_halfR[expired_slide_index_cur].size();
            c.close_neighbors_halfR.erase(expired_slide_index_cur);
        }
        c.close_neighbors_R.erase(expired_slide_index_cur);
        c.close_neighbors_3halfR.erase(expired_slide_index_cur);
        c.close_neighbors_2R.erase(expired_slide_index_cur);
    }
}

vector<CorePoint> select_core(int s_idx) {
    vector<CorePoint> core_points;
    vector<CorePoint> new_cores;
    for(int i=0;i<SLIDE;i++) {
        C_Data d = all_slides[s_idx][i];

        // scan with current cores first
        for(int j=core_points.size()-1;j>=0;j--) {
            CorePoint c = core_points[j];
            double distance = euclidean_distance(d, c);
            if(distance <= R / 2) {
                if(c.close_neighbors_halfR.find(s_idx) != c.close_neighbors_halfR.end()) {
                    c.close_neighbors_halfR[s_idx].push_back(d);
                } else {
                    vector<C_Data> close_neighbors;
                    close_neighbors.push_back(d);
                    c.close_neighbors_halfR[s_idx] = close_neighbors;
                }
                d.close_core_halfR = &c;
                break;
            } else if(distance <= R) {
                if(c.close_neighbors_R.find(s_idx) != c.close_neighbors_R.end()) {
                    c.close_neighbors_R[s_idx].push_back(d);
                } else {
                    vector<C_Data> close_neighbors;
                    close_neighbors.push_back(d);
                    c.close_neighbors_R[s_idx] = close_neighbors;
                }
                d.close_core_maps_R.push_back(c);
                break;
            }
        }

        // scan in all core points (except in current slide)
        if(d.close_core_maps_R.empty() && d.close_core_halfR == nullptr) {
            MTreeCorePoint::query query = mtree->get_nearest(d, R, 1);
            CorePoint c;
            double distance;
            for(MTreeCorePoint::query::iterator i = query.begin(); i != query.end(); ++i) {
                MTreeCorePoint::query::value_type r = *i;
            }
        }
    }
}

void probe(C_Data* d, int newest_slide) {

}
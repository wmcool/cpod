//
// Created by wmcool on 2022/3/15.
//

static double euclidean_distance(Data d1, Data d2) {
    double sum_square = 0;
    for(int i=0;i<d1.values.size();i++) {
        sum_square += pow(d1.values[i] - d2.values[i], 2);
    }
    return sqrt(sum_square);
}


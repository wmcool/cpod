//
// Created by dell on 2022/3/20.
//

#include "include/mtree.h"
#include "cpod.h"
#include "vector"
#include "iostream"

using namespace std;

int main() {
    vector<int> a;
    a.push_back(1);
    vector<int> b = a;
    b.push_back(2);
    for(int i=0;i<b.size();i++) cout << b[i] << endl;
    return 0;
}


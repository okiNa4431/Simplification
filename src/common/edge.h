#pragma once

#include "point.h"

struct Edge {
    Edge() {}
    Edge(Point s, Point t) {
        if (s.index < t.index) {
            vertex1 = s; vertex2 = t;
        }
        else {
            vertex1 = t; vertex2 = s;
        }
    }
    //vertex1の方がインデックスが小さくなるように設定する
    Point vertex1;
    Point vertex2;

    //比較演算子をオーバーロード
    bool operator==(const Edge rhs) {
        return vertex1.index == rhs.vertex1.index && vertex2.index == rhs.vertex2.index;
    }
    const bool operator==(const Edge rhs) const {
        return vertex1.index == rhs.vertex1.index && vertex2.index == rhs.vertex2.index;
    }

    bool operator!=(const Edge rhs) {
        return vertex1.index != rhs.vertex1.index || vertex2.index != rhs.vertex2.index;
    }
    const bool operator!=(const Edge rhs) const {
        return vertex1.index != rhs.vertex1.index || vertex2.index != rhs.vertex2.index;
    }

    bool operator<(const Edge rhs) {
        return vertex1.index < rhs.vertex1.index || (vertex1.index == rhs.vertex1.index && vertex2.index < rhs.vertex2.index);
    }
    const bool operator<(const Edge rhs) const {
        return vertex1.index < rhs.vertex1.index || (vertex1.index == rhs.vertex1.index && vertex2.index < rhs.vertex2.index);
    }

    bool operator<=(const Edge rhs) {
        return vertex1.index <= rhs.vertex1.index || (vertex1.index == rhs.vertex1.index && vertex2.index <= rhs.vertex2.index);
    }
    const bool operator<=(const Edge rhs) const {
        return vertex1.index <= rhs.vertex1.index || (vertex1.index == rhs.vertex1.index && vertex2.index <= rhs.vertex2.index);
    }

    bool operator>(const Edge rhs) {
        return vertex1.index > rhs.vertex1.index || (vertex1.index == rhs.vertex1.index && vertex2.index > rhs.vertex2.index);
    }
    const bool operator>(const Edge rhs) const {
        return vertex1.index > rhs.vertex1.index || (vertex1.index == rhs.vertex1.index && vertex2.index > rhs.vertex2.index);
    }

    bool operator>=(const Edge rhs) {
        return vertex1.index >= rhs.vertex1.index || (vertex1.index == rhs.vertex1.index && vertex2.index >= rhs.vertex2.index);
    }
    const bool operator>=(const Edge rhs) const {
        return vertex1.index >= rhs.vertex1.index || (vertex1.index == rhs.vertex1.index && vertex2.index >= rhs.vertex2.index);
    }

    //片方の点を指定した点に変更する関数
    void convertPoint(const Point from, const Point to) {
        if (vertex1 == from) vertex1 = to;
        else if (vertex2 == from) vertex2 = to;
    }
};
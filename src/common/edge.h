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
    //vertex1�̕����C���f�b�N�X���������Ȃ�悤�ɐݒ肷��
    Point vertex1;
    Point vertex2;

    //��r���Z�q���I�[�o�[���[�h
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

    //�Е��̓_���w�肵���_�ɕύX����֐�
    void convertPoint(const Point from, const Point to) {
        if (vertex1 == from) vertex1 = to;
        else if (vertex2 == from) vertex2 = to;
    }
};
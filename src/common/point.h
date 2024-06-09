#pragma once

#include <stdexcept>
#include "array.h"

struct Point {
public:
	Point(){
        x = 0; y = 0; z = 0;
        index = -1;
    }
    Point(double x, double y, double z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }
	//���W���
	double x=0,y=0,z=0;
    //�F���
    unsigned char r = 255, g = 255, b = 255;
    //CT�l
    unsigned short CTval = 0;
	//�ʕ�ŗׂ荇���O�p�`������Ƃ��̒��_�ԍ�
	int index=-1;
	//�_�̎��@�����
	Array normal;

	//�Y�����Z�q���I�[�o�[���[�h
	double& operator[](size_t pos) {
		if (pos < 0 || pos >= 3) {
			throw std::runtime_error("Vector index out of bounds!");
		}
		switch (pos) {
			case 0:
				return x;
			case 1:
				return y;
			case 2:
				return z;

		}
	}
	const double& operator[](size_t pos) const{
		if (pos < 0 || pos >= 3) {
			throw std::runtime_error("Vector index out of bounds!");
		}
		switch (pos) {
		case 0:
			return x;
		case 1:
			return y;
		case 2:
			return z;

		}
	}
	inline Point operator+(const Point &p) {
		Point res;
		res.x = this->x + p.x;
		res.y = this->y + p.y;
		res.z = this->z + p.z;
		return res;
	}

	inline const Point operator+(const Point& p) const{
		Point res;
		res.x = this->x + p.x;
		res.y = this->y + p.y;
		res.z = this->z + p.z;
		return res;
	}
    inline Point operator-(const Point& p) {
        Point res;
        res.x = this->x - p.x;
        res.y = this->y - p.y;
        res.z = this->z - p.z;
        return res;
    }

    inline const Point operator-(const Point& p) const {
        Point res;
        res.x = this->x - p.x;
        res.y = this->y - p.y;
        res.z = this->z - p.z;
        return res;
    }

	inline Point operator/(int num) {
		Point res;
		res.x = this->x / (double)num;
		res.y = this->y / (double)num;
		res.z = this->z / (double)num;
		return res;
	}

	inline const Point operator/(int num) const{
		Point res;
		res.x = this->x / (double)num;
		res.y = this->y / (double)num;
		res.z = this->z / (double)num;
		return res;
	}

    inline Point operator=(Point p) {
        this->x = p.x; this->y = p.y; this->z = p.z;
        this->normal.x = p.normal.x; this->normal.y = p.normal.y; this->normal.z = p.normal.z;
        this->CTval = p.CTval; this->r = p.r; this->g = p.g; this->b = p.b;
        this->index = p.index;
        return *this;
    }

    //��r���Z�q���I�[�o�[���[�h
    bool operator==(const Point rhs) {
        return index == rhs.index;
    }
    const bool operator==(const Point rhs) const {
        return index == rhs.index;
    }

    bool operator!=(const Point rhs) {
        return index != rhs.index;
    }
    const bool operator!=(const Point rhs) const {
        return index != rhs.index;
    }

    bool operator<(const Point rhs) {
        return index < rhs.index;
    }
    const bool operator<(const Point rhs) const {
        return index < rhs.index;
    }

    bool operator<=(const Point rhs) {
        return index <= rhs.index;
    }
    const bool operator<=(const Point rhs) const {
        return index <= rhs.index;
    }

    bool operator>(const Point rhs) {
        return index > rhs.index;
    }
    const bool operator>(const Point rhs) const {
        return index > rhs.index;
    }

    bool operator>=(const Point rhs) {
        return index >= rhs.index;
    }
    const bool operator>=(const Point rhs) const {
        return index >= rhs.index;
    }

	//������Point�Ƃ̋��������߂�
	const double distance(const Point p) const{
		return sqrt((x - p.x) * (x - p.x) + (y - p.y) * (y - p.y) + (z - p.z) * (z - p.z));
	}

	//������Point�Ƃ̋����̓������߂�
	const double squareDistance(const Point p) const {
		return (x - p.x) * (x - p.x) + (y - p.y) * (y - p.y) + (z - p.z) * (z - p.z);
	}
    //�����̃|�C���g�Ƃ̍����x�N�g���Ƃ��ċ��߂�
    Array differentialVector(const Point p) {
        Array res;
        res.x = x - p.x;
        res.y = y - p.y;
        res.z = z - p.z;
        return res;
    }
	const Array differentialVector(const Point p) const {
		Array res;
		res.x = x - p.x;
		res.y = y - p.y;
		res.z = z - p.z;
		return res;
	}
    //�f�o�b�O�p�̓_�̍��W�Ɩ@���̕\��
    inline void displayD() {
        printf("���W: %lf, %lf, %lf  �@��: %lf, %lf, %lf �C���f�b�N�X: %d\n", x, y, z, normal.x, normal.y, normal.z, index);
    }
    //�f�o�b�O�p�̐F�ϊ�(��)
    void convertRed() {
        r = 255; g = 0; b = 0;
    }
    //�f�o�b�O�p�̐F�ϊ�(��)
    void convertGreen() {
        r = 0; g = 255; b = 0;
    }
    //�f�o�b�O�p�̐F�ϊ�(��)
    void convertBlue() {
        r = 0; g = 0; b = 255;
    }
    //�f�o�b�O�p�̐F�ϊ�(��)
    void convertWhite() {
        r = 255; g = 255; b = 255;
    }
};
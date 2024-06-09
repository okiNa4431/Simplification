#pragma once

#include <stdexcept>
#include <cmath>
#include "point.h"

struct Array {
public:
	//���W���
	double x, y, z;
	Array() {}
    Array(double x, double y, double z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }

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
	const double& operator[](size_t pos) const {
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
    //���Z�q���I�[�o�[���[�h
    Array operator+(Array arr) {
        Array ret;
        ret.x = this->x + arr.x;
        ret.y = this->y + arr.y;
        ret.z = this->z + arr.z;
        return ret;
    }
    const Array operator+(Array arr) const {
        Array ret;
        ret.x = this->x + arr.x;
        ret.y = this->y + arr.y;
        ret.z = this->z + arr.z;
        return ret;
    }

    //*����ςƂ��ăI�[�o�[���[�h
    double operator*(Array arr) {
        return x * arr.x + y * arr.y + z * arr.z;
    }
    const double operator*(Array arr) const {
        return x * arr.x + y * arr.y + z * arr.z;
    }

    //���K��
	void normalize() {
		double norm = sqrt(x * x + y * y + z * z);
		x /= norm;
		y /= norm;
		z /= norm;
	}

    //�O��
    Array cross(Array arr) {
        Array ret;
        ret.x = this->y * arr.z - this->z * arr.y;
        ret.y = this->z * arr.x - this->x * arr.z;
        ret.z = this->x * arr.y - this->y * arr.x;
        return ret;
    }
    const Array cross(Array arr) const{
        Array ret;
        ret.x = this->y * arr.z - this->z * arr.y;
        ret.y = this->z * arr.x - this->x * arr.z;
        ret.z = this->x * arr.y - this->y * arr.x;
        return ret;
    }

    //�傫����Ԃ�
    double norm() {
        return sqrt(x * x + y * y + z * z);
    }
    const double norm() const{
        return sqrt(x * x + y * y + z * z);
    }

    //�p�x��Ԃ�
    double calcAngle(Array arr) {
        Array thisArray = *this;
        const double dot = thisArray * arr;
        const double thisNorm = thisArray.norm();
        const double arrNorm = arr.norm();
        return std::acos(dot / (thisNorm * arrNorm));
    }

    //�f�o�b�O�p�̓_�̊e�����̕\��
    inline void displayD() {
        printf("x,y,z����: %lf, %lf, %lf\n", x, y, z);
    }
};
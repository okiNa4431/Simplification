#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include "point.h"
#include "triangle.h"

enum class fileType
{
    withColor = 0x00,
    nonColor = 0x01,
};

inline void read_ply(const std::string& filename, std::vector<Point>& points, std::vector<Triangle>& triangles, int& pN) {
    std::ifstream reader(filename.c_str(), std::ios::in | std::ios::binary);
    if (reader.fail()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    //�F�t��ply�t�@�C����ǂݎ�邱�Ƃ�z��
    std::string str;
    std::getline(reader, str);  //ply
    std::getline(reader, str);  //format binary_little_endian 1.0
    reader >> str; reader >> str;   //element vertex
    reader >> str;
    pN = std::stoi(str);    //pN
    for (int i = 0; i < 12; i++) {
        std::getline(reader, str);  //property float x ~ property uchar alpha
    }
    reader >> str; reader >> str;   //element face
    reader >> str;
    int faceN = std::stoi(str);    //faceN
    for (int i = 0; i < 7; i++) {
        std::getline(reader, str);  //property list uchar int vertex_indices ~ end_header
    }

    //points�ɓ_�̍��W�ƐF���i�[
    points.resize(pN);
    for (int i = 0; i < pN; i++) {
        //���W
        float buf_Coordinate[6];
        reader.read((char*)buf_Coordinate, sizeof(float) * 6);
        points[i].x = buf_Coordinate[0]; points[i].y = buf_Coordinate[1]; points[i].z = buf_Coordinate[2];

        //�@��
        points[i].normal = Array(buf_Coordinate[3], buf_Coordinate[4], buf_Coordinate[5]);

        //�F
        unsigned char buf_Color[4];
        reader.read((char*)buf_Color, sizeof(unsigned char) * 4);
        points[i].r = buf_Color[0]; points[i].g = buf_Color[1]; points[i].b = buf_Color[2];

        //CT�l
        unsigned short buf_CT[1];
        reader.read((char*)buf_CT, sizeof(unsigned short) * 1);
        points[i].CTval = buf_CT[0];

        //�C���f�b�N�X
        points[i].index = i;
    }

    //triangles�ɓ_�̍��W�ƐF���i�[
    triangles.resize(faceN);
    for (int i = 0; i < faceN; i++) {
        const uint8_t k = 3;
        reader.read((char*)&k, sizeof(uint8_t));

        //�O�p�`�̃C���f�b�N�X��ǂݍ���
        uint32_t buf_Coordinate[3];
        reader.read((char*)buf_Coordinate, sizeof(uint32_t) * 3);
        triangles[i] = Triangle(points[buf_Coordinate[0]], points[buf_Coordinate[1]], points[buf_Coordinate[2]]);

        //�O�p�`�̐F����ǂݍ���
        unsigned char buf_Color[4];
        reader.read((char*)buf_Color, sizeof(unsigned char) * 4);
    }
}

inline void write_ply(const std::string& filename, const std::vector<Point>& points, const std::vector<Triangle>& generateTriangles) {
    std::ofstream writer(filename.c_str(), std::ios::out | std::ios::binary);
    if (writer.fail()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    //ply�t�@�C���̌`���ʂ�ɋL�q
    writer << "ply"
        << "\n";
    writer << "format binary_little_endian 1.0"
        << "\n";

    //�g���Ă��Ȃ��_���폜���A���̕��C���f�b�N�X���l�߂�
    std::vector<int> pointNum(points.size(), 0);
    //�_�̃C���f�b�N�X�ɑ΂��Ă��̓_�����g���Ă��邩�𐔂���
    for (int i = 0; i < generateTriangles.size(); i++) {
        pointNum[generateTriangles[i].vertex1.index]++;
        pointNum[generateTriangles[i].vertex2.index]++;
        pointNum[generateTriangles[i].vertex3.index]++;
    }
    //pointNum�̑��݂���_�̃C���f�b�N�X�̒l��V���ȃC���f�b�N�X�̔ԍ��ɏ���������
    int countP = 0;
    for (int i = 0; i < points.size(); i++) {
        if (pointNum[i] == 0) {
            pointNum[i] = -1;
            continue;
        }
        pointNum[i] = countP;
        countP++;
    }
    printf("�_�̐�:%d\n", countP);

    writer << "element vertex " << countP << "\n";
    writer << "property float x"
        << "\n";
    writer << "property float y"
        << "\n";
    writer << "property float z"
        << "\n";
    writer << "property float nx"
        << "\n";
    writer << "property float ny"
        << "\n";
    writer << "property float nz"
        << "\n";
    writer << "property uchar red"
        << "\n";
    writer << "property uchar green"
        << "\n";
    writer << "property uchar blue"
        << "\n";
    writer << "property uchar alpha"
        << "\n";
    writer << "property ushort quality"
        << "\n";

    //�c����L�q(�ʉ��)
    const int nFaces = (int)generateTriangles.size();
    printf("�ʂ̐�:%d\n", nFaces);
    writer << "element face " << nFaces << "\n";
    writer << "property list uchar int vertex_indices"
        << "\n";
    writer << "property uchar red"
        << "\n";
    writer << "property uchar green"
        << "\n";
    writer << "property uchar blue"
        << "\n";
    writer << "property uchar alpha"
        << "\n";
    writer << "end_header"
        << "\n";

    for (const auto& p : points) {
        if (pointNum[p.index] == -1) continue;
        float buf_Coordinate[6] = { (float)(p.x), (float)p.y, (float)(p.z), (float)p.normal.x, (float)p.normal.y, (float)p.normal.z };
        unsigned char buf_Color[4] = { p.r, p.g, p.b, 255 };
        unsigned short buf_CT[1] = { p.CTval };
        writer.write((char*)buf_Coordinate, sizeof(float) * 6);
        writer.write((char*)buf_Color, sizeof(unsigned char) * 4);
        writer.write((char*)buf_CT, sizeof(unsigned short) * 1);
    }

    for (int i = 0; i < nFaces; i++) {
        const uint8_t k = 3;
        writer.write((char*)&k, sizeof(uint8_t));
        //a,b,c�͎O�p�`���\�����钸�_�̃C���f�b�N�X
        const int a = pointNum[generateTriangles[i].vertex1.index];
        const int b = pointNum[generateTriangles[i].vertex2.index];
        const int c = pointNum[generateTriangles[i].vertex3.index];
        //�ʂ̗��\�𒲐�
        const Point A2B = generateTriangles[i].vertex2 - generateTriangles[i].vertex1;
        const Point A2C = generateTriangles[i].vertex3 - generateTriangles[i].vertex1;
        Array AB = Array(A2B.x, A2B.y, A2B.z);
        Array AC = Array(A2C.x, A2C.y, A2C.z);
        Array faceNormal = AB.cross(AC);
        double angle = faceNormal.calcAngle(generateTriangles[i].vertex1.normal);
        //���_�̖@���Ɩʂ̖@���Ƃ̊p�x�̍���90�x�𒴂��邩�ǂ����ɂ���Ĕ���
        if (angle < 1.5708) {
            uint32_t buf_Coordinate[3] = { a, b, c };
            writer.write((char*)buf_Coordinate, sizeof(uint32_t) * 3);
        }
        else {
            uint32_t buf_Coordinate[3] = { a, c, b };
            writer.write((char*)buf_Coordinate, sizeof(uint32_t) * 3);
        }

        //�F�����������ށA�ʂ̐F��3�̒��_�̐F�̕��ςƂ���
        unsigned char red = (points[a].r + points[b].r + points[c].r) / 3;
        unsigned char green = (points[a].g + points[b].g + points[c].g) / 3;
        unsigned char blue = (points[a].b + points[b].b + points[c].b) / 3;
        unsigned char buf_Color[4] = { red, green, blue, 255 };
        writer.write((char*)buf_Color, sizeof(unsigned char) * 4);
    }

    writer.close();
}
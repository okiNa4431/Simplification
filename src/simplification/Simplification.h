#pragma once

#include "common/point.h"
#include "common/triangle.h"

void deleteSameTriangles(std::vector<Triangle>& generateTriangles);
void simplifyQEM(std::vector<Point>& points, std::vector<Triangle>& generateTriangles, const int faceSize);
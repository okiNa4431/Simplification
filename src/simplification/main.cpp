#include "Simplification.h"
#include "common/io.h"

int main()
{
	//ファイルからメッシュを読み取る
	const char input[] = "D:Mesh/Bunny.ply";
	int pN;
	std::vector<Point> points;
	std::vector<Triangle> triangles;
	read_ply(input, points, triangles, pN);

	//簡略化
	simplifyQEM(points, triangles, 1000);

	//メッシュを出力
	const char output[] = "D:Mesh/SimplifiedBunny.ply";
	write_ply(output, points, triangles);
}
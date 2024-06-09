#include "Simplification.h"
#include "common/io.h"

int main()
{
	//�t�@�C�����烁�b�V����ǂݎ��
	const char input[] = "D:Mesh/Bunny.ply";
	int pN;
	std::vector<Point> points;
	std::vector<Triangle> triangles;
	read_ply(input, points, triangles, pN);

	//�ȗ���
	simplifyQEM(points, triangles, 1000);

	//���b�V�����o��
	const char output[] = "D:Mesh/SimplifiedBunny.ply";
	write_ply(output, points, triangles);
}
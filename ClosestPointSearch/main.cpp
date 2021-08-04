#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <list>

#include "CPS_AABBTree.h"
#include "MeshDefinition.h"

int main()
{
	Mesh mesh;
	MeshTools::ReadMesh(mesh, "D:/MyData/Research/Data_Models/SurfaceMesh/PumpkinMesh.obj");

	std::vector<double> coords;
	std::vector<int> indices;
	for (auto vh : mesh.vertices())
	{
		auto point = mesh.point(vh);
		coords.push_back(point[0]);
		coords.push_back(point[1]);
		coords.push_back(point[2]);
	}

	for (auto fh : mesh.faces())
	{
		for (auto fvh : mesh.fv_range(fh))
		{
			indices.push_back(fvh.idx());
		}
	}
	// old version include test on both CGAL AABB tree and mine.
	ClosestPointSearch::AABBTree tree(coords, indices);

	auto v0 = tree.closest_point_and_face_id(ClosestPointSearch::Vec3d(1, 2, 3));
	auto v1 = tree.closest_point_and_face_id(ClosestPointSearch::Vec3d(10, 20, 30));
	auto v2 = tree.closest_point_and_face_id(ClosestPointSearch::Vec3d(100, 200, 300));
	auto v3 = tree.closest_point_and_face_id(ClosestPointSearch::Vec3d(-10, -20, -30));
	std::cout << v0.first[0] << "  " << v0.first[1] << "  " << v0.first[2] << "  " << v0.second << std::endl;
	std::cout << v1.first[0] << "  " << v1.first[1] << "  " << v1.first[2] << "  " << v1.second << std::endl;
	std::cout << v2.first[0] << "  " << v2.first[1] << "  " << v2.first[2] << "  " << v2.second << std::endl;
	std::cout << v3.first[0] << "  " << v3.first[1] << "  " << v3.first[2] << "  " << v3.second << std::endl;
}

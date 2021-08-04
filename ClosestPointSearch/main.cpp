#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <list>

#include "CPS_AABBTree.h"

int main()
{
	Mesh mesh;
	MeshTools::ReadMesh(mesh, "D:/MyData/Research/Data_Models/SurfaceMesh/PumpkinMesh.obj");

	// old version include test on both CGAL AABB tree and mine.
	ClosestPointSearch::AABBTree tree(mesh);
	auto v0 = tree.closest_point_and_face_handle(OpenMesh::Vec3d(1, 2, 3));
	auto v1 = tree.closest_point_and_face_handle(OpenMesh::Vec3d(10, 20, 30));
	auto v2 = tree.closest_point_and_face_handle(OpenMesh::Vec3d(100, 200, 300));
	auto v3 = tree.closest_point_and_face_handle(OpenMesh::Vec3d(-10, -20, -30));
	std::cout << v0.first << " " << v0.second << std::endl;
	std::cout << v1.first << " " << v1.second << std::endl;
	std::cout << v2.first << " " << v2.second << std::endl;
	std::cout << v3.first << " " << v3.second << std::endl;
}

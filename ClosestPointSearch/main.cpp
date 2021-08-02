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
	std::cout << tree.closest_point(OpenMesh::Vec3d(1, 2, 3)) << std::endl;
	std::cout << tree.closest_vertex(OpenMesh::Vec3d(1, 2, 3)) << std::endl<< std::endl;
	std::cout << tree.closest_point(OpenMesh::Vec3d(10, 20, 30)) << std::endl;
	std::cout << tree.closest_vertex(OpenMesh::Vec3d(10, 20, 30)) << std::endl<< std::endl;
	std::cout << tree.closest_point(OpenMesh::Vec3d(100, 200, 300)) << std::endl;
	std::cout << tree.closest_vertex(OpenMesh::Vec3d(100, 200, 300)) << std::endl<< std::endl;
	std::cout << tree.closest_point(OpenMesh::Vec3d(-10, -20, -30)) << std::endl;
	auto result = tree.closest_vertex_and_index(OpenMesh::Vec3d(-10, -20, -30));
	std::cout << result.first << std::endl;
	std::cout << result.second << " : " << mesh.point(mesh.vertex_handle(result.second)) << std::endl;
}

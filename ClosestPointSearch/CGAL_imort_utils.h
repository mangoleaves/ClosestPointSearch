#pragma once
/*
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Kernel/global_functions.h>
#include <CGAL/IO/OBJ_reader.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>


typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::FT FT;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
typedef Kernel::Triangle_3 Triangle;

typedef CGAL::Surface_mesh<Kernel::Point_3> CGAL_Mesh;
typedef CGAL_Mesh::vertex_index vertex_index;

typedef std::vector<Triangle>::iterator Iterator;
typedef CGAL::AABB_triangle_primitive<Kernel, Iterator> Primitive;
typedef CGAL::AABB_traits<Kernel, Primitive> AABB_triangle_traits;
typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;


bool ReadObjFile(std::fstream& fs, CGAL_Mesh& mesh)
{
	// Read points and faces from .obj file
	std::vector<Point> points;
	std::vector<std::vector<std::size_t>> faces;

	if (!fs.is_open() || !CGAL::read_OBJ(fs, points, faces))
	{
		std::cerr << "Failed to read obj file" << std::endl;
		return false;
	}
	// Add points into mesh
	std::vector<vertex_index> vertices;
	vertices.reserve(points.size());

	for (const auto& point : points)
	{
		vertices.emplace_back(mesh.add_vertex(point));
	}
	// Add faces into mesh
	for (const auto& face : faces)
	{
		if (mesh.add_face(vertices[face[0]], vertices[face[1]], vertices[face[2]])
			== CGAL_Mesh::null_face())
		{
			std::cerr << "Error occurred while adding faces." << std::endl;
			return false;
		}
	}
	return true;
}

std::array<Point, 3> FacePoints(CGAL_Mesh& mesh, CGAL_Mesh::face_index& face)
{
	std::array<Point, 3> points;
	auto v_iter = mesh.vertices_around_face(mesh.halfedge(face)).begin();
	points[0] = mesh.point(*v_iter++);
	points[1] = mesh.point(*v_iter++);
	points[2] = mesh.point(*v_iter);
	return points;
}

Triangle Face2Triangle(CGAL_Mesh& mesh, CGAL_Mesh::face_index& face)
{
	auto v = FacePoints(mesh, face);
	return Triangle(v[0], v[1], v[2]);
}

bool InitAABBTree(CGAL_Mesh& mesh, Tree& tree, std::vector<Triangle>& triangles)
{
	for (auto& face : mesh.faces())
	{
		triangles.emplace_back(Face2Triangle(mesh, face));
	}

	tree.insert(triangles.begin(), triangles.end());
	tree.build();
	tree.accelerate_distance_queries();
	return true;
}
*/

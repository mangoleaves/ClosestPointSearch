#pragma once
#include <OpenMesh/Core/Geometry/VectorT.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <queue>

#ifdef DEBUG
#pragma comment(lib, "OpenMeshCored.lib")
#pragma comment(lib, "OpenMeshToolsd.lib")
#else
#pragma comment(lib, "OpenMeshCore.lib")
#pragma comment(lib, "OpenMeshTools.lib")
#endif
struct MeshTraits : public OpenMesh::DefaultTraits
{
	typedef OpenMesh::Vec3d Point;
	typedef OpenMesh::Vec3d Normal;
	typedef OpenMesh::Vec2d TexCoord2D;

	typedef typename std::pair<Point, Point> Link;               // for local links
	typedef typename std::vector<Link> Link_vector;                        // for out links
	typedef typename std::deque<Link> Link_deque;            // for in links				

	VertexAttributes(OpenMesh::Attributes::Status | OpenMesh::Attributes::TexCoord2D | OpenMesh::Attributes::Normal);
	FaceAttributes(OpenMesh::Attributes::Status | OpenMesh::Attributes::Normal);
	EdgeAttributes(OpenMesh::Attributes::Status);
	HalfedgeAttributes(OpenMesh::Attributes::Status);

	FaceTraits
	{
	public:
		int m_tag = 0;                        // general-purpose tag
		// links
		Link_vector m_face_out_links;      // point links from this facet
		Link_deque m_face_in_links;  // point links to this facet
		Link_deque m_edge_in_links;
		Link_deque m_vertex_in_links;
	};

	EdgeTraits
	{
	public:
		// links
		Link_vector m_edge_out_links;         // point link from this edge
	};

	HalfedgeTraits
	{
		HalfedgeT() :face_he_var(-1)
		{
		};
	private:
		int face_he_var;
	public:
		int get_face_he_var()const { return face_he_var; };
		void set_face_he_var(int fhe) { face_he_var = fhe; };
	};

	VertexTraits
	{
		VertexT() : new_pos_fixed(false)
		{
		};
	private:
		OpenMesh::Vec3d new_pos;//can be used for deformation and parameterization
		bool new_pos_fixed;
	public:
		void set_New_Pos(const OpenMesh::Vec3d& n_p) { new_pos = n_p; }
		const OpenMesh::Vec3d& get_New_Pos()const { return new_pos; }
		void set_new_pos_fixed(bool f) { new_pos_fixed = f; }
		bool get_new_pos_fixed()const { return new_pos_fixed; }

		// links
		Link m_vertex_out_link;                 // Links from this vertex
	};
};

typedef OpenMesh::TriMesh_ArrayKernelT<MeshTraits> Mesh;
/*
bool check_in_triangle_face(const std::vector<OpenMesh::Vec3d>& tri, const OpenMesh::Vec3d& p);
bool baryCoord(const OpenMesh::Vec3d& _p, const OpenMesh::Vec3d& _u, const OpenMesh::Vec3d& _v, const OpenMesh::Vec3d& _w, OpenMesh::Vec3d&_result);

void compute_point_area(Mesh* mesh_, std::vector<std::map<int, double>>& cornerArea, std::vector<double>& pointArea, bool use_np = false);

void rot_coord_sys(const OpenMesh::Vec3d &old_u, const OpenMesh::Vec3d &old_v,
	const OpenMesh::Vec3d &new_norm,
	OpenMesh::Vec3d &new_u, OpenMesh::Vec3d &new_v);

void proj_curv(const OpenMesh::Vec3d &old_u, const OpenMesh::Vec3d &old_v,
	double old_ku, double old_kuv, double old_kv,
	const OpenMesh::Vec3d &new_u, const OpenMesh::Vec3d &new_v,
	double &new_ku, double &new_kuv, double &new_kv);

// Given a curvature tensor, find principal directions and curvatures
// Makes sure that pdir1 and pdir2 are perpendicular to normal
void diagonalize_curv(const OpenMesh::Vec3d &old_u, const OpenMesh::Vec3d &old_v,
	double ku, double kuv, double kv,
	const OpenMesh::Vec3d &new_norm,
	OpenMesh::Vec3d &pdir1, OpenMesh::Vec3d &pdir2, double &vk1, double &vk2);

void compute_principal_curvature(Mesh* mesh_,
	std::vector<double>& K1, std::vector<double>& K2,
	std::vector<OpenMesh::Vec3d>& dir1, std::vector<OpenMesh::Vec3d>& dir2);
	*/
#pragma once

#include "CPS_Vector.h"
#include "CPS_BoundingBox.h"

namespace ClosestPointSearch
{
#ifdef USE_VEC
	class Triangle
	{
	public:
		Vec3d ver0, ver1, ver2;
		// used for accelarating.
		Vec3d face_normal;
		Vec3d edge_vec01, edge_vec02, edge_vec12;
		Vec3d edge_nor01, edge_nor02, edge_nor12;
		double edge_sqrlen01, edge_sqrlen02, edge_sqrlen12;
		bool is_plane_degenerate;
		bool has_obtuse_angle;
		// used for recording openmesh face handle
		OpenMesh::FaceHandle face_handle;
	public:
		Triangle() {}

		Triangle(const Vec3d& p1, const Vec3d& p2, const Vec3d& p3)
		{
			Vec3d p12 = p2 - p1;
			Vec3d p23 = p3 - p2;
			Vec3d p31 = p1 - p3;
			// find obtuse angle, move obtuse angle point to ver0
			if (p12.dot(p31) > 0)
			{
				has_obtuse_angle = true;
				ver0 = p1;	ver1 = p2;	ver2 = p3;
			}
			else if (p12.dot(p23) > 0)
			{
				has_obtuse_angle = true;
				ver0 = p2;	ver1 = p3;	ver2 = p1;
			}
			else if (p23.dot(p31) > 0)
			{
				has_obtuse_angle = true;
				ver0 = p3;	ver1 = p1;	ver2 = p2;
			}
			else
			{
				has_obtuse_angle = false;
				ver0 = p1;	ver1 = p2;	ver2 = p3;
			}
			// calculate edge vector and squared length
			edge_vec01 = ver1 - ver0;
			edge_sqrlen01 = edge_vec01.squaredNorm();
			edge_vec02 = ver2 - ver0;
			edge_sqrlen02 = edge_vec02.squaredNorm();
			edge_vec12 = ver2 - ver1;
			edge_sqrlen12 = edge_vec12.squaredNorm();
			// calculate face normal
			face_normal = edge_vec01.cross(edge_vec02).normalized();
			is_plane_degenerate = face_normal.x == 0.0 && face_normal.y == 0.0 && face_normal.z == 0.0;
			// calculate edge normal
			if (!is_plane_degenerate)
			{
				edge_nor01 = (edge_vec01).cross(face_normal).normalized();
				edge_nor02 = (-edge_vec02).cross(face_normal).normalized();
				edge_nor12 = (edge_vec12).cross(face_normal).normalized();
			}
		}

		Triangle(const Vec3d& p1, const Vec3d& p2, const Vec3d& p3, const OpenMesh::FaceHandle& fh)
			:Triangle(p1, p2,p3)
		{
			face_handle = fh;
		}

		Vec3d normal()
		{
			return face_normal;
		}

		Vec3d normal() const
		{
			return face_normal;
		}

		Vec3d centroid()
		{
			return (ver0 + ver1 + ver2) / 3.0;
		}

		Vec3d centroid() const
		{
			return (ver0 + ver1 + ver2) / 3.0;
		}

		BoundingBox compute_bbox() const
		{
			Vec3d minBound, maxBound;
			// for x
			if (ver0.x <= ver1.x)
			{
				minBound.x = ver0.x;
				maxBound.x = ver1.x;
			}
			else
			{
				minBound.x = ver1.x;
				maxBound.x = ver0.x;
			}
			if (minBound.x > ver2.x)
			{
				minBound.x = ver2.x;
			}
			else if (maxBound.x < ver2.x)
			{
				maxBound.x = ver2.x;
			}
			// for y
			if (ver0.y <= ver1.y)
			{
				minBound.y = ver0.y;
				maxBound.y = ver1.y;
			}
			else
			{
				minBound.y = ver1.y;
				maxBound.y = ver0.y;
			}
			if (minBound.y > ver2.y)
			{
				minBound.y = ver2.y;
			}
			else if (maxBound.y < ver2.y)
			{
				maxBound.y = ver2.y;
			}
			// for z
			if (ver0.z <= ver1.z)
			{
				minBound.z = ver0.z;
				maxBound.z = ver1.z;
			}
			else
			{
				minBound.z = ver1.z;
				maxBound.z = ver0.z;
			}
			if (minBound.z > ver2.z)
			{
				minBound.z = ver2.z;
			}
			else if (maxBound.z < ver2.z)
			{
				maxBound.z = ver2.z;
			}
			return BoundingBox(minBound, maxBound);
		}

		double closest_point(const Vec3d& query, Vec3d& result) const
		{
			result = closest_point(query);
			return (result - query).squaredNorm();
		}

		Vec3d closest_point(const Vec3d& query) const
		{
			if (is_plane_degenerate)
			{
				// If the plane is degenerate, then the triangle is degenerate, and
				// one tries to find to which segment it is equivalent.
				auto segment = find_longest_segment();
				if (is_segment_degerate(segment))
				{
					return *(segment.first);
				}

				return project_to_segment(segment, query);
			}

			if (!has_obtuse_angle)
			{
				Vec3d ver0_to_query = query - ver0;
				if (ver0_to_query.dot(edge_nor01) > 0.0)	// is outside edge 01 ?
				{
					double numerator = ver0_to_query.dot(edge_vec01);
					if (numerator < 0.0)
					{
						return ver0;
					}
					else if (numerator > edge_sqrlen01)
					{
						return ver1;
					}
					else
					{
						return ver0 + (numerator / edge_sqrlen01) * edge_vec01;
					}
				}
				else if (ver0_to_query.dot(edge_nor02) > 0.0)	// is outside edge 02 ?
				{
					double numerator = ver0_to_query.dot(edge_vec02);
					if (numerator < 0.0)
					{
						return ver0;
					}
					else if (numerator > edge_sqrlen02)
					{
						return ver1;
					}
					else
					{
						return ver0 + (numerator / edge_sqrlen02) * edge_vec02;
					}
				}
			}
			else
			{
				Vec3d ver0_to_query = query - ver0;
				if (ver0_to_query.dot(edge_nor01) > 0.0)	// is outside edge 01?
				{
					double numerator01 = ver0_to_query.dot(edge_vec01);
					if (numerator01 > edge_sqrlen01)
					{
						return ver1;
					}
					else if (numerator01 < 0.0)
					{
						// must also outside edge 02
						double numerator02 = ver0_to_query.dot(edge_vec02);
						if (numerator02 > edge_sqrlen02)
						{
							return ver2;
						}
						else if (numerator02 < 0.0)
						{
							return ver0;
						}
						else
						{
							return ver0 + (numerator02 / edge_sqrlen02) * edge_vec02;
						}
					}
					else
					{
						return ver0 + (numerator01 / edge_sqrlen01) * edge_vec01;
					}
				}
				else if (ver0_to_query.dot(edge_nor02) > 0.0)	// is outside edge 02?
				{
					double numerator = ver0_to_query.dot(edge_vec02);
					if (numerator > edge_sqrlen02)
					{
						return ver2;
					}
					else
					{
						return ver0 + (numerator / edge_sqrlen02) * edge_vec02;
					}
				}
			}
			Vec3d ver1_to_query = query - ver1;
			if (ver1_to_query.dot(edge_nor12) > 0.0)	// is outside edge 12 ?
			{
				double numerator = ver1_to_query.dot(edge_vec12);
				if (numerator < 0.0)
				{
					return ver1;
				}
				else if (numerator > edge_sqrlen12)
				{
					return ver2;
				}
				else
				{
					return ver1 + (numerator / edge_sqrlen12) * edge_vec12;
				}
			}
			double propotion = (ver1_to_query).dot(face_normal);	// is inside triangle!
			return query - propotion * face_normal;
		}
	private:
		bool is_segment_degerate(std::pair<const Vec3d*, const Vec3d*> segment) const
		{
			return *(segment.first) == *(segment.second);
		}

		std::pair<const Vec3d*, const Vec3d*> find_longest_segment()const
		{
			if (edge_sqrlen01 > edge_sqrlen12)
			{
				if (edge_sqrlen01 > edge_sqrlen02)
				{
					return std::pair<const Vec3d*, const Vec3d*>(&ver0, &ver1);
				}
				else
				{
					return std::pair<const Vec3d*, const Vec3d*>(&ver2, &ver0);
				}
			}
			else
			{
				if (edge_sqrlen12 > edge_sqrlen02)
				{
					return std::pair<const Vec3d*, const Vec3d*>(&ver1, &ver2);
				}
				else
				{
					return std::pair<const Vec3d*, const Vec3d*>(&ver2, &ver0);
				}
			}
		}

		Vec3d project_to_segment(const std::pair<const Vec3d*, const Vec3d*> segment, const Vec3d& query)const
		{
			// degeneration is a rare case, so I didn't optimize this function.
			double segment_to_vector_x = segment.second->x - segment.first->x,
				segment_to_vector_y = segment.second->y - segment.first->y,
				segment_to_vector_z = segment.second->z - segment.first->z;

			double source_to_query_x = query.x - segment.first->x,
				source_to_query_y = query.y - segment.first->y,
				source_to_query_z = query.z - segment.first->z;


			double dot1 = segment_to_vector_x * source_to_query_x
				+ segment_to_vector_y * source_to_query_y
				+ segment_to_vector_z * source_to_query_z;

			if (dot1 <= 0)
			{
				return *(segment.first);
			}
			else
			{
				double target_to_query_x = query.x - segment.second->x,
					target_to_query_y = query.y - segment.second->y,
					target_to_query_z = query.z - segment.second->z;

				double dot2 = segment_to_vector_x * target_to_query_x
					+ segment_to_vector_y * target_to_query_y
					+ segment_to_vector_z * target_to_query_z;

				if (dot2 >= 0)
				{
					return *(segment.second);
				}
				else
				{
					double num = dot1;
					double dom = segment_to_vector_x * segment_to_vector_x + segment_to_vector_y * segment_to_vector_y + segment_to_vector_z * segment_to_vector_z;
					return Vec3d(segment.first->x + (num / dom) * segment_to_vector_x, segment.first->y + (num / dom) * segment_to_vector_y, segment.first->z + (num / dom) * segment_to_vector_z);
				}
			}
		}
	};
#endif //USE_VEC
#ifdef USE_AVX
	class Triangle
	{
	public:
		Vec3d ver0, ver1, ver2;
		// used for accelarating.
		Vec3d face_normal;
		Vec3d edge_vec01, edge_vec02, edge_vec12;
		Vec3d edge_nor01, edge_nor02, edge_nor12;
		double edge_sqrlen01, edge_sqrlen02, edge_sqrlen12;
		bool is_plane_degenerate;
		bool has_obtuse_angle;
		// used for recording openmesh face handle
		int face_id;
	public:
		Triangle() {}

		Triangle(const Vec3d& p1, const Vec3d& p2, const Vec3d& p3)
		{
			Vec3d p12 = p2 - p1;
			Vec3d p23 = p3 - p2;
			Vec3d p31 = p1 - p3;
			// find obtuse angle, move obtuse angle point to ver0
			if (p12.dot(p31) > 0)
			{
				has_obtuse_angle = true;
				ver0 = p1;	ver1 = p2;	ver2 = p3;
			}
			else if (p12.dot(p23) > 0)
			{
				has_obtuse_angle = true;
				ver0 = p2;	ver1 = p3;	ver2 = p1;
			}
			else if (p23.dot(p31) > 0)
			{
				has_obtuse_angle = true;
				ver0 = p3;	ver1 = p1;	ver2 = p2;
			}
			else
			{
				has_obtuse_angle = false;
				ver0 = p1;	ver1 = p2;	ver2 = p3;
			}
			// calculate edge vector and squared length
			edge_vec01 = ver1 - ver0;
			edge_sqrlen01 = edge_vec01.squaredNorm();
			edge_vec02 = ver2 - ver0;
			edge_sqrlen02 = edge_vec02.squaredNorm();
			edge_vec12 = ver2 - ver1;
			edge_sqrlen12 = edge_vec12.squaredNorm();
			// calculate face normal
			face_normal = edge_vec01.cross(edge_vec02).normalized();
			is_plane_degenerate = face_normal == Vec3d(0, 0, 0);
			// calculate edge normal
			if (!is_plane_degenerate)
			{
				edge_nor01 = (edge_vec01).cross(face_normal).normalized();
				edge_nor02 = (-edge_vec02).cross(face_normal).normalized();
				edge_nor12 = (edge_vec12).cross(face_normal).normalized();
			}
		}

		Triangle(const Vec3d& p1, const Vec3d& p2, const Vec3d& p3, int fid)
			:Triangle(p1, p2,p3)
		{
			face_id = fid;
		}

		Vec3d normal()
		{
			return face_normal;
		}

		Vec3d normal() const
		{
			return face_normal;
		}

		Vec3d centroid()
		{
			return (ver0 + ver1 + ver2) / 3.0;
		}

		Vec3d centroid() const
		{
			return (ver0 + ver1 + ver2) / 3.0;
		}

		BoundingBox compute_bbox() const
		{
			Vec3d minBound(_mm256_min_pd(_mm256_min_pd(ver0.vec, ver1.vec), ver2.vec)),
				maxBound(_mm256_max_pd(_mm256_max_pd(ver0.vec, ver1.vec), ver2.vec));
			return BoundingBox(minBound, maxBound);
		}

		double closest_point(const Vec3d& query, Vec3d& result) const
		{
			result = closest_point(query);
			return (result - query).squaredNorm();
		}

		Vec3d closest_point(const Vec3d& query) const
		{
			if (is_plane_degenerate)
			{
				// If the plane is degenerate, then the triangle is degenerate, and
				// one tries to find to which segment it is equivalent.
				auto segment = find_longest_segment();
				if (is_segment_degerate(segment))
				{
					return *(segment.first);
				}

				return project_to_segment(segment, query);
			}

			if (!has_obtuse_angle)
			{
				Vec3d ver0_to_query = query - ver0;
				if (ver0_to_query.dot(edge_nor01) > 0.0)	// is outside edge 01 ?
				{
					double numerator = ver0_to_query.dot(edge_vec01);
					if (numerator < 0.0)
					{
						return ver0;
					}
					else if (numerator > edge_sqrlen01)
					{
						return ver1;
					}
					else
					{
						return ver0 + (numerator / edge_sqrlen01) * edge_vec01;
					}
				}
				else if (ver0_to_query.dot(edge_nor02) > 0.0)	// is outside edge 02 ?
				{
					double numerator = ver0_to_query.dot(edge_vec02);
					if (numerator < 0.0)
					{
						return ver0;
					}
					else if (numerator > edge_sqrlen02)
					{
						return ver1;
					}
					else
					{
						return ver0 + (numerator / edge_sqrlen02) * edge_vec02;
					}
				}
			}
			else
			{
				Vec3d ver0_to_query = query - ver0;
				if (ver0_to_query.dot(edge_nor01) > 0.0)	// is outside edge 01?
				{
					double numerator01 = ver0_to_query.dot(edge_vec01);
					if (numerator01 > edge_sqrlen01)
					{
						return ver1;
					}
					else if (numerator01 < 0.0)
					{
						// must also outside edge 02
						double numerator02 = ver0_to_query.dot(edge_vec02);
						if (numerator02 > edge_sqrlen02)
						{
							return ver2;
						}
						else if (numerator02 < 0.0)
						{
							return ver0;
						}
						else
						{
							return ver0 + (numerator02 / edge_sqrlen02) * edge_vec02;
						}
					}
					else
					{
						return ver0 + (numerator01 / edge_sqrlen01) * edge_vec01;
					}
				}
				else if (ver0_to_query.dot(edge_nor02) > 0.0)	// is outside edge 02?
				{
					double numerator = ver0_to_query.dot(edge_vec02);
					if (numerator > edge_sqrlen02)
					{
						return ver2;
					}
					else
					{
						return ver0 + (numerator / edge_sqrlen02) * edge_vec02;
					}
				}
			}
			Vec3d ver1_to_query = query - ver1;
			if (ver1_to_query.dot(edge_nor12) > 0.0)	// is outside edge 12 ?
			{
				double numerator = ver1_to_query.dot(edge_vec12);
				if (numerator < 0.0)
				{
					return ver1;
				}
				else if (numerator > edge_sqrlen12)
				{
					return ver2;
				}
				else
				{
					return ver1 + (numerator / edge_sqrlen12) * edge_vec12;
				}
			}
			double propotion = (ver1_to_query).dot(face_normal);	// is inside triangle!
			return query - propotion * face_normal;
		}
	private:
		bool is_segment_degerate(std::pair<const Vec3d*, const Vec3d*> segment) const
		{
			return *(segment.first) == *(segment.second);
		}

		std::pair<const Vec3d*, const Vec3d*> find_longest_segment()const
		{
			if (edge_sqrlen01 > edge_sqrlen12)
			{
				if (edge_sqrlen01 > edge_sqrlen02)
				{
					return std::pair<const Vec3d*, const Vec3d*>(&ver0, &ver1);
				}
				else
				{
					return std::pair<const Vec3d*, const Vec3d*>(&ver2, &ver0);
				}
			}
			else
			{
				if (edge_sqrlen12 > edge_sqrlen02)
				{
					return std::pair<const Vec3d*, const Vec3d*>(&ver1, &ver2);
				}
				else
				{
					return std::pair<const Vec3d*, const Vec3d*>(&ver2, &ver0);
				}
			}
		}

		Vec3d project_to_segment(const std::pair<const Vec3d*, const Vec3d*> segment, const Vec3d& query)const
		{
			// degeneration is a rare case, so I didn't optimize this function.
			Vec3d segment_to_vector = *segment.second - *segment.first;
			Vec3d source_to_query_x = query - *segment.first;

			double dot1 = segment_to_vector.dot(source_to_query_x);

			if (dot1 <= 0)
			{
				return *(segment.first);
			}
			else
			{
				Vec3d target_to_query = query - *segment.second;

				double dot2 = segment_to_vector.dot(target_to_query);

				if (dot2 >= 0)
				{
					return *(segment.second);
				}
				else
				{
					double num = dot1;
					double dom = segment_to_vector.squaredNorm();
					return Vec3d(*segment.first + (num / dom) * segment_to_vector);
				}
			}
		}
	};
#endif // USE_AVX
	typedef Triangle* TrianglePtr;
	typedef std::vector<Triangle> Triangles;
	typedef Triangles::iterator TriIter;
	typedef Triangles::const_iterator ConstTriIter;
}

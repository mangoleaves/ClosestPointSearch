#pragma once

#include "CPS_Vector.h"
#include "CPS_BoundingBox.h"
#include "CPS_Triangle.h"
#include "CPS_KdTree.h"

namespace ClosestPointSearch
{

	/* All of the followings are modified from CGAL::AABB_tree */

	class Projection_traits
	{
	private:
		Vec3d m_query;
		Vec3d m_closest_point;
		double m_square_distance;
		TrianglePtr m_closest_triangle;
	public:
		Projection_traits(const Vec3d& query, const Vec3d& hint, TrianglePtr hint_triangle)
			:m_query(query),
			m_closest_point(hint),
			m_closest_triangle(hint_triangle),
			m_square_distance((query - hint).squaredNorm())
		{}

		void intersection(Triangle& tri)
		{
			Vec3d new_closest_point;
			double new_square_distance = tri.closest_point(m_query, new_closest_point);
			if (new_square_distance < m_square_distance)
			{
				m_closest_point = new_closest_point;
				m_square_distance = new_square_distance;
				m_closest_triangle = &tri;
			}
		}

		inline bool do_intersect(const BoundingBox& bbox) const
		{

			return bbox.do_intersect_sphere(m_query, m_square_distance);
		}

		Vec3d closest_point() const { return m_closest_point; }

		TrianglePtr primitive()const
		{
			return m_closest_triangle;
		}

		double square_distance() const { return m_square_distance; }
	};

	class AABBNode
	{
	public:
		typedef AABBNode Node;
	private:
		BoundingBox m_bbox;
		void* m_p_left_child, * m_p_right_child;
	public:
		AABBNode()
			:m_bbox(),
			m_p_left_child(nullptr),
			m_p_right_child(nullptr)
		{}

		~AABBNode() {}

		const BoundingBox& bbox() const { return m_bbox; }
		const Node& left_child() const
		{
			return *static_cast<Node*>(m_p_left_child);
		}
		const Node& right_child() const
		{
			return *static_cast<Node*>(m_p_right_child);
		}
		const Triangle& left_data() const
		{
			return *static_cast<TrianglePtr>(m_p_left_child);
		}
		const Triangle& right_data() const
		{
			return *static_cast<TrianglePtr>(m_p_right_child);
		}

		BoundingBox compute_bbox(TriIter first, TriIter beyond)
		{
			BoundingBox bbox = first->compute_bbox();
			for (++first; first != beyond; ++first)
			{
				bbox += first->compute_bbox();
			}
			return bbox;
		}

		void split_primitives(TriIter first, TriIter beyond, const BoundingBox& box)
		{
			TriIter middle = first + (beyond - first) / 2;
#ifdef USE_VEC
			switch (box.longest_axis())
			{
			case 0:	// sort along x
				std::nth_element(first, middle, beyond, [](Triangle& lhs, Triangle& rhs) {return lhs.ver0.x < rhs.ver0.x; });
				break;
			case 1: // sort along y
				std::nth_element(first, middle, beyond, [](Triangle& lhs, Triangle& rhs) {return lhs.ver0.y < rhs.ver0.y; });
				break;
			case 2: // sort along z
				std::nth_element(first, middle, beyond, [](Triangle& lhs, Triangle& rhs) {return lhs.ver0.z < rhs.ver0.z; });
				break;
			default:
				assert(0);
				break;
			}
#endif // USE_VEC
#ifdef USE_AVX
			int split_dim = box.longest_axis();
			std::nth_element(first, middle, beyond, [&](Triangle& lhs, Triangle& rhs) {return lhs.ver0.less_on(split_dim, rhs.ver0); });
#endif	// USE_AVX
		}

		void expand(TriIter first, TriIter beyond, const size_t range)
		{
			m_bbox = compute_bbox(first, beyond);
			split_primitives(first, beyond, m_bbox);

			switch (range)
			{
			case 2:
				m_p_left_child = &(*first);
				m_p_right_child = &(*(++first));
				break;
			case 3:
				m_p_left_child = &(*first);
				m_p_right_child = static_cast<Node*>(this) + 1;
				right_child().expand(first + 1, beyond, 2);
				break;
			default:
				const size_t new_range = range / 2;

				m_p_left_child = static_cast<Node*>(this) + 1;
				m_p_right_child = static_cast<Node*>(this) + new_range;
				left_child().expand(first, first + new_range, new_range);
				right_child().expand(first + new_range, beyond, range - new_range);
			}
		}

		void traversal(Projection_traits& traits, const size_t nb_primitives)
		{
			switch (nb_primitives)
			{
			case 2:
				traits.intersection(left_data());
				traits.intersection(right_data());
				break;
			case 3:
				traits.intersection(left_data());
				if (traits.do_intersect(right_child().bbox()))
				{
					right_child().traversal(traits, 2);
				}
				break;
			default:
				if (traits.do_intersect(left_child().bbox()))
				{
					left_child().traversal(traits, nb_primitives / 2);
					if (traits.do_intersect(right_child().bbox()))
					{
						right_child().traversal(traits, nb_primitives - nb_primitives / 2);
					}
				}
				else if (traits.do_intersect(right_child().bbox()))
				{
					right_child().traversal(traits, nb_primitives - nb_primitives / 2);
				}
			}
		}
	private:
		Node& left_child() { return *static_cast<Node*>(m_p_left_child); }
		Node& right_child() { return *static_cast<Node*>(m_p_right_child); }
		Triangle& left_data() { return *static_cast<TrianglePtr>(m_p_left_child); }
		Triangle& right_data() { return *static_cast<TrianglePtr>(m_p_right_child); }
	};

	class AABBTree
	{
	private:
		typedef AABBNode Node;
	private:
		Triangles m_primitives;	// store iterators from triangles(typename vector<Triangle>)
		Node* m_p_root_node = nullptr;
		KdTree* m_p_search_tree = nullptr;
	public:
		AABBTree() {}

		AABBTree(const std::vector<double> coords, const std::vector<int> indices)
		{
			insert(coords, indices);
			build();
		}

		~AABBTree()
		{
			clear();
		}

		void insert(const std::vector<double> coords, const std::vector<int> indices)
		{
			clear();

			std::vector<Vec3d> points;
			points.reserve(coords.size() / 3);
			for (int i = 0; i < coords.size(); i += 3)
			{
				points.emplace_back(coords[i], coords[i + 1], coords[i + 2]);
			}

			m_primitives.reserve(indices.size() / 3);
			for (int face_id = 0; face_id < indices.size() / 3; face_id++)
			{
				int v0 = indices[face_id * 3];
				int v1 = indices[face_id * 3 + 1];
				int v2 = indices[face_id * 3 + 2];

				m_primitives.emplace_back(points[v0], points[v1], points[v2], face_id);
			}
		}

		void build()
		{
			clear_nodes();
			if (m_primitives.size() > 1)
			{
				// allocate tree nodes.
				m_p_root_node = new Node[m_primitives.size() - 1]();
				if (m_p_root_node == nullptr)
				{
					clear();
					throw std::bad_alloc();
				}
				// construct AABB tree.
				m_p_root_node->expand(m_primitives.begin(), m_primitives.end(), m_primitives.size());
				// build search tree
				build_kd_tree();
			}
		}

		size_t size() const { return m_primitives.size(); }
		bool empty() const { return m_primitives.empty(); }

		void clear()
		{
			clear_nodes();
			m_primitives.clear();
			clear_search_tree();
		}

		Vec3d closest_point(const Vec3d& query)
		{
			auto hint = best_hint(query);
			return closest_point(query, hint).first;
		}

		std::pair<Vec3d, int> closest_point_and_face_id(const Vec3d& query)
		{
			auto hint = best_hint(query);
			std::pair<Vec3d, TrianglePtr> result = closest_point(query, hint);
			return std::pair<Vec3d, int>(result.first, result.second->face_id);
		}

		std::vector<Vec3d> closest_point(const std::vector<Vec3d>& queries)
		{
			std::vector<Vec3d> result;
			result.reserve(queries.size());

			auto hint = best_hint(queries[0]);

			for (const Vec3d& query : queries)
			{
				Vec3d cp = closest_point(query, hint).first;
				result.emplace_back(cp);
			}
			return result;
		}

		std::vector<std::pair<Vec3d, int>> closest_point_and_face_id(const std::vector<Vec3d>& queries)
		{
			std::vector<std::pair<Vec3d, int>> result;
			result.reserve(queries.size());

			auto hint = best_hint(queries[0]);

			for (const Vec3d& query : queries)
			{
				auto res = closest_point(query, hint);
				result.emplace_back(res.first, res.second->face_id);
			}
			return result;
		}

		double closest_distance(const Vec3d& query)
		{
			auto hint = best_hint(query);
			return closest_distance(query, hint).first;
		}

		std::pair<double, int> closest_distance_and_face_id(const Vec3d& query)
		{
			auto hint = best_hint(query);
			auto res = closest_distance(query, hint);
			return std::pair<double, int>(res.first, res.second->face_id);
		}

	private:
		inline std::pair<Vec3d, TriIter> best_hint(const Vec3d& query) const
		{
			return m_p_search_tree->search_nearest_point(query);
		}

		std::pair<Vec3d, const TrianglePtr> closest_point(const Vec3d& query, const std::pair<Vec3d, TriIter>& hint)
		{
			Projection_traits projection_traits(query, hint.first, &(*hint.second));
			this->traversal(projection_traits);
			return std::pair<Vec3d, const TrianglePtr>(projection_traits.closest_point(), projection_traits.primitive());
		}

		std::pair<double, const TrianglePtr> closest_distance(const Vec3d& query, const std::pair<Vec3d, TriIter>& hint)
		{
			Projection_traits projection_traits(query, hint.first, &(*hint.second));
			this->traversal(projection_traits);
			return std::pair<double, const TrianglePtr>(std::sqrt(projection_traits.square_distance()), projection_traits.primitive());
		}

		void traversal(Projection_traits& traits)
		{
			switch (size())
			{
			case 0:
				break;
			case 1:
				traits.intersection(m_primitives[0]);
				break;
			default:
				m_p_root_node->traversal(traits, m_primitives.size());
			}
		}

		void build_kd_tree()
		{
			std::vector<Vec3d> points;
			std::vector<TriIter> iters;
			points.reserve(m_primitives.size());
			iters.reserve(m_primitives.size());
			for (TriIter iter = m_primitives.begin(); iter != m_primitives.end(); iter++)
			{
				points.emplace_back((*iter).ver0);
				iters.emplace_back(iter);
			}
			m_p_search_tree = new KdTree(points, iters);
			if (m_p_search_tree == nullptr)
			{
				clear();
				throw std::bad_alloc();
			}
		}

		void clear_search_tree()
		{
			if (m_p_search_tree)
			{
				delete m_p_search_tree;
				m_p_search_tree = nullptr;
			}
		}

		void clear_nodes()
		{
			if (size() > 1)
			{
				delete[] m_p_root_node;
			}
			m_p_root_node = nullptr;
		}
	};
}

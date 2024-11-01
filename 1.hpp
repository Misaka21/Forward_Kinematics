#include <Eigen/Dense>
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <optional>
#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <queue>
#include <memory>
#include <type_traits>
#include <cmath>

template <typename Scalar, int Dim>
class GenericCoordinateTransform : public std::enable_shared_from_this<GenericCoordinateTransform<Scalar, Dim>>
{
public:
	using VectorType = Eigen::Matrix<Scalar, Dim, 1>;
	using MatrixType = Eigen::Matrix<Scalar, Dim + 1, Dim + 1>;

	struct TransformData
	{
		MatrixType transform;
		bool is_inverse;
	};

	static std::shared_ptr<GenericCoordinateTransform> create()
	{
		return std::make_shared<GenericCoordinateTransform>();
	}

	void add_transform(const std::string &from, const std::string &to, const MatrixType &transform)
	{
		auto &forward = transforms[from][to];
		auto &backward = transforms[to][from];

		if (forward)
		{
			std::cout << "Warning: Overwriting existing transform from " << from << " to " << to << std::endl;
		}

		forward = std::make_shared<TransformData>(TransformData{transform, false});
		backward = std::make_shared<TransformData>(TransformData{transform.inverse(), true});
	}
	std::optional<MatrixType> get_transform(const std::string &from, const std::string &to) const
	{
		if (from == to)
			return MatrixType::Identity();

		std::vector<std::vector<std::string>> all_paths = find_all_paths(from, to);

		if (all_paths.empty())
			return std::nullopt;

		const Scalar epsilon = 1e-6; // Tolerance for floating-point comparisons
		std::optional<MatrixType> result;

		for (const auto &path : all_paths)
		{
			MatrixType current_transform = MatrixType::Identity();
			bool valid_path = true;

			for (size_t i = 0; i < path.size() - 1; ++i)
			{
				const auto &current = path[i];
				const auto &next = path[i + 1];

				auto it = transforms.find(current);
				if (it == transforms.end())
				{
					valid_path = false;
					break;
				}

				auto transform_it = it->second.find(next);
				if (transform_it == it->second.end())
				{
					// Check if inverse transform exists
					auto inverse_it = transforms.find(next);
					if (inverse_it == transforms.end() || inverse_it->second.find(current) == inverse_it->second.end())
					{
						valid_path = false;
						break;
					}
					transform_it = inverse_it->second.find(current);
					current_transform *= transform_it->second->transform.inverse();
				}
				else
				{
					current_transform *= transform_it->second->transform;
				}
			}

			if (!valid_path)
				continue;

			if (!result)
			{
				result = current_transform;
			}
			else if ((result->matrix() - current_transform.matrix()).norm() > epsilon)
			{
				std::cout << "Inconsistent transforms detected between " << from << " and " << to << std::endl;
				return std::nullopt;
			}
		}

		return result;
	}

	void print_transform_chain(const std::string &from, const std::string &to) const
	{
		auto path = find_path(from, to);
		if (!path)
		{
			std::cout << "No valid path found from " << from << " to " << to << std::endl;
			return;
		}

		std::cout << "Transform chain from " << from << " to " << to << ":" << std::endl;
		for (size_t i = 0; i < path->size() - 1; ++i)
		{
			const auto &current = (*path)[i];
			const auto &next = (*path)[i + 1];
			std::cout << current << " -> " << next << std::endl;

			auto transform = get_transform(current, next);
			if (transform)
			{
				std::cout << *transform << std::endl << std::endl;
			}
			else
			{
				std::cout << "Error: Missing transform" << std::endl << std::endl;
			}
		}
	}

	template <typename PointType>
	std::shared_ptr<std::vector<PointType>> transform_point_cloud(const std::string &from, const std::string &to,
	                                                            const std::vector<PointType> &points) const
	{
		static_assert(std::is_same_v<typename PointType::Scalar, Scalar>, "Point type must have the same scalar type as the transform");
		static_assert(PointType::RowsAtCompileTime == Dim, "Point dimension must match the transform dimension");

		auto transform = get_transform(from, to);
		if (!transform)
			throw std::runtime_error("No valid transform found");

		auto transformed_points = std::make_shared<std::vector<PointType>>();
		transformed_points->reserve(points.size());

		auto rotation = transform->template block<Dim, Dim>(0, 0);
		auto translation = transform->template block<Dim, 1>(0, Dim);

		std::transform(points.begin(), points.end(), std::back_inserter(*transformed_points),
		               [&](const PointType &p)
		               { return rotation * p + translation; });

		return transformed_points;
	}

	void print_transform(const std::string &from, const std::string &to) const
	{
		auto transform = get_transform(from, to);
		if (transform)
		{
			std::cout << "Total transform from " << from << " to " << to << ":" << std::endl;
			std::cout << *transform << std::endl;
		}
		else
		{
			std::cout << "No valid transform found from " << from << " to " << to << std::endl;
		}
	}

	bool has_direct_transform(const std::string &from, const std::string &to) const
	{
		auto it = transforms.find(from);
		if (it != transforms.end())
		{
			return it->second.find(to) != it->second.end();
		}
		return false;
	}

	void remove_transform(const std::string &from, const std::string &to)
	{
		auto it_from = transforms.find(from);
		auto it_to = transforms.find(to);

		if (it_from != transforms.end())    it_from->second.erase(to);

		if (it_to != transforms.end()) it_to->second.erase(from);

	}

	[[nodiscard]] std::vector<std::string> get_all_coordinate_systems() const
	{
		std::vector<std::string> systems;
		for (const auto &[system, _] : transforms)
		{
			systems.push_back(system);
		}
		return systems;
	}

private:
	std::unordered_map<std::string, std::unordered_map<std::string, std::shared_ptr<TransformData>>> transforms;

	std::vector<std::vector<std::string>> find_all_paths(const std::string &start, const std::string &end) const
	{
		std::vector<std::vector<std::string>> all_paths;
		std::vector<std::string> current_path;
		std::unordered_set<std::string> visited;

		find_all_paths_dfs(start, end, visited, current_path, all_paths);

		return all_paths;
	}

	void find_all_paths_dfs(const std::string &current, const std::string &end,
	                     std::unordered_set<std::string> &visited,
	                     std::vector<std::string> &currentPath,
	                     std::vector<std::vector<std::string>> &allPaths) const
	{
		visited.insert(current);
		currentPath.push_back(current);

		if (current == end)
		{
			allPaths.push_back(currentPath);
		}
		else
		{
			auto it = transforms.find(current);
			if (it != transforms.end())
			{
				for (const auto &[next, _] : it->second)
				{
					if (visited.find(next) == visited.end())
					{
						find_all_paths_dfs(next, end, visited, currentPath, allPaths);
					}
				}
			}
		}

		visited.erase(current);
		currentPath.pop_back();
	}
	void check_consistency(const std::string &from, const std::string &to)
	{
		const Scalar epsilon = 1e-6; // Tolerance for floating-point comparisons

		for (const auto &[start, _] : transforms)
		{
			for (const auto &[end, __] : transforms)
			{
				if (start == from && end == to) {
					continue; // Skip the newly added transform
				}

				auto direct = get_transform(start, end);
				if (!direct) continue; // No path exists

				auto through_new = get_transform(start, from).value_or(MatrixType::Identity()) *
				                  get_transform(from, to).value_or(MatrixType::Identity()) *
				                  get_transform(to, end).value_or(MatrixType::Identity());

				if ((direct->matrix() - through_new.matrix()).norm() > epsilon)
				{
					std::cout << "Warning: Inconsistency detected in transforms!" << std::endl;
					std::cout << "Direct transform from " << start << " to " << end << ":" << std::endl;
					std::cout << *direct << std::endl;
					std::cout << "Transform through newly added path:" << std::endl;
					std::cout << through_new << std::endl;
				}
			}
		}
	}

	[[nodiscard]] std::shared_ptr<std::vector<std::string>> find_path(const std::string &start, const std::string &end) const
	{
		if (start == end) {
			return std::make_shared<std::vector<std::string>>(std::vector<std::string>{start});
		}

		std::unordered_map<std::string, std::string> came_from;
		std::queue<std::string> queue;
		queue.push(start);

		while (!queue.empty())
		{
			std::string current = queue.front();
			queue.pop();

			if (current == end)
			{
				auto path = std::make_shared<std::vector<std::string>>();
				for (auto at = end; at != start; at = came_from[at]) {
					path->push_back(at);
				}
				path->push_back(start);
				std::reverse(path->begin(), path->end());
				return path;
			}

			auto it = transforms.find(current);
			if (it == transforms.end()) {
				continue;
			}

			for (const auto &[next, _] : it->second)
			{
				if (came_from.find(next) == came_from.end())
				{
					queue.push(next);
					came_from[next] = current;
				}
			}
		}

		return nullptr;
	}
};
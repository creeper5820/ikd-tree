#pragma once

#include <eigen3/Eigen/Core>

#include <memory>
#include <vector>

namespace creeper {

template <typename Point>
concept point_concept = requires(Point point) {
    { point.x } -> std::convertible_to<float>;
    { point.y } -> std::convertible_to<float>;
    { point.z } -> std::convertible_to<float>;
};

enum class RemoveTime : uint8_t { NONE, REBUILD };

enum class Axis : uint8_t { X, Y, Z };

template <point_concept Point> class IkdTree {
public:
    using Points = std::vector<Point, Eigen::aligned_allocator<Point>>;

    struct Area {
        Point max;
        Point min;
    };

    IkdTree();
    ~IkdTree();

    IkdTree(const IkdTree&) = delete;
    IkdTree& operator=(const IkdTree&) = delete;

    IkdTree(IkdTree&&) noexcept;
    IkdTree& operator=(IkdTree&&) noexcept;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl;
};

} // namespace creeper

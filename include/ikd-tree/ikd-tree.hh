#pragma once

#include <eigen3/Eigen/Core>

#include <memory>
#include <optional>
#include <stack>
#include <unordered_set>
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

/// @brief 标签索引的对象池
/// @note ikd树在重建，插入过程中涉及到大量内存操作
///       借对象池可以减少申请的开销
template <typename T>
    requires requires { T {}; }
class NodePool {
public:
    explicit NodePool(std::uint32_t capacity = 1'000)
        : capacity_(capacity) {

        objects_.resize(capacity);
        for (auto index = 0; index < capacity; index++)
            unused_index_.push(index);
    }

    bool contains(std::uint32_t index) const { //
        return used_index_.contains(index);
    }

    std::optional<T&> get(std::uint32_t index) {
        if (!contains(index))
            return std::nullopt;
        return objects_[index];
    }

    T& unsafe_get(std::uint32_t index) { //
        return objects_[index];
    }

    const T& read(std::uint32_t index) { //
        return objects_[index];
    }

    std::uint32_t pull() {
        if (unused_index_.empty())
            expand();
        auto index = unused_index_.top();
        unused_index_.pop();
        used_index_.insert(index);
        return index;
    }

    void push(std::uint32_t index) {
        used_index_.erase(index);
        unused_index_.push(index);
    }

    std::uint32_t capacity() const { return capacity_; }

    std::uint32_t used_size() const { return used_index_.size(); }

    std::uint32_t unused_size() const { return unused_index_.size(); }

private:
    std::uint32_t capacity_ { 1'000 };

    std::stack<std::uint32_t> unused_index_;
    std::unordered_set<std::uint32_t> used_index_;

    std::vector<T> objects_;

private:
    void expand() {
        const auto capacity = capacity_ * 2;

        objects_.resize(capacity);
        for (auto i = capacity_; i < capacity; i++)
            unused_index_.push(i);

        capacity_ = capacity;
    }
};

} // namespace creeper

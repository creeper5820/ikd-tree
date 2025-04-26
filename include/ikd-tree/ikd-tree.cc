#include "ikd-tree.hh"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <optional>
#include <thread>

using namespace creeper;

template <point_concept Point> class IkdTree<Point>::Impl {

    struct Node {
        std::weak_ptr<Node> father = nullptr;
        std::shared_ptr<Node> left = nullptr;
        std::shared_ptr<Node> right = nullptr;

        Area area { Point { 0, 0, 0 }, Point { 0, 0, 0 } };
        Point point { 0, 0, 0 };
        Axis axis { Axis::X };
        std::size_t size { 0 };

        std::atomic<std::size_t> deprecated_count = 0;
        std::atomic<std::size_t> downsample_deprecated_count = 0;
        std::atomic<std::size_t> node_available_count = 0;

        bool deleted = false;
        bool downsample_deleted = false;

        bool tree_downsample_deleted = false;
        bool tree_deleted = false;

        bool request_push_down_left = false;
        bool request_push_down_right = false;

        bool working = false;
        std::mutex push_down_lock {};

#ifdef USE_RECORD
        double alpha_deleted = 0;
        double alpha_balance = 0;
#endif // USE_RECORD
    };

    // 删除Node的阈值，当需要删除的节点占比超过该阈值，则执行删除操作
    float delete_critical = 0.5;

    // 重建阈值，不平衡的节点超过此阈值，则执行重建操作
    float rebuild_critical = 0.7;

    // 降采样方盒的大小，单位为米
    float downsample_length = 0.2;

    std::jthread rebuild_thread;

    void config(float delete_critical, float rebuild_critical, float downsample_length) noexcept {
        this->delete_critical = delete_critical;
        this->rebuild_critical = rebuild_critical;
        this->downsample_length = downsample_length;
    }

    void initialize() { }

    void nearest_search(Point point, std::size_t find_size, float max_distance, Points& points,
        std::vector<float>& distances) const { }

    void insert_points(const Points& points, bool use_downsample) { }

    void recovery_points(const std::vector<std::pair<Point, Point>>& areas) { }

    void remove_points(const Points& points) { }

    void remove_points(const std::vector<std::pair<Point, Point>>& areas) { }

    void extract_removed_points(Points& points) { }

    std::size_t size() const { return {}; }

    std::size_t valid_count() const { return {}; }

    std::pair<Point, Point> area() const { return {}; }

private:
    static void _internal_flatten(const std::shared_ptr<Node>& source, Points& destination) { }

    static Area _internal_make_area() {
        auto max = Point {
            -std::numeric_limits<float>::infinity(),
            -std::numeric_limits<float>::infinity(),
            -std::numeric_limits<float>::infinity(),
        };
        auto min = Point {
            std::numeric_limits<float>::infinity(),
            std::numeric_limits<float>::infinity(),
            std::numeric_limits<float>::infinity(),
        };
        return { .max = max, .min = min };
    }

    static std::optional<std::shared_ptr<Node>> _internal_build(
        Points& points, std::size_t begin, std::size_t end) {

        if (begin > end)
            return std::nullopt;

        auto result = std::shared_ptr<Node>();

        // 寻找最佳划分轴
        auto area = _internal_make_area();
        auto& [max, min] = area;

        for (const auto point : points)
            min.x = std::min(min.x, point.x);

        const auto range = std::array {
            max.x - min.x,
            max.y - min.y,
            max.z - min.z,
        };
        const auto index
            = std::distance(range.begin(), std::max_element(range.begin(), range.end()));

        // 进行中间域划分，仅确保左侧节点大于右侧节点
        // 并应用最广域轴
        const auto middle = (begin + end) / 2;
        switch (index) {
        case 0:
            std::nth_element(points.begin() + begin, points.begin() + middle, points.begin() + end,
                [](const Point& point, const Point& other) { return point.x > other.x; });
            result->axis = Axis::X;
            break;
        case 1:
            std::nth_element(points.begin() + begin, points.begin() + middle, points.begin() + end,
                [](const Point& point, const Point& other) { return point.y > other.y; });
            result->axis = Axis::Y;
            break;
        default:
            std::nth_element(points.begin() + begin, points.begin() + middle, points.begin() + end,
                [](const Point& point, const Point& other) { return point.z > other.z; });
            result->axis = Axis::Z;
            break;
        }

        // 中间节点即为根节点
        // 开始递归二分构建
        result->point = points.at(middle);

        auto left = _internal_build(points, begin, middle - 1);
        if (left)
            result->left = std::move(left.value());

        auto right = _internal_build(points, middle + 1, end);
        if (right)
            result->right = std::move(right.value());

        return result;
    }

    void _internal_update_tree(const std::shared_ptr<Node>& root) {
        auto& l = root->left;
        auto& r = root->right;
        auto area = _internal_make_area();

        if (l == nullptr && r == nullptr) {
            root->size = 1;
            root->deprecated_count = root->deleted ? 1 : 0;
            root->downsample_deprecated_count = root->downsample_deleted ? 1 : 0;
            root->tree_deleted = root->deleted;
            root->tree_downsample_deleted = root->downsample_deleted;
            root->area = { root->point, root->point };
        }

        if (l)
            l->father = root;
        if (r)
            r->father = root;
    }
};

template <point_concept Point>
IkdTree<Point>::IkdTree()
    : pimpl(std::make_unique<Impl>()) { }

template <point_concept Point> IkdTree<Point>::~IkdTree() = default;

template <point_concept Point> IkdTree<Point>::IkdTree(IkdTree&&) noexcept = default;

template <point_concept Point>
IkdTree<Point>& IkdTree<Point>::operator=(IkdTree&&) noexcept = default;

template class creeper::IkdTree<pcl::PointXYZ>;
// template class creeper::IkdTree<pcl::PointXYZI>;
// template class creeper::IkdTree<pcl::PointXYZL>;
// template class creeper::IkdTree<pcl::PointXYZINormal>;

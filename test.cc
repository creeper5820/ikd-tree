#include "ikd-tree/ikd-tree.hh"

#include <chrono>
#include <iostream>
#include <pcl/point_types.h>

using namespace creeper;

auto main() -> int {
    std::cout << "Hello World!" << std::endl;

    static std::size_t count {};
    struct Node {
        int a { 0 };
        int b { 0 };
        int c[100] {};

        void print() const { count += a; }
    };

    Node* node;

    auto timestamp_a = std::chrono::high_resolution_clock::now();
    for (auto i = 0; i < 1'000'000; i++) {
        node = new Node;
        node->a = i;
        node->print();
    }

    auto timestamp_b = std::chrono::high_resolution_clock::now();
    auto pool = NodePool<Node> {};
    for (auto i = 0; i < 1'000'000; i++) {
        auto index = pool.pull();
        auto& node = pool.unsafe_get(index);
        node.a = i;
        node.print();
    }

    auto timestamp_c = std::chrono::high_resolution_clock::now();

    std::cout << std::chrono::duration<double>(timestamp_b - timestamp_a) << std::endl;
    std::cout << std::chrono::duration<double>(timestamp_c - timestamp_b) << std::endl;
    std::cout << "count: " << count << std::endl;
    std::cout << "capacity: " << pool.capacity() << std::endl;
    std::cout << "used: " << pool.used_size() << std::endl;

    return 0;
}

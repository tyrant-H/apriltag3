#pragma once

#include <cstdint>
#include <vector>

namespace apriltag3_cpp {

class UnionFind {
public:
    explicit UnionFind(std::size_t n = 0);

    void reset(std::size_t n);
    std::size_t find(std::size_t x);
    void unite(std::size_t a, std::size_t b);
    std::size_t size(std::size_t x);

private:
    std::vector<std::size_t> parent_;
    std::vector<std::size_t> rank_;
    std::vector<std::size_t> size_;
};

}  // namespace apriltag3_cpp

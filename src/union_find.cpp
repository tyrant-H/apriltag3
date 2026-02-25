#include "apriltag3_cpp/union_find.hpp"

namespace apriltag3_cpp {

UnionFind::UnionFind(std::size_t n) {
    reset(n);
}

void UnionFind::reset(std::size_t n) {
    parent_.resize(n);
    rank_.assign(n, 0);
    size_.assign(n, 1);
    for (std::size_t i = 0; i < n; ++i) {
        parent_[i] = i;
    }
}

std::size_t UnionFind::find(std::size_t x) {
    while (parent_[x] != x) {
        parent_[x] = parent_[parent_[x]];
        x = parent_[x];
    }
    return x;
}

void UnionFind::unite(std::size_t a, std::size_t b) {
    a = find(a);
    b = find(b);
    if (a == b) {
        return;
    }
    if (rank_[a] < rank_[b]) {
        auto t = a;
        a = b;
        b = t;
    }
    parent_[b] = a;
    size_[a] += size_[b];
    if (rank_[a] == rank_[b]) {
        ++rank_[a];
    }
}

std::size_t UnionFind::size(std::size_t x) {
    return size_[find(x)];
}

}  // namespace apriltag3_cpp

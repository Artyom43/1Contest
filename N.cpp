#include <iostream>
#include <vector>
#include <climits>

//====================================================DSU=============================================================//

class DSU {
public:
    const size_t NO_DELEGATE = UINT_MAX;
    const size_t NO_RANK = UINT_MAX;

    explicit DSU(size_t elements_count):
        delegate_(elements_count), rank_(elements_count), heap_size_(elements_count){
        for (size_t i = 0; i < elements_count; ++i) {
            makeSet(i);
        }
    }

    void makeSet(size_t x) {
        heap_size_[x] = 1;
        delegate_[x] = x;
        rank_[x] = 0;
    }

    size_t find(size_t x) {
        if(delegate_[x] == x) {
            return x;
        }
        delegate_[x] = find(delegate_[x]);
        return delegate_[x];
    }

    size_t unite(size_t x, size_t y) {
        x = find(x);
        y = find(y);
        if (rank_[x] < rank_[y]) {
            if (x != y) {
                heap_size_[y] += heap_size_[x];
            }
            delegate_[x] = y;
            return y;
        } else {
            if (x != y) {
                heap_size_[x] += heap_size_[y];
            }
            delegate_[y] = x;
            if (rank_[x] == rank_[y]) {
                ++rank_[x];
            }
            return x;
        }
    }

    size_t getHeapSize(size_t x) const {
        return heap_size_[x];
    }


private:
    std::vector<size_t> heap_size_;
    std::vector<size_t> delegate_;
    std::vector<size_t> rank_;
};


//====================================================main============================================================//


int main() {
    size_t vertex_count, edges_count;
    std::cin >> vertex_count >> edges_count;
    DSU dsu(vertex_count);
    size_t min_count = 0;
    bool is_min_reached = false;

    for (size_t i = 0; i < edges_count; ++i) {
        size_t from, to;
        std::cin >> from >> to;
        size_t heap_lead = dsu.unite(from, to);
        //std::cout << dsu.getHeapSize(heap_lead) << "\n";
        if (dsu.getHeapSize(heap_lead) == vertex_count && !is_min_reached) {
            is_min_reached = true;
            min_count = i;
        }
    }

    std::cout << min_count + 1;

    return 0;

}


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
        heap_size_[x] = 0;
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

    size_t unite(size_t x, size_t y, size_t weight) {
        x = find(x);
        y = find(y);
        if (x == y) {
            heap_size_[x] += weight;
            return x;
        }
        if (rank_[x] < rank_[y]) {
            heap_size_[y] += heap_size_[x] + weight;
            delegate_[x] = y;
            return y;
        } else {
            heap_size_[x] += heap_size_[y] + weight;
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
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(nullptr);
    size_t vertex_count, command_count;
    std::cin >> vertex_count >> command_count;
    DSU dsu(vertex_count);
    size_t command;

    do {
        std::cin >> command;
        if (command == 1) {
            size_t from, to, weight;
            std::cin >> from >> to >> weight;
            dsu.unite(--from, --to, weight);
        }

        if (command == 2) {
            size_t component_member;
            std::cin >> component_member;
            size_t lead = dsu.find(--component_member);
            std::cout << dsu.getHeapSize(lead) << "\n";
        }
        command_count--;

    } while (command_count > 0);

    return 0;
}
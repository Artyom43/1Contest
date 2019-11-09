#include<iostream>
#include<map>
#include<vector>
#include<queue>
#include<algorithm>


std::vector<size_t> getChildren(size_t number) {
    std::vector<size_t> ret;
    if (number % 10 != 1) {
        ret.push_back(number - 1);
    }
    if (number / 1000 != 9) {
        ret.push_back(number + 1000);
    }
    ret.push_back(number / 10 + (number % 10) * 1000);
    ret.push_back(10 * (number % 1000) + number / 1000);
    return ret;
}

void SmthLikeBfsWithPrevs(size_t from, size_t to, std::map<size_t, size_t>& prev) {
    std::queue<size_t> q;
    q.push(from);
    size_t  current_number = 0;
    while (current_number != to) {
        current_number = q.front();
        q.pop();
        for (size_t number : getChildren(current_number)) {
            if (prev.find(number) == prev.end()) {
                prev[number] = current_number;
                q.push(number);
            }
        }
    }
}

std::vector<size_t> findWay(size_t from, size_t to) {
    std::vector<size_t> ret(0);
    std::map<size_t, size_t> prev;
    SmthLikeBfsWithPrevs(from, to, prev);
    size_t current_number = to;
    while( current_number != from) {
        ret.push_back(current_number);
        current_number = prev[current_number];
    }
    ret.push_back(from);
    std::reverse(ret.begin(), ret.end());
    return ret;
}


int main() {
    size_t from, to;
    std::cin >> from >> to;

    std::vector<size_t> way = findWay(from, to);
    for (size_t number : way) {
        std::cout << number << "\n";
    }
    return 0;
}
#include<iostream>
#include<vector>
#include<queue>
#include<climits>

enum PositionColor {WHITE, GRAY, BLACK};
const size_t INF = UINT_MAX;

std::vector<size_t> getNeighbors(size_t position, size_t vertical_size, size_t horizontal_size) {
    std::vector<size_t> ret(0);
    std::vector<int> dx({1, 0, -1});
    std::vector<int> dy({1, 0, -1});
    size_t position_x = position % horizontal_size;
    size_t position_y = position / horizontal_size;
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {

            if (position_x + dx[i] >= 0 &&
                position_x + dx[i] < horizontal_size &&
                position_y + dy[j] >= 0 &&
                position_y + dy[j] < vertical_size &&
                dy[j] * dy[j] + dx[i] * dx[i] == 1) {
                ret.push_back(position_x + dx[i] + horizontal_size * ( position_y + dy[j]));
            }
        }
    }
    return ret;
}

size_t bfsReturnsDistanceToNearestOne(size_t position, const std::vector<std::vector<size_t>>& matrix,
                                    size_t vertical_size, size_t horizontal_size){
    std::vector<size_t> distances(vertical_size * horizontal_size, INF);
    std::vector<PositionColor> color(vertical_size * horizontal_size, WHITE);
    distances[position] = 0;
    std::queue<size_t> q;
    q.push(position);
    color[position] = GRAY;
    size_t current_position = position;
    size_t current_position_x = current_position % horizontal_size;
    size_t current_position_y = current_position / horizontal_size;
    while (matrix[current_position_y][current_position_x] != 1) {
        current_position = q.front();
        q.pop();
        current_position_x = current_position % horizontal_size;
        current_position_y = current_position / horizontal_size;
        for (size_t p : getNeighbors(current_position, vertical_size, horizontal_size)) {
            if (color[p] == WHITE) {
                distances[p] = distances[current_position] + 1;
                color[p] = GRAY;
                q.push(p);
            }
        }
        color[current_position] = BLACK;
    }
    return distances[current_position];
}

std::vector<std::vector<size_t>> DistancesToNearestOneMatrix(size_t vertical_size, size_t horizontal_size,
                                const std::vector<std::vector<size_t>>& matrix ) {
    std::vector<std::vector<size_t>> ret(vertical_size, std::vector<size_t>(horizontal_size));
    for (size_t position = 0; position < vertical_size * horizontal_size; ++position) {
        size_t position_x = position % horizontal_size;
        size_t position_y = position / horizontal_size;
        ret[position_y][position_x] = bfsReturnsDistanceToNearestOne(position, matrix, vertical_size, horizontal_size);
    }
    return ret;

}


int main() {
    size_t vertical_size, horizontal_size;
    std::cin >> vertical_size >> horizontal_size;

    std::vector<std::vector<size_t>> in_matrix(vertical_size, std::vector<size_t>(horizontal_size));

    for(size_t i = 0; i < vertical_size; ++i) {
        for (size_t j = 0; j < horizontal_size; ++j) {
            std::cin >> in_matrix[i][j];
        }
    }

    std::vector<std::vector<size_t>> distances_matrix = DistancesToNearestOneMatrix(vertical_size, horizontal_size,
                                                                                    in_matrix);
    for (size_t i = 0; i < vertical_size; ++i) {
        for (size_t j = 0; j < horizontal_size; ++j) {
            std::cout << distances_matrix[i][j] << " ";
        }
        std::cout << "\n";
    }
    return 0;
}


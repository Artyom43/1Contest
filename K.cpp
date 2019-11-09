#include <iostream>
#include <vector>
#include <queue>
#include <stdexcept>
#include <climits>


//====================================================Graph===========================================================//


class Graph {
public:
    typedef size_t  Vertex;

    explicit Graph(size_t vertex_count, bool is_directed = false):
            vertex_count_(vertex_count), edge_count_(0), is_directed_(is_directed){}

    size_t getVertexCount() const {
        return vertex_count_;
    }
    size_t getEdgeCount() const {
        return edge_count_;
    }
    bool isDirected() const {
        return is_directed_;
    }
    virtual void addEdge (Vertex from, Vertex to) {
        ++edge_count_;
    }
    virtual std::vector<Vertex> getNeighbors(Vertex v) const = 0;

protected:
    size_t vertex_count_;
    size_t edge_count_;
    bool is_directed_;
};


//================================================AdjVertexGraph======================================================//


class AdjListGraph: public Graph {
public:
    const size_t IS_EDGE = 1;

    explicit AdjListGraph (size_t vertex_count, bool is_directed = false):
            Graph(vertex_count, is_directed), adj_list_(vertex_count_) {}

    AdjListGraph(size_t vertex_count, const std::vector<std::vector<Vertex>>& adj_matrix, bool is_directed = false ):
            Graph(vertex_count, is_directed){
        if (adj_matrix.size() != vertex_count) {
            throw std::runtime_error("Wrong input");
        }
        for (size_t i = 0; i < vertex_count; ++i) {
            if (adj_matrix[i].size() != vertex_count) {
                throw std::runtime_error("Wrong input");
            }
        }
        adj_list_ = std::vector<std::vector<Vertex>>(vertex_count);
        for (size_t i = 0; i < vertex_count; ++i) {
            for (size_t j = 0; j < vertex_count; ++j) {
                if (adj_matrix[i][j] == IS_EDGE) {
                    adj_list_[i].push_back(j);
                    ++edge_count_;
                }
            }
        }
    }

    std::vector<Vertex> getNeighbors(Vertex v) const override {
        if (v >= vertex_count_) {
            throw std::runtime_error("Invalid vertex");
        }
        return adj_list_[v];
    }

    void addEdge(Vertex from, Vertex to) override {
        if (from >= vertex_count_ || to >= vertex_count_) {
            throw std::runtime_error("Invalid vertex");
        }
        if (!is_directed_) {
            adj_list_[from].push_back(to);
            adj_list_[to].push_back(from);
        } else {
            adj_list_[from].push_back(to);
        }
    }

private:
    std::vector<std::vector<Vertex>> adj_list_;
};


//===============================================GraphProcessing======================================================//


namespace GraphProcessing {
    typedef size_t Vertex;
    enum VertexMark {WHITE, GRAY, BLACK};
    const size_t INF = UINT_MAX;

    std::vector<Vertex> getNeighbors(Vertex v, const std::vector<std::vector<char>>& game_field) {
        std::vector<Vertex> neighbors(0);
        size_t vertical_size = game_field.size();
        size_t horizontal_size = game_field[0].size();
        long long vertex_x = v % horizontal_size;
        long long vertex_y = v / horizontal_size;
        if (game_field[vertex_y][vertex_x] == '#') {
            return neighbors;
        }

        long long current_v_x = vertex_x;
        long long current_v_y = vertex_y;

        while (current_v_x < horizontal_size &&
            game_field[current_v_y][current_v_x] != '#') {
            ++current_v_x;
        }
        --current_v_x;
        if (current_v_x != vertex_x) {
            current_v_x = current_v_x - (current_v_x - vertex_x) / 2;
            neighbors.push_back(current_v_y * horizontal_size + current_v_x);
        }
        current_v_x = vertex_x;

        while (current_v_x >= 0 &&
            game_field[current_v_y][current_v_x] != '#') {
            --current_v_x;
        }
        ++current_v_x;
        if (current_v_x != vertex_x) {
            current_v_x = current_v_x + (vertex_x - current_v_x) / 2;
            neighbors.push_back(current_v_y * horizontal_size + current_v_x);
        }
        current_v_x = vertex_x;

        while (current_v_y < vertical_size &&
            game_field[current_v_y][current_v_x] != '#') {
            ++current_v_y;
        }
        --current_v_y;
        if (current_v_y != vertex_y) {
            current_v_y = current_v_y - (current_v_y - vertex_y) / 2;
            neighbors.push_back(current_v_y * horizontal_size + current_v_x);
        }
        current_v_y = vertex_y;

        while (current_v_y >= 0 &&
            game_field[current_v_y][current_v_x] != '#') {
            --current_v_y;
        }
        ++current_v_y;
        if (current_v_y != vertex_y) {
            current_v_y = current_v_y + (vertex_y - current_v_y) / 2;
            neighbors.push_back(current_v_y * horizontal_size + current_v_x);
        }

        return neighbors;
    }

    size_t shortestWaySize (Vertex from, Vertex to, const Graph& g) {
        std::vector<VertexMark> color(g.getVertexCount(), WHITE);
        std::vector<size_t> distance(g.getVertexCount(), INF);
        color[from] = GRAY;
        distance[from] = 0;
        Vertex current_v = from;
        std::queue<Vertex> q;
        q.push(from);
        while (!q.empty()) {
            current_v = q.front();
            q.pop();
            color[current_v] = GRAY;
            for (Vertex neighbor : g.getNeighbors(current_v)) {
                if (color[neighbor] == WHITE) {
                    color[neighbor] = GRAY;
                    distance[neighbor] = distance[current_v] + 1;
                    q.push(neighbor);
                }
            }
            color[current_v] = BLACK;
        }
        return distance[to];
    }

}


//====================================================main============================================================//


int main() {
    size_t vertical_size, horizontal_size;
    std::cin >> vertical_size >> horizontal_size;
    size_t from = 0, to = 0;

    std::vector<std::vector<char>> game_field(vertical_size, std::vector<char>(horizontal_size));
    for (size_t y = 0; y < vertical_size; ++y) {
        for (size_t x = 0; x < horizontal_size; ++x) {
            std::cin >> game_field[y][x];
            if (game_field[y][x] == 'S') {
                from = y * horizontal_size + x;
            }
            if (game_field[y][x] == 'T') {
                to = y * horizontal_size + x;
            }
        }
    }

    AdjListGraph graph(vertical_size * horizontal_size, true);
    for (size_t v = 0; v < graph.getVertexCount(); ++v) {
        std::vector<size_t> neighbors = GraphProcessing::getNeighbors(v, game_field);
        for (size_t neighbor : neighbors) {
            graph.addEdge(v, neighbor);
        }
    }

    size_t distance = GraphProcessing::shortestWaySize(from, to, graph);
    if (distance == UINT_MAX) {
        std::cout << -1;
    } else {
        std::cout << distance;
    }

    return 0;
}
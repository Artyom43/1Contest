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
    enum VertexMark {
        WHITE, GRAY, BLACK
    };
    const size_t NO_VERTEX = UINT_MAX;
    const size_t INF = UINT_MAX;

    std::vector<Vertex> getHorseNeighbors(Vertex v, size_t table_size) {
        std::vector<Vertex> neighbors(0);
        std::vector<int> dx = {-2, -1, 1, 2};
        std::vector<int> dy = {-1, -2, 2, 1};
        size_t v_x = v / table_size;
        size_t v_y = v % table_size;
        for (size_t i = 0; i < dx.size(); ++i) {
            if (v_x + dx[i] >= 0 &&
                v_x + dx[i] < table_size &&
                v_y + dy[i] >= 0 &&
                v_y + dy[i] < table_size) {
                Vertex neighbor = (v_x + dx[i]) * table_size + v_y + dy[i];
                neighbors.push_back(neighbor);
            }
            if (v_x + dx[i] >= 0 &&
                v_x + dx[i] < table_size &&
                v_y + dy[3 - i] >= 0 &&
                v_y + dy[3 - i] < table_size) {
                Vertex neighbor = (v_x + dx[i]) * table_size + v_y + dy[3 - i];
                neighbors.push_back(neighbor);
            }
        }
        return neighbors;
    }

    void bfsWithPrev(Vertex v, const Graph& g, std::vector<size_t>& distance, std::vector<Vertex>& prev) {
        std::vector<VertexMark> color(g.getVertexCount(), WHITE);
        color[v] = GRAY;
        distance[v] = 0;
        std::queue<Vertex> q;
        q.push(v);
        while(!q.empty()) {
            Vertex u = q.front();
            q.pop();
            for (Vertex w : g.getNeighbors(u)) {
                if (color[w] == WHITE) {
                    color[w] = GRAY;
                    prev[w] = u;
                    distance[w] = distance[u] + 1;
                    q.push(w);
                }
            }
            color[u] = BLACK;
        }
    }

    std::vector<Vertex> findShortestWay(Vertex from, Vertex to, const Graph& g) {
        std::vector<size_t> distance(g.getVertexCount(), INF);
        std::vector<Vertex> prev(g.getVertexCount(), NO_VERTEX);
        bfsWithPrev(from, g, distance, prev);
        std::vector<Vertex> inverted_way(0);
        if (distance[to] == INF) {
            return inverted_way;
        }
        inverted_way.push_back(to);
        while (to != from) {
            to = prev[to];
            inverted_way.push_back(to);
        }
        std::vector<Vertex> way(inverted_way.size());
        for (long long i = way.size() - 1; i >= 0; --i) {
            way[i] = inverted_way[way.size() - i - 1];
        }
        return way;
    }

}


//====================================================main============================================================//


int main() {
    size_t table_size;
    std::cin >> table_size;
    AdjListGraph graph(table_size * table_size, true);

    for (size_t v = 0; v < table_size * table_size; ++v) {
        std::vector<size_t> neighbor = GraphProcessing::getHorseNeighbors(v, table_size);
        for (size_t u : neighbor) {
            graph.addEdge(v, u);
        }
    }

    size_t x1, y1, x2, y2;
    std::cin >> x1 >> y1 >> x2 >> y2;
    size_t begin = (--x1) * table_size + (--y1);
    size_t end = (--x2) * table_size + (--y2);

    std::vector<size_t> way = GraphProcessing::findShortestWay(begin, end, graph);
    std::cout << way.size() - 1 << "\n";
    for (size_t v : way) {
        std::cout << (v / table_size) + 1  << " " << (v % table_size) + 1 << "\n";
    }
    return 0;
}

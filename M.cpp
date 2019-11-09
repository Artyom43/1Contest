#include <iostream>
#include <vector>
#include <map>
#include <stdexcept>
#include <climits>
#include <algorithm>


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
    const Vertex NO_VERTEX = UINT_MAX;
    const size_t INF = UINT_MAX;

    void dfsVisit(Vertex v, const Graph& g, std::vector<VertexMark>& color, std::vector<bool>& is_cutpoint,
            size_t timer ,std::vector<size_t>& tin, std::vector<size_t>& fup,
            Vertex prev = NO_VERTEX) {
        color[v] = GRAY;
        tin[v] = fup[v] = timer;
        ++timer;
        size_t children = 0;
        for (Vertex u : g.getNeighbors(v)) {
            if (prev == u) {
                continue;
            }
            if (color[u] == GRAY) {
                fup[v] = std::min(tin[u], fup[v]);
            }
            if (color[u] == WHITE){
                dfsVisit(u, g, color, is_cutpoint, timer, tin, fup, v);
                fup[v] = std::min(fup[v], fup[u]);
                if (fup[u] >= tin[v] && prev != NO_VERTEX) {
                    is_cutpoint[v] = true;
                }
                ++children;
            }
        }
        if (prev == NO_VERTEX && children > 1) {
            is_cutpoint[v] = true;
        }
    }

    std::vector<Vertex> findCutpoints (const Graph& g) {
        std::vector<Vertex> cutpoints(0);
        std::vector<VertexMark> color(g.getVertexCount(), WHITE);
        std::vector<bool> is_cutpoint(g.getVertexCount(), false);
        std::vector<size_t> tin(g.getVertexCount(), INF);
        std::vector<size_t> fup(g.getVertexCount(), INF);
        size_t timer = 0;
        for (Vertex v = 0; v < g.getVertexCount(); ++v) {
            if (color[v] == WHITE) {
                dfsVisit(v, g, color, is_cutpoint, timer, tin, fup);
            }
        }
        for (size_t i = 0; i < g.getVertexCount(); ++i) {
            if (is_cutpoint[i]) {
                cutpoints.push_back(i);
            }
        }
        return cutpoints;
    }

}


//=====================================================main===========================================================//


int main() {
    size_t vertex_count, edges_count;
    std::cin >> vertex_count >> edges_count;
    AdjListGraph graph(vertex_count);

    for (size_t i = 0; i < edges_count; ++i) {
        size_t from, to;
        std::cin >> from >> to;
        graph.addEdge(--from, --to);
    }

    std::vector<size_t> cutpoints = GraphProcessing::findCutpoints(graph);
    std::cout << cutpoints.size() << "\n";
    for (size_t cutpoint : cutpoints) {
        std::cout << cutpoint + 1 << "\n";
    }

    return 0;
}


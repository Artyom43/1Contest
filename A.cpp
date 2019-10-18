#include <iostream>
#include <vector>
#include <stdexcept>


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

    explicit AdjListGraph (size_t vertex_count):
        Graph(vertex_count), adj_list_(vertex_count_) {}

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
    enum VertexMark {WHITE, BLACK, GRAY};
    const int NO_WAY = -1;

    void dfsVisit(const Graph& g, Vertex v, std::vector<VertexMark>& color,
            std::vector<Vertex>& result) {
        color[v] = GRAY;
        for (Vertex u : g.getNeighbors(v)) {
            if (color[u] == WHITE) {
                dfsVisit(g, u, color, result);
            }
        }
        result.push_back(v);
        color[v] = BLACK;
    }

    std::vector<std::vector<Vertex>> getAdjComponents(const Graph& g) {
        std::vector<std::vector<Vertex>> result;
        std::vector<VertexMark> color(g.getVertexCount(), WHITE);

        std::vector<bool> used(g.getVertexCount(), false);
        for (Vertex v = 0; v < g.getVertexCount(); ++v) {
            if (color[v] == WHITE) {
                std::vector<Vertex> component(0);
                dfsVisit(g, v, color, component);
                result.push_back(component);
            }
        }

        return result;
    }

}


//====================================================main============================================================//


int main() {
    size_t  vertex_number, edge_number;
    std::cin >> vertex_number >> edge_number;

    AdjListGraph graph(vertex_number);
    for(size_t i = 0; i < edge_number; ++i) {
        size_t from, to;
        std::cin >> from >> to;
        graph.addEdge(--from, --to);
    }

    std::vector<std::vector<size_t>> components = GraphProcessing::getAdjComponents(graph);
    std::cout << components.size() << "\n";

    for (const auto& component : components) {
        std::cout << component.size() << "\n";
        for (auto vertex : component) {
            std::cout << ++vertex << " ";
        }
        std::cout << "\n";
    }

    return 0;
}
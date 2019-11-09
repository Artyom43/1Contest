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

    void dfsVisitWithOutOrder(Vertex v, const Graph& g, std::vector<VertexMark>& color, std::vector<size_t>& order) {
        color[v] = GRAY;
        for (Vertex u : g.getNeighbors(v)) {
            if (color[u] == WHITE) {
                dfsVisitWithOutOrder(u, g, color, order);
            }
        }
        color[v] = BLACK;
        order.push_back(v);
    }

    void dfsVisitInvertedGraph(Vertex v, const Graph& g, size_t& component_number, std::vector<int>& component) {
        component[v] = component_number;
        for (Vertex u : g.getNeighbors(v)) {
            if (component[u] == -1) {
                dfsVisitInvertedGraph(u, g, component_number, component);
            }
        }
    }

    std::pair<size_t, std::vector<Vertex>>condensation(const Graph& g, const Graph& inverted_g) {
        std::vector<VertexMark> color(g.getVertexCount(), WHITE);
        std::vector<size_t> order(0);
        for (Vertex v = 0; v < g.getVertexCount(); ++v) {
            if (color[v] == WHITE) {
                dfsVisitWithOutOrder(v, g, color, order);
            }
        }
        std::vector<int> component(g.getVertexCount(), -1);
        size_t component_size = 1;
        for (long long i = order.size() - 1; i >= 0; --i) {
            Vertex v = order[i];
            if (component[v] == -1) {
                dfsVisitInvertedGraph(v, inverted_g, component_size, component);
                ++component_size;
            }
        }
        std::vector<Vertex> ret(0);
        for (Vertex v = 0; v < g.getVertexCount(); ++v) {
            ret.push_back(component[v]);
        }
        return std::make_pair(--component_size, ret);
    }


}


//====================================================main============================================================//


int main() {
    size_t vertex_number, edge_number;
    std::cin >> vertex_number >> edge_number;

    AdjListGraph graph(vertex_number, true), inverted_graph(vertex_number, true);

    for (size_t i = 0; i < edge_number; ++i) {
        size_t from, to;
        std::cin >> from  >> to;
        graph.addEdge(--from, --to);
        inverted_graph.addEdge(to, from);
    }

    std::pair<size_t, std::vector<size_t>> ret  = GraphProcessing::condensation(graph, inverted_graph);
    size_t component_count = ret.first;
    std::vector<size_t> condensation = ret.second;
    std::cout << component_count << "\n";
    for (size_t i = 0; i < condensation.size(); ++i) {
        std::cout << condensation[i] << " ";
    }


    return 0;
}

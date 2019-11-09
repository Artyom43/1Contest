#include <iostream>
#include <vector>
#include <stdexcept>
#include <climits>
#include <algorithm>


//====================================================Graph===========================================================//


class Edge {
public:
    typedef size_t Vertex;
    const Vertex NO_VERTEX = UINT_MAX;

    Edge(const Edge& edge): weight_(edge.weight_), from_(edge.from_), to_(edge.to_) {}
    Edge(): from_(NO_VERTEX), to_(NO_VERTEX), weight_(0) {}
    Edge(Vertex from, Vertex to, size_t weight): from_(from), to_(to), weight_(weight) {}

    Vertex getTo() const {
        return to_;
    }
    Vertex  getFrom () const{
        return from_;
    }
    size_t getWeight() const {
        return weight_;
    }

    Edge& operator=(const Edge& edge) {
        this->weight_ = edge.weight_;
        this->to_ = edge.to_;
        this->from_ = edge.from_;
        return *this;
    };

private:
    Vertex to_;
    Vertex from_;
    size_t weight_;
};
bool operator<(const Edge lhs, const Edge rhs) {
    return lhs.getWeight() < rhs.getWeight();
}
class Cmp {
public:
    bool operator()(const Edge lhs, const Edge rhs) {
        return lhs < rhs;
    }
};

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
    virtual void addEdge (Vertex from, Vertex to, size_t weight) {
        ++edge_count_;
    }
    virtual std::vector<Edge> getNeighbors(Vertex v) const = 0;

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


    std::vector<Edge> getNeighbors(Vertex v) const override {
        if (v >= vertex_count_) {
            throw std::runtime_error("Invalid vertex");
        }
        return adj_list_[v];
    }

    void addEdge(Vertex from, Vertex to, size_t weight) override {
        if (from >= vertex_count_ || to >= vertex_count_) {
            throw std::runtime_error("Invalid vertex");
        }
        if (!is_directed_) {
            adj_list_[from].push_back({from, to, weight});
            adj_list_[to].push_back({to, from, weight});
        } else {
            adj_list_[from].push_back({from, to,  weight});
        }
    }

private:
    std::vector<std::vector<Edge>> adj_list_;
};


//===============================================GraphProcessing======================================================//


namespace GraphProcessing {
    typedef size_t Vertex;
    const Vertex NO_VERTEX = UINT_MAX;
    const size_t INF = 30000;

    void initSimpleSource(Vertex s, const Graph& g, std::vector<size_t>& distance, std::vector<Vertex>& previous) {
        for (Vertex v = 0; v < g.getVertexCount(); ++v) {
            distance[v] = INF;
            previous[v] = NO_VERTEX;
        }
        distance[s] = 0;
    }

    void Relax(Vertex u, Vertex v, size_t weight,
            std::vector<size_t>& distance, std::vector<Vertex>& previous) {
        if (distance[v] > distance[u] + weight) {
            if (distance[u] == INF) {
                return;
            }
            distance[v] = distance[u] + weight;
            previous[v] = u;
        }

    }

    std::vector<size_t> bellmanFord (Vertex s, const Graph& g, std::vector<Edge>& edges) {
        std::vector<size_t> distance(g.getVertexCount());
        std::vector<Vertex> previous(g.getVertexCount());
        initSimpleSource(s, g, distance, previous);
        for (size_t i = 0; i < g.getVertexCount() - 1; ++i) {
            for (const Edge& edge : edges) {
                Relax(edge.getFrom(), edge.getTo(), edge.getWeight(), distance, previous);
            }
        }
        return distance;
    }

}


//====================================================main============================================================//


int main() {
    size_t vertex_count, edges_count;
    std::cin >> vertex_count >> edges_count;
    AdjListGraph g(vertex_count, false);
    std::vector<Edge> edges;

    for (size_t i = 0; i < edges_count; ++i) {
        size_t from, to, weight;
        std::cin >> from >> to >> weight;
        g.addEdge(--from, --to, weight);
        edges.emplace_back(from, to, weight);
    }

    std::vector<size_t> distances = GraphProcessing::bellmanFord(0, g, edges);
    for (size_t distance : distances) {
        std::cout << distance << " ";
    }

    return 0;
}
#include <iostream>
#include <vector>
#include <stdexcept>
#include <climits>
#include <algorithm>


//====================================================DSU=============================================================//

class DSU {
public:
    const size_t NO_DELEGATE = UINT_MAX;
    const size_t NO_RANK = UINT_MAX;

    explicit DSU(size_t elements_count):
            delegate_(elements_count), rank_(elements_count) {
        for (size_t i = 0; i < elements_count; ++i) {
            makeSet(i);
        }
    }

    void makeSet(size_t x) {
        delegate_[x] = x;
        rank_[x] = 0;
    }

    size_t find(size_t x) {
        if(delegate_[x] == x) {
            return x;
        }
        return delegate_[x] = find(delegate_[x]);
    }

    size_t unite(size_t x, size_t y) {
        x = find(x);
        y = find(y);
        if (x == y) {
            return x;
        }
        if (rank_[x] < rank_[y]) {
            delegate_[x] = y;
            return y;
        } else {
            delegate_[y] = x;
            if (rank_[x] == rank_[y]) {
                ++rank_[x];
            }
            return x;
        }
    }


private:
    std::vector<size_t> delegate_;
    std::vector<size_t> rank_;
};


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
    enum VertexMark {WHITE, GRAY, BLACK};
    const Vertex NO_VERTEX = UINT_MAX;
    const size_t INF = UINT_MAX;


    size_t KruskalReturnsWeight(const Graph& g, const std::vector<Edge>& edges, DSU vertices) {
        size_t total_weight = 0;
        for(size_t i = 0; i < edges.size(); ++i) {
            Vertex from = edges[i].getFrom();
            Vertex to = edges[i].getTo();
            size_t weight = edges[i].getWeight();
            if (vertices.find(from) != vertices.find(to)) {
                total_weight += weight;
                vertices.unite(from, to);
            }
        }
        return total_weight;

    }

}


//====================================================main============================================================//


int main() {
    size_t vertex_count, edges_count;
    std::cin >> vertex_count >> edges_count;
    std::vector<Edge> edges(0);
    AdjListGraph g(vertex_count, false);
    DSU vertices(vertex_count);

    for (size_t i = 0; i < edges_count; ++i) {
        size_t from, to, weight;
        std::cin >> from >> to >> weight;
        g.addEdge(--from, --to, weight);
        edges.push_back({from, to, weight});
    }

    std::sort(edges.begin(), edges.end(), Cmp());

    size_t weight_of_min_spanning_tree = GraphProcessing::KruskalReturnsWeight(g, edges, vertices);
    std::cout << weight_of_min_spanning_tree;


    return 0;
}


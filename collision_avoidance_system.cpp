#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <array>

struct Node {
    std::string id;
    std::array<double, 3> pos;
    bool is_hazard = false;
    std::string hazard_reason;

    Node(const std::string& id_, double x, double y, double z)
        : id(id_), pos{x, y, z} {}
};

struct Edge {
    std::string to;
    double weight;
    Edge(const std::string& t, double w) : to(t), weight(w) {}
};

class OrbitalGraph {
public:
    void add_node(const Node& node) {
        nodes[node.id] = node;
        adjacency[node.id] = std::vector<Edge>{};
    }

    void add_edge(const std::string& a, const std::string& b, double w) {
        adjacency[a].emplace_back(b, w);
        adjacency[b].emplace_back(a, w);
    }

    void mark_hazard(const std::string& id, const std::string& reason) {
        if (nodes.count(id) == 0) return;
        nodes[id].is_hazard = true;
        nodes[id].hazard_reason = reason;
    }

    void print_nodes() const {
        for (const auto& [id, n] : nodes) {
            std::cout << "Node " << id << " pos=("
                      << n.pos[0] << ", " << n.pos[1] << ", " << n.pos[2] << ") "
                      << "hazard=" << (n.is_hazard ? "YES" : "NO");
            if (n.is_hazard) std::cout << " reason=" << n.hazard_reason;
            std::cout << "\n";
        }
    }

private:
    std::map<std::string, Node> nodes;
    std::map<std::string, std::vector<Edge>> adjacency;
};

int main() {

    return 0;
}

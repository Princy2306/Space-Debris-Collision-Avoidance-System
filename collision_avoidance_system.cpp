#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <array>
#include<cmath>
using namespace std;

struct Node {
    string id;
    array<double, 3> pos;
    bool is_hazard = false;
    string hazard_reason;
    
    Node()=default;
    Node(const string& id1, double x, double y, double z)
        : id(id1), pos{x, y, z} {}
};

struct Edge {
    string to;
    double weight;
    Edge(const string& t, double w) : to(t), weight(w) {}
};

class OrbitalGraph {
public:
    void add_node(const Node& node) {
        nodes[node.id] = node;
        adjacency[node.id] = vector<Edge>{};
    }

    void add_edge(const string& a, const string& b, double w) {
        adjacency[a].emplace_back(b, w);
        adjacency[b].emplace_back(a, w);
    }

    void mark_hazard(const string& node_id, const string& reason = "Predicted Debris Intersection") {
        // Marks a specific waypoint as a hazard to be avoided.
        if (has_node(node_id)) {
            nodes.at(node_id).is_hazard = true;
            nodes.at(node_id).hazard_reason = reason;
            cout << "--- COLLISION PREDICTED: Node " << node_id << " marked as HAZARD. Reason: " << reason << " ---" << endl;
        } else {
            cerr << "Error: Cannot mark hazard. Node " << node_id << " does not exist." << endl;
        }
    }

    void print_nodes() const {
        for (const auto& [id, n] : nodes) {
            cout << "Node " << id << " pos=("
                      << n.pos[0] << ", " << n.pos[1] << ", " << n.pos[2] << ") "
                      << "hazard=" << (n.is_hazard ? "YES" : "NO");
            if (n.is_hazard) cout << " reason=" << n.hazard_reason;
            cout << "\n";
        }
    }

private:
    map<string, Node> nodes;
    map<string, vector<Edge>> adjacency;
};


// claculate 3D Eulidean distance between two nodes
double calculate_heuristic (Node &current, Node & goal){
    double distance;
     double dx=goal.pos[0]-current.pos[0];
     double dy=goal.pos[1]-current.pos[1];
     double dz=goal.pos[2]-current.pos[2];
    distance=sqrt(dx*dx+dy*dy+dz*dz); 
    return distance;
}

int main() {
    string id1,id2;
double x1,y1,z1;
double x2,y2,z2;
cout<< "Information of Node 1"<< endl;
cin>> id1>>x1>>y1>>z1;

cout<< "Information of Node 2"<< endl;

cin>> id2>> x2>> y2>> z2;
Node source(id1,x1,y1,z1);
Node goal(id2,x2,y2,z2);

double heuristic_distance=calculate_heuristic (source,goal);

cout<< "Heuristic from " <<id1<<"to " <<id2<<"is "<< heuristic_distance;

    return 0;
}

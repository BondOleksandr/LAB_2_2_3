#include <vector>
#include <queue>
#include <iostream>
#include <chrono>

using namespace std;

class Graph_Node{
private:
    int designation = 0;
    int BFS_distance = 100000000;//as inf
    int Bellman_Ford_distance = 100000000;//as inf

public:
    /////////geter-setter////////////
    Graph_Node() = default;
    Graph_Node(int des): designation(des) {}

    int getDesignation() const { return designation; }
    int getBellmanFordDistance() const { return Bellman_Ford_distance; }
    int getBFSDistance() const { return BFS_distance; }

    void setDesignation(int des) { designation = des; }
    void setBellmanFordDistance(int dist) { Bellman_Ford_distance = dist; }
    void setBFSDistance(int dist) { BFS_distance = dist; }
    /////////geter-setter////////////
};

class Graph_Rib{
private:
    Graph_Node beg;
    Graph_Node end;
    string type = "tree";
    int weight = 0;

public:
    /////////geter-seter////////////
    Graph_Rib() = default;
    Graph_Rib(const Graph_Node& from, const Graph_Node& to, int w = 0) : beg(from), end(to), weight(w) {}

    Graph_Node getBegin() const { return beg; }
    Graph_Node getEnd() const { return end; }
    int getWeight() const { return weight; }

    void setBegin(const Graph_Node& from) { beg = from; }
    void setEnd(const Graph_Node& to) { end = to; }
    void setWeight(int w) { weight = w; }
    /////////geter-seter////////////
};

class Graph{
private:
    vector<Graph_Rib> ribs;
    Graph_Node* nodes;
    int size = 1;

public:
    Graph() = default;
    Graph(const int& in_size, const vector<Graph_Rib>& in_ribs): size(in_size), ribs(in_ribs){
        nodes = new Graph_Node[in_size];
        for(int i=0;i<in_size;i++){
            nodes[i] = Graph_Node(i);
        }
    }

    ~Graph() {
        delete[] nodes;
    }
};

int main()
{
}

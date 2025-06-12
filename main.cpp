#include <vector>
#include <queue>
#include <iostream>
#include <chrono>

using namespace std;

class Graph_Rib{
private:
    int beg;
    int end;
    int weight = 0;

public:
    /////////geter-seter////////////
    Graph_Rib() = default;
    Graph_Rib(const int& from, const int& to, int w = 0) : beg(from), end(to), weight(w) {}

    int getBegin() const { return beg; }
    int getEnd() const { return end; }
    int getWeight() const { return weight; }

    void setBegin(const int& from) { beg = from; }
    void setEnd(const int& to) { end = to; }
    void setWeight(int w) { weight = w; }
    /////////geter-seter////////////
};

class Graph{
private:
    vector<Graph_Rib> ribs;
    int size = 1;
    int** distance;
    bool** adjacency;
    int* bfs_distance;

public:
    Graph() = default;
    Graph(const int& in_size, const vector<Graph_Rib>& in_ribs): size(in_size), ribs(in_ribs){
        adjacency = new bool*[in_size];
        distance = new int*[in_size];
        bfs_distance = new int[in_size];

        for(int i=0;i<in_size;i++){
            adjacency[i] = new bool[in_size];
            distance[i] = new int[in_size];
            bfs_distance = 10000;//inf
            for(int j=0;j<in_size;j++){
                distance[i][j] = 1000000;//inf
            }
        }
        
        for(int i=0;i<in_ribs.size();i++){
            adjacency[in_ribs[i].getBegin()][in_ribs[i].getEnd()] = 1;
        }
    }

    ~Graph() {
    }

    
};

int main()
{
}

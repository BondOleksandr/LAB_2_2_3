#define CATCH_CONFIG_MAIN
#include "catch_amalgamated.hpp"
#include "main.cpp" 

const int INF = 1e8;
const double EPS = 0.001;

TEST_CASE("BFS finds correct shortest distances", "[Graph][BFS]") {
    vector<Graph_Rib> ribs = {
        Graph_Rib(0, 1), Graph_Rib(0, 2),
        Graph_Rib(1, 3), Graph_Rib(2, 3),
        Graph_Rib(3, 4)
    };

    Graph G(5, ribs);
    int* distances = G.bfs(0);

    REQUIRE(distances[0] == 0);   
    REQUIRE(distances[1] == 1);   
    REQUIRE(distances[2] == 1);   
    REQUIRE(distances[3] == 2);   
    REQUIRE(distances[4] == 3);   
}

TEST_CASE("BFS unreachable nodes stay INF", "[Graph][BFS]") {
    vector<Graph_Rib> ribs = {
        Graph_Rib(0, 1), Graph_Rib(1, 2)
       
    };

    Graph G(4, ribs);
    int* distances = G.bfs(0);
    REQUIRE(distances[3] == INF);
}

TEST_CASE("BFS_Parallel finds correct shortest distances", "[Graph][BFS_Parallel]") {
    vector<Graph_Rib> ribs = {
        Graph_Rib(0, 1), Graph_Rib(0, 2),
        Graph_Rib(1, 3), Graph_Rib(2, 3),
        Graph_Rib(3, 4)
    };

    Graph G(5, ribs);
    int* distances = G.bfs_parallel(0);

    REQUIRE(distances[0] == 0);   
    REQUIRE(distances[1] == 1);   
    REQUIRE(distances[2] == 1);   
    REQUIRE(distances[3] == 2);   
    REQUIRE(distances[4] == 3);   
}

TEST_CASE("BFS_Parallel unreachable nodes stay INF", "[Graph][BFS_Parallel]") {
    vector<Graph_Rib> ribs = {
        Graph_Rib(0, 1), Graph_Rib(1, 2)
       
    };

    Graph G(4, ribs);
    int* distances = G.bfs_parallel(0);

    REQUIRE(distances[0] == 0);
    REQUIRE(distances[1] == 1);
    REQUIRE(distances[2] == 2);
    REQUIRE(distances[3] == INF);
}

TEST_CASE("Floyd-Warshall produces correct distances", "[Graph][FloydWarshall]") {
    vector<Graph_Rib> ribs = {
        Graph_Rib(0, 1, 2),
        Graph_Rib(1, 2, 3),
        Graph_Rib(0, 2, 10),
        Graph_Rib(2, 3, 1)
    };

    Graph G(4, ribs);
    int** result = G.floyd_warshall();

    REQUIRE(result[0][0] == 0);
    REQUIRE(result[0][1] == 2);
    REQUIRE(result[0][2] == 5);  
    REQUIRE(result[0][3] == 6);  
    REQUIRE(result[3][0] == INF); 
}

TEST_CASE("ThreadPool Floyd-Warshall gives same result as sequential", "[Graph][ThreadPoolFloyd]") {
    vector<Graph_Rib> ribs = {
        Graph_Rib(0, 1, 2),
        Graph_Rib(1, 2, 3),
        Graph_Rib(0, 2, 10),
        Graph_Rib(2, 3, 1)
    };

    Graph G1(4, ribs);
    Graph G2(4, ribs);

    int** seq_result = G1.floyd_warshall();
    int** par_result = G2.floyd_warshall_thread_pool();

    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            REQUIRE(seq_result[i][j] == par_result[i][j]);
}

TEST_CASE("Bellman-Ford computes correct shortest distances", "[Graph][BellmanFord]") {
    vector<Graph_Rib> ribs = {
        Graph_Rib(0, 1, 4),
        Graph_Rib(0, 2, 5),
        Graph_Rib(1, 2, -2),
        Graph_Rib(2, 3, 3)
    };

    Graph G(4, ribs);
    int* distances = G.bellman_ford(0);

    REQUIRE(distances[0] == 0);
    REQUIRE(distances[1] == 4);
    REQUIRE(distances[2] == 2);  // 0->1->2
    REQUIRE(distances[3] == 5);  // 0->1->2->3
}

TEST_CASE("Parallel Bellman-Ford gives same result as sequential", "[Graph][BellmanFordThread]") {
    vector<Graph_Rib> ribs = {
        Graph_Rib(0, 1, 4),
        Graph_Rib(0, 2, 5),
        Graph_Rib(1, 2, -2),
        Graph_Rib(2, 3, 3)
    };

    Graph G1(4, ribs);
    Graph G2(4, ribs);

    int* seq = G1.bellman_ford(0);
    int* par = G2.bellman_ford_thread_pool(0);

    for (int i = 0; i < 4; ++i)
        REQUIRE(seq[i] == par[i]);
}

TEST_CASE("PageRank and PageRank_thread_pool produce same result", "[PageRank]") {
    vector<Graph_Rib> ribs = {
        Graph_Rib(0, 1),
        Graph_Rib(1, 2),
        Graph_Rib(2, 0),
        Graph_Rib(2, 1)
    };

    Graph G1(3, ribs);
    Graph G2(3, ribs);

    double* R1 = G1.PageRank();
    double* R2 = G2.PageRank_thread_pool();

    for (int i = 0; i < 3; ++i) {
        REQUIRE(fabs(R1[i] - R2[i]) < EPS);
    }
}
#include <queue>
#include <iostream>
#include <chrono>
#include <mutex>
#include <thread>
#include <vector>
#include <functional>
#include <future>
#include <atomic>
#include <condition_variable>
#include <algorithm>
#include <cmath>

using namespace std;

const double eps = 0.001;
const int inf = 1e8;
const double PR_d = 0.85;

/**
 * @class ThreadPool
 * @brief A simple thread pool for parallel task execution.
 */
class ThreadPool {
    vector<thread> workers;
    queue<function<void()>> tasks;

    mutex queue_mutex;
    condition_variable condition;
    atomic<bool> stop;

public:
 /**
     * @brief Initializes the thread pool with the specified number of threads.
     * @param threads Number of threads to create.
     */
    ThreadPool(size_t threads) : stop(false) {
        for (size_t i = 0; i < threads; ++i) {
            workers.emplace_back([this]() {
                while (true) {
                    function<void()> task;

                    {
                        unique_lock<mutex> lock(this->queue_mutex);
                        this->condition.wait(lock, [this]() {
                            return this->stop || !this->tasks.empty();
                        });

                        if (this->stop && this->tasks.empty())
                            return;

                        task = std::move(this->tasks.front());
                        this->tasks.pop();
                    }

                    task();
                }
            });
        }
    }
    /**
     * @brief Adds a task to the queue.
     * @param task A function to execute in the thread pool.
     */
    void enqueue(function<void()> task) {
        {
            unique_lock<mutex> lock(queue_mutex);
            tasks.emplace(std::move(task));
        }
        condition.notify_one();
    }

    /**
     * @brief Waits until all tasks have been processed.
     */
    void wait_all() {
        while (true) {
            {
                unique_lock<mutex> lock(queue_mutex);
                if (tasks.empty()) break;
            }
            this_thread::sleep_for(chrono::nanoseconds(1));
        }
    }

    /**
     * @brief Shuts down the thread pool and joins all threads.
     */
    ~ThreadPool() {
        stop = true;
        condition.notify_all();
        for (thread &worker : workers)
            worker.join();
    }
};

/**
 * @class Graph_Rib
 * @brief Represents an edge (rib) in a directed graph.
 */
class Graph_Rib{
private:
    int beg;
    int end;
    int weight = 0;

public:
    Graph_Rib() = default;
    Graph_Rib(const int& from, const int& to, int w = 0) : beg(from), end(to), weight(w) {}

    int getBegin() const { return beg; }
    int getEnd() const { return end; }
    int getWeight() const { return weight; }

    void setBegin(const int& from) { beg = from; }
    void setEnd(const int& to) { end = to; }
    void setWeight(int w) { weight = w; }
    
};

/**
 * @class Graph
 * @brief Directed weighted graph with adjacency matrix, rib-list, several aux arrays for storing data.
 */
class Graph{
private:
    vector<Graph_Rib> ribs;
    int size = 1;
    int** distance;
    bool** adjacency;
    int* bfs_distance;
    int* bellman_ford_distance;
    double* rank;

public:
    Graph() = default;
    Graph(const int& in_size, const vector<Graph_Rib>& in_ribs): size(in_size), ribs(in_ribs){
        adjacency = new bool*[in_size]{0};
        distance = new int*[in_size];
        bfs_distance = new int[in_size];
        bellman_ford_distance = new int[in_size];
        rank = new double[in_size]{0.85};

        for(int i=0;i<in_size;i++){
            adjacency[i] = new bool[in_size];
            distance[i] = new int[in_size];
            bfs_distance[i] = inf;//inf
            bellman_ford_distance[i]=inf;
            rank[i]=1/in_size;
            for(int j=0;j<in_size;j++){
                distance[i][j] = inf;//inf
            }
        }
        
        for(int i=0;i<in_ribs.size();i++){
            adjacency[in_ribs[i].getBegin()][in_ribs[i].getEnd()] = 1;
        }
    }

    /**
     * @brief Generates a random directed graph.
     * @param node_ammount Number of nodes.
     * @param rib_chance Probability (0–100) of edge creation between any two nodes.
     * @return Randomly generated Graph object.
     */
    static Graph GEN(int node_ammount, int rib_chance){
        vector<Graph_Rib> gen_ribs;
        rib_chance = rib_chance % 100;

        for(int i=0; i< node_ammount; i++){
            for(int j=0; j<node_ammount; j++){
                if(i!=j && (rand()%100 >= 100-rib_chance)) gen_ribs.push_back(Graph_Rib(i, j, rand()%10));
            }
        }

        Graph generated = Graph(node_ammount, gen_ribs);

        return generated;
    }

/**
* @brief Performs Breadth-First Search from a given node.
* @param start_node Index of the start node.
* @return Pointer to array with shortest distances from start_node.
*/
int* bfs(int start_node) {
    for(int i=0;i<size;i++){
        bfs_distance[i]=inf;
    }

    using namespace std::chrono;
    auto begin_time = high_resolution_clock::now();

    queue<int> q;

    bfs_distance[start_node] = 0;
    q.push(start_node);
    
    while (!q.empty()) {
        int curr = q.front();
        q.pop();
        for (int i = 0; i < size; ++i) {
            if (adjacency[curr][i] && bfs_distance[i] == inf) {
                bfs_distance[i] = bfs_distance[curr] + 1;
                q.push(i);
            }
        }
    }

    auto end_time = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(end_time - begin_time).count();
    cout << "BFS computation time " << duration << " microseconds" << endl;
    return bfs_distance;
}


/**
* @brief Parallel version of BFS using std::thread.
* @param start_node Index of the starting node.
* @return Pointer to array with distances from start_node.
*/
int* bfs_parallel(int start_node) {
    for (int i = 0; i < size; ++i)
        bfs_distance[i] = inf;

    vector<atomic<bool>> visited(size);
    for (int i = 0; i < size; ++i) visited[i] = false;

    using namespace std::chrono;
    auto begin_time = high_resolution_clock::now();

    queue<int> q;
    bfs_distance[start_node] = 0;
    visited[start_node] = true;
    q.push(start_node);

    while (!q.empty()) {
        int level_size = q.size();
        vector<thread> threads;
        vector<int> next_level;

        mutex next_mutex;

        for (int t = 0; t < level_size; ++t) {
            int curr = q.front();
            q.pop();

            threads.emplace_back([&, curr]() {
                vector<int> local_next;

                for (int i = 0; i < size; ++i) {
                    if (adjacency[curr][i] && !visited[i].exchange(true)) {
                        bfs_distance[i] = bfs_distance[curr] + 1;
                        local_next.push_back(i);
                    }
                }

                lock_guard<mutex> lock(next_mutex);
                for (int v : local_next)
                    next_level.push_back(v);
            });
        }

        for (auto& t : threads) t.join();

        for (int v : next_level)
            q.push(v);
    }

    auto end_time = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(end_time - begin_time).count();
    cout << "\nBFS-threaded (safe) computation time: " << duration << " microseconds" << endl;
    return bfs_distance;
}



/**
* @brief Standard Floyd-Warshall algorithm for all-pairs shortest paths.
* @return Pointer to 2D array with shortest distances.
*/
int** floyd_warshall() {
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            if (i == j) distance[i][j] = 0;
            else distance[i][j] = inf;
        }
    }

    for (const auto& rib : ribs) {
        int u = rib.getBegin();
        int v = rib.getEnd();
        int w = rib.getWeight();
        distance[u][v] = w;
    }

    using namespace std::chrono;
    auto begin_time = high_resolution_clock::now();

    for (int k = 0; k < size; ++k) {
        for (int i = 0; i < size; ++i) {
            for (int j = 0; j < size; ++j) {
                if (distance[i][k] < inf && distance[k][j] < inf)
                    distance[i][j] = min(distance[i][j], distance[i][k] + distance[k][j]);
            }
        }
    }

    auto end_time = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(end_time - begin_time).count();
    cout << "\nFloyd–Warshall time: " << duration << " microseconds" << endl;
    return distance;
}

/**
* @brief Parallel Floyd-Warshall using ThreadPool.
* @return Pointer to 2D array with shortest distances.
*/
int** floyd_warshall_thread_pool() {
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            distance[i][j] = (i == j) ? 0 : inf;
        }
    }

    for (const auto& rib : ribs) {
        int u = rib.getBegin();
        int v = rib.getEnd();
        distance[u][v] = rib.getWeight();
    }

    using namespace std::chrono;
    auto begin_time = high_resolution_clock::now();

    ThreadPool Tpool(thread::hardware_concurrency());

    for (int k = 0; k < size; ++k) {
        for (int i = 0; i < size; ++i) {
            Tpool.enqueue([=]() {
                for (int j = 0; j < size; ++j) {
                    if (distance[i][k] < inf && distance[k][j] < inf) {
                        int new_dist = distance[i][k] + distance[k][j];
                        if (new_dist < distance[i][j]) {
                            distance[i][j] = new_dist;
                        }
                    }
                }
            });
        }


        Tpool.wait_all();
    }

    auto end_time = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(end_time - begin_time).count();

    cout << "\nThreadPool Floyd–Warshall time: " << duration << " microseconds" << endl;
    return distance;
}

/**
* @brief Standard Bellman-Ford algorithm for single-source shortest paths.
* @param start_node Index of the source node.
* @return Pointer to array with shortest distances.
*/
int* bellman_ford(int start_node) {
    for (int i = 0; i < size; ++i) bellman_ford_distance[i] = inf;
    bellman_ford_distance[start_node] = 0;
    using namespace std::chrono;
    auto begin_time = high_resolution_clock::now();

    for (int i = 0; i < size - 1; i++) {
        for (const auto& rib : ribs) {
            int beg = rib.getBegin();
            int end = rib.getEnd();
            int w = rib.getWeight();

            if ( bellman_ford_distance[beg] + w < bellman_ford_distance[end]) {
                bellman_ford_distance[end] = bellman_ford_distance[beg] + w;
            }
        }
    }

    auto end_time = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(end_time - begin_time).count();
    std::cout << "\nBellman-Ford time: " << duration << " microseconds" << std::endl;
    return bellman_ford_distance;
}

/**
* @brief Parallel Bellman-Ford using ThreadPool.
* @param start_node Index of the source node.
* @return Pointer to array with shortest distances.
*/
int* bellman_ford_thread_pool(int start_node) {
    for (int i = 0; i < size; ++i) bellman_ford_distance[i] = inf;

    bellman_ford_distance[start_node] = 0;

    using namespace std::chrono;
    auto begin_time = high_resolution_clock::now();

    ThreadPool Tpool(thread::hardware_concurrency());
    mutex update_mutex;

    for (int iter = 0; iter < size - 1; ++iter) {
        for (const auto& rib : ribs) {
            Tpool.enqueue([&, rib]() {
                int beg = rib.getBegin();
                int end = rib.getEnd();
                int w = rib.getWeight();

                int new_dist = bellman_ford_distance[beg] + w;

                if (new_dist < bellman_ford_distance[end]) {
                    lock_guard<mutex> lock(update_mutex);
                    if (new_dist < bellman_ford_distance[end]) {
                        bellman_ford_distance[end] = new_dist;
                    }
                }
            });
        }
        Tpool.wait_all();
    }

    auto end_time = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(end_time - begin_time).count();
    std::cout << "\nBellman-Ford(ThreadPool) time: " << duration << " microseconds" << std::endl;

    return bellman_ford_distance;
}

/**
* @brief Calculates PageRank values using iterative method.
* @return Pointer to array with rank values for each node.
*/
double* PageRank(){
int* L_arr = new int[size]{0};
double* new_rank = new double[size]{0};
for (int i=0;i<size;i++){
    rank[i] = 1.0/size;
    for(int j=0;j<size;j++){
        L_arr[i] += adjacency[i][j];
    }
}

int ribnumber = ribs.size();
bool surp=true;
Graph_Rib aux;

sort(ribs.begin(), ribs.end(), [](const Graph_Rib& a, const Graph_Rib& b) {
        return a.getEnd() < b.getEnd();
    });

double aux_summ;
double current_rank;
int current_node;
surp = 1;
int iter = 0;

using namespace std::chrono;
auto begin_time = high_resolution_clock::now();

while(surp){
    iter = 0;
    surp = 0;
    current_node = 0;

    while(iter<ribnumber){
        current_node = ribs[iter].getEnd();
        aux_summ = 0;

        while(iter<ribnumber&& current_node==ribs[iter].getEnd()){
            aux_summ += rank[ribs[iter].getBegin()]/L_arr[ribs[iter].getBegin()];
            iter++;
        }

        current_rank=((1-PR_d)/size)+PR_d*aux_summ;
        if(current_rank-rank[ribs[iter-1].getEnd()]>eps || current_rank-rank[ribs[iter-1].getEnd()]< eps*(-1) )surp = true;
        new_rank[ribs[iter-1].getEnd()] = current_rank;
    }

    for(int i=0;i<size;i++) rank[i]=new_rank[i];
}

auto end_time = high_resolution_clock::now();
auto duration = duration_cast<microseconds>(end_time - begin_time).count();
std::cout << "\nPageRank time: " << duration << " microseconds" << std::endl;

delete[] L_arr;
delete[] new_rank;

return rank;
}

/**
* @brief Calculates PageRank values using ThreadPool-based parallelism.
* @return Pointer to array with rank values for each node.
*/
double* PageRank_thread_pool() {
    int* L_arr = new int[size]{0};
    double* new_rank = new double[size]{0};
    mutex update_mutex;
    for (int i = 0; i < size; i++) {
        rank[i] = 1.0 / size;
        for (int j = 0; j < size; j++) {
            L_arr[i] += adjacency[i][j];
        }
    }

    sort(ribs.begin(), ribs.end(), [](const Graph_Rib& a, const Graph_Rib& b) {
        return a.getEnd() < b.getEnd();
    });

    using namespace std::chrono;
    auto begin_time = high_resolution_clock::now();

    ThreadPool Tpool(thread::hardware_concurrency());
    bool surp = true;

    while (surp) {
        surp = false;
        double* local_rank = new double[size]{0};

        atomic<int> active_tasks = 0;
        int iter = 0;

        while (iter < ribs.size()) {
            int target = ribs[iter].getEnd();
            int start_iter = iter;

            while (iter < ribs.size() && ribs[iter].getEnd() == target) {
                iter++;
            }

            active_tasks++;

            Tpool.enqueue([=, &L_arr, &update_mutex, &surp, &active_tasks, &local_rank]() mutable {
                double sum = 0;
                for (int k = start_iter; k < iter; ++k) {
                    int from = ribs[k].getBegin();
                    if (L_arr[from] != 0)
                        sum += rank[from] / L_arr[from];
                }

                double current_rank = ((1.0 - PR_d) / size) + PR_d * sum;

                {
                    if (fabs(current_rank - rank[target]) > eps) {
                        surp = true;
                    }
                    local_rank[target] = current_rank;
                }

                active_tasks--;
            });
        }

        Tpool.wait_all();

        for (int i = 0; i < size; i++)
            rank[i] = local_rank[i];

        delete[] local_rank;
    }


    delete[] L_arr;
    delete[] new_rank;

    auto end_time = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(end_time - begin_time).count();
    std::cout << "\nPageRank(threaded) time: " << duration << " microseconds" << std::endl;

    return rank;
}

    
};

/**
* @brief Main interface and state-control of the program.
*/
int main()
{
    Graph hraph;
    hraph = Graph::GEN(10, 30);
    hraph.bfs(0);
    hraph.bfs_parallel(0);
    hraph.floyd_warshall();
    hraph.floyd_warshall_thread_pool();
    hraph.bellman_ford(0);
    hraph.bellman_ford_thread_pool(0);
    hraph.PageRank();
    hraph.PageRank_thread_pool();
    
    while(1);

    return 0;
}

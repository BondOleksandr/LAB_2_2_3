#include <queue>
#include <iostream>
#include <chrono>
#include <mutex>
#include <thread>
#include <vector>
#include <functional>
#include <future>
#include <queue>
#include <atomic>
#include <condition_variable>

using namespace std;

const int inf = 1e8;

class ThreadPool {
    vector<thread> workers;
    queue<function<void()>> tasks;

    mutex queue_mutex;
    condition_variable condition;
    atomic<bool> stop;

public:
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

    void enqueue(function<void()> task) {
        {
            unique_lock<mutex> lock(queue_mutex);
            tasks.emplace(std::move(task));
        }
        condition.notify_one();
    }

    void wait_all() {
        while (true) {
            {
                unique_lock<mutex> lock(queue_mutex);
                if (tasks.empty()) break;
            }
            this_thread::sleep_for(chrono::milliseconds(1));
        }
    }

    ~ThreadPool() {
        stop = true;
        condition.notify_all();
        for (thread &worker : workers)
            worker.join();
    }
};

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
            bfs_distance[i] = inf;//inf
            for(int j=0;j<in_size;j++){
                distance[i][j] = inf;//inf
            }
        }
        
        for(int i=0;i<in_ribs.size();i++){
            adjacency[in_ribs[i].getBegin()][in_ribs[i].getEnd()] = 1;
        }
    }

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

    ~Graph() {
    }

    void bfs(int start_node) {///bfs realization - single thread
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
}


void bfs_parallel(int start_node) {
    for(int i = 0; i < size; i++) {
        bfs_distance[i] = inf;
    }

    mutex bfs_mutex;
    mutex add_mutex;

    using namespace std::chrono;
    auto begin_time = high_resolution_clock::now();

    queue<int> q;
    bfs_distance[start_node] = 0;
    q.push(start_node);


    while (!q.empty()) {
        int curr = q.front();
        q.pop();

        vector<thread> threads;
        vector<int> to_add;

        for (int i = 0; i < size; ++i) {
            if (adjacency[curr][i]) {
                threads.emplace_back([&, curr, i]() {
                    if (bfs_distance[i] == inf) {
                        //what happens in each thread//
                        std::lock_guard<std::mutex> lock(bfs_mutex);
                        if (bfs_distance[i] == inf) {
                            bfs_distance[i] = bfs_distance[curr] + 1;
                            std::lock_guard<std::mutex> lock2(add_mutex);
                            to_add.push_back(i);
                        }
                        //...........................//
                    }
                });
            }
        }

        for (auto& t : threads) t.join();

        for (int i : to_add) {
            q.push(i);
        }
    }

    auto end_time = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(end_time - begin_time).count();
    cout << "\nBFS-threaded computation time: " << duration << " microseconds" << endl;
}
////works badly, alot of threads, they are very short-lived. Protection from data race takes up lot's of resources, annihilating all the gain for multithreadedness.////


void floyd_warshall() {
    //cleanup
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            if (i == j) distance[i][j] = 0;
            else distance[i][j] = inf;
        }
    }

    for (const auto& rib : ribs) {//setting distances between nodes, that are connected by ribs
        int u = rib.getBegin();
        int v = rib.getEnd();
        int w = rib.getWeight();
        distance[u][v] = w;
    }

    using namespace std::chrono;
    auto begin_time = high_resolution_clock::now();

    for (int k = 0; k < size; ++k) {//Floyd-Warshall main
        for (int i = 0; i < size; ++i) {
            for (int j = 0; j < size; ++j) {
                if (distance[i][k] < inf && distance[k][j] < inf)
                    distance[i][j] = min(distance[i][j], distance[i][k] + distance[k][j]);
            }
        }
    }

    auto end_time = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(end_time - begin_time).count();
    cout << "\n Floyd–Warshall time: " << duration << " microseconds" << endl;
    
}

void floyd_warshall_thread_pool() {
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
}


    
};

int main()
{
    Graph hraph;
    hraph = Graph::GEN(200, 10);
    hraph.bfs(0);
    hraph.bfs_parallel(0);
    hraph.floyd_warshall();
    hraph.floyd_warshall_thread_pool();
    return 0;
}

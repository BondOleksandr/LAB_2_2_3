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
#include <algorithm>
#include <cmath>

using namespace std;

const double eps = 0.001;
const int inf = 1e8;
const double PR_d = 0.85;

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
            this_thread::sleep_for(chrono::nanoseconds(1));
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
    int* bellman_ford_distance;
    double* rank;

public:
    Graph() = default;
    Graph(const int& in_size, const vector<Graph_Rib>& in_ribs): size(in_size), ribs(in_ribs){
        adjacency = new bool*[in_size];
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
    cout << "Floyd–Warshall time: " << duration << " microseconds" << endl;
    
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
}//works well, alot of independent, paralel calculations

void bellman_ford(int start_node) {
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
    std::cout << "Bellman-Ford time: " << duration << " microseconds" << std::endl;

}

void bellman_ford_thread_pool(int start_node) {
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
    std::cout << "Bellman-Ford(ThreadPool) time: " << duration << " microseconds" << std::endl;
}

void PageRank(){
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
std::cout << "PageRank time: " << duration << " microseconds" << std::endl;

delete[] L_arr;
delete[] new_rank;
}

void PageRank_ThreadPool() {
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
    std::cout << "PageRank(threaded) time: " << duration << " microseconds" << std::endl;
}

    
};

int main()
{
    Graph hraph;
    hraph = Graph::GEN(10, 90);
    hraph.bfs(0);
    hraph.bfs_parallel(0);
    hraph.floyd_warshall();
    hraph.floyd_warshall_thread_pool();
    hraph.PageRank();
    hraph.PageRank_ThreadPool();
    
    return 0;
}

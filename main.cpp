#include <iostream>
#include <map>
#include <unordered_map>
#include <set>
#include <vector>
#include <algorithm>
#include <cassert>
#include <queue>
#include "blimit.hpp"
#include <ctime>
#include <thread>
#include <mutex>
#include <pthread.h>
#include <chrono>
#include <queue>
#include <atomic>
#include <iostream>
#include <condition_variable>
#include <thread>
#include <chrono>
#include <array>
#include <future>
#include <fstream>

typedef std::mutex *mutexVector;

typedef std::atomic<long long> *atomicVector;


class edge {
public:
    long long source;
    long long source_oldid;
    long long destination;
    long long destination_oldid;
    long long weight;

    edge(long long s, long long os, long long d, long long od, long long w) {
        source = s;
        source_oldid = os;
        destination = d;
        destination_oldid = od;
        weight = w;
    }

    edge() = default;


    operator bool() const {
        return this != NULL;
    };

    edge reverse() {
        return edge(destination, destination_oldid, source, source_oldid, weight);
    }

    // needed for set operations
    edge reverse_const() const {
        return edge(destination, destination_oldid, source, source_oldid, weight);
    }


    bool operator<(const edge &rhs) const {
        // W(v,w) :<: W(v,x) <=> (W(v,w) < W(v,x)) || (W(v,w)==W(v,x) && w < x).
        return (weight < rhs.weight ||
                (weight == rhs.weight && destination_oldid < rhs.destination_oldid));
    }
};

class graph;

class node {
public:
    unsigned old_id;
    unsigned id;
    std::vector<edge> edges;
    std::set<edge> Suitors;

    node(long long oid, long long newid) : edges(), Suitors() {
        old_id = oid;
        id = newid;
        edges = std::vector<edge>();
        Suitors = std::set<edge>();
    }

    edge *argmax(std::vector<node> *nodes, unsigned method, mutexVector m, atomicVector last) {
        //arg max(v) {W(u,v) : (v in N[u] - T[u]) && (W(u,v) :>: W(v, S[v].last))};
        while (last[id] >= 0) {
            std::unique_lock<std::mutex> lock(m[edges[last[id]].destination]);
            if ((*nodes)[edges[last[id]].destination].Suitors.find(edges[last[id]].reverse_const()) ==
                (*nodes)[edges[last[id]].destination].Suitors.end() &&
                bvalue(method, edges[last[id]].destination_oldid) != 0) {
                const edge *aux = (*nodes)[edges[last[id]].destination].last(method);
                if (!aux || *aux < edges[last[id]].reverse_const()) {
                    return &(edges[last[id]]);
                }
            }
            last[id]--;
        }
        return NULL;
    }

    edge *last(unsigned method) {
        if (!Suitors.empty() && Suitors.size() == bvalue(method, old_id)) {
            return new edge(Suitors.begin()->source,
                            Suitors.begin()->source_oldid,
                            Suitors.begin()->destination,
                            Suitors.begin()->destination_oldid,
                            Suitors.begin()->weight);
        } else
            return NULL;
    }

    void Sinsert(edge n) {
        Suitors.insert(n);
    }

    void Sremove(const edge *n) {
        Suitors.erase(*n);
    }


    operator bool() const {
        return this != NULL;
    };
};

struct graph {
    std::vector<node> nodes;

    void
    insert(const long long &s, const long long &olds, const long long &d, const long long &oldd, const long long &w) {
        nodes[s].edges.push_back(edge(s, olds, d, oldd, w));
    }


    graph(std::string file) : nodes() {
        nodes = std::vector<node>();
        std::string line;
        long long s, d, w;
        long long new_s;
        long long new_d;
        long long last = 0;
        std::map<long long, long long> no2id;
        std::map<long long, long long> id;
        auto it = id.begin();
        std::ifstream input(file);
        while (getline(input, line)) {
            if (line.size() > 1 && line.at(0) != '#') {
                sscanf(line.c_str(), " %Ld %Ld %Ld", &s, &d, &w);
                it = no2id.find(s);
                if (it == no2id.end()) {
                    no2id.emplace(s, last++);
                    new_s = last - 1;
                    nodes.emplace_back(node(s, new_s));
                } else {
                    new_s = it->second;
                }
                it = no2id.find(d);
                if (it == no2id.end()) {
                    no2id.emplace(d, last++);
                    new_d = last - 1;
                    nodes.emplace_back(node(d, new_d));
                } else {
                    new_d = it->second;
                }
                insert(new_s, s, new_d, d, w);
                insert(new_d, d, new_s, s, w);
            }
        }
        input.close();
        for (auto it = nodes.begin(); it != nodes.end(); it++) {
            std::sort(it->edges.begin(), it->edges.end());
        }
    }


    void makeMatch(edge *n, mutexVector m, atomicVector Tsize) {
        Tsize[n->source]++;
        nodes[n->destination].Sinsert(n->reverse());
    }

    void removeMatch(const edge *n, mutexVector m, atomicVector Tsize) {
        nodes[n->source].Sremove(n);
        Tsize[n->destination]--;
    }

    long long sum() {
        long long s = 0;
        for (auto u = nodes.begin(); u != nodes.end(); u++) {
            for (edge e : u->Suitors) {
                s += e.weight;
            }
            u->Suitors.clear();
        }
        return s / 2;
    }
};

void process_node(node &u, unsigned long long method, graph &g, std::set<long long> &r,
                  mutexVector m, std::mutex &R, atomicVector last, atomicVector Tsize) {
    if (bvalue(method, u.old_id) != 0) {
        while (Tsize[u.id] < bvalue(method, u.old_id) && last[u.id] >= 0) {

            edge *xo = u.argmax(&(g.nodes), method, m, last); // u-x
            if (xo != NULL) {

                std::unique_lock<std::mutex> lock(m[xo->destination]); //x-y

                const edge *yo = g.nodes[xo->destination].last(method);
                if (g.nodes[xo->destination].Suitors.find(xo->reverse_const()) ==
                    g.nodes[xo->destination].Suitors.end() && !yo || *yo < xo->reverse()) {
                    const edge *yo = g.nodes[xo->destination].last(method); //x-y
                    g.makeMatch(xo, m, Tsize);
                    if (yo != NULL) {
                        g.removeMatch(yo, m, Tsize);
                        std::unique_lock<std::mutex> lock2(R);
                        r.emplace(yo->destination);
                    }
                }
            } else {
                break;
            }
        }
    }
}


void q_and_process(unsigned long long method, graph &g, std::queue<long long> &q, std::set<long long> &r,
                   mutexVector m,
                   std::mutex &onq, std::mutex &onr, atomicVector last, atomicVector Tsize) {
    std::set<long long> priv_R;
    long long aux;
    onq.lock();
    while (!q.empty()) {
        node &u = g.nodes[q.front()];
        q.pop();
        onq.unlock();
        process_node(u, method, g, r, m, onr, last, Tsize);
        onq.lock();
    }
    onq.unlock();
}

void match(unsigned long long method, int thread_count, graph &g, mutexVector m) {
    std::atomic<long long> *last = new std::atomic<long long>[g.nodes.size()];
    std::atomic<long long> *Tsize = new std::atomic<long long>[g.nodes.size()];
    for (size_t i = 0; i < g.nodes.size(); i++) {
        last[i] = g.nodes[i].edges.size() - 1;
    }
    for (size_t i = 0; i < g.nodes.size(); i++) {
        Tsize[i] = 0;
    }
    std::mutex onQ;
    std::mutex onR;
    std::queue<long long> q;
    for (auto e : g.nodes) {
        q.emplace(e.id);
    }
    while (!q.empty()) {
        std::set<long long> r;
        std::vector<std::thread> threads(0);
        for (int i = 0; i < thread_count - 1; i++) {
            threads.push_back(std::thread{
                    [method, &g, &q, &r, &m, &onQ, &onR, last, Tsize] {
                        q_and_process(method, g, q, r, m, onQ, onR, last, Tsize);
                    }});
        }
        q_and_process(method, g, q, r, m, onQ, onR, last, Tsize);
        for (auto &t : threads) {
            t.join();
        }
        for (auto _ : r) {
            q.emplace(_);
        }
        threads.clear();
        r.clear();
    }
}


int main(int argc, char *argv[]) {
    std::set<int> x;
    std::string line;
    int maxthreads = atoi(argv[1]);
    std::string file = (std::string) argv[2];
    int blimit = atoi(argv[3]);
    graph g = graph(file);
    mutexVector m1 = new std::mutex[g.nodes.size()];
    for (int b = 0; b <= blimit; b++) {
        match(b, maxthreads, g, m1);
        std::cout << g.sum() << std::endl;
    }
    return 0;
}
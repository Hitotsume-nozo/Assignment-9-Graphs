#include <iostream>
#include <vector>
#include <queue>
#include <climits>
using namespace std;

void dijkstra(vector<vector<pair<int, int>>>& adj, int src, int V) {
    vector<int> dist(V, INT_MAX);
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    
    dist[src] = 0;
    pq.push({0, src});
    
    while (!pq.empty()) {
        int u = pq.top().second;
        int d = pq.top().first;
        pq.pop();
        
        if (d > dist[u]) continue;
        
        for (auto& [v, weight] : adj[u]) {
            if (dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                pq.push({dist[v], v});
            }
        }
    }
    
    cout << "Shortest distances from node " << src << ":" << endl;
    for (int i = 0; i < V; i++) {
        if (dist[i] == INT_MAX)
            cout << "Node " << i << ": INF" << endl;
        else
            cout << "Node " << i << ": " << dist[i] << endl;
    }
}

int main() {
    int V = 6;
    vector<vector<pair<int, int>>> adj(V);
    
    adj[0].push_back({1, 4});
    adj[0].push_back({2, 1});
    adj[1].push_back({3, 1});
    adj[2].push_back({1, 2});
    adj[2].push_back({3, 5});
    adj[3].push_back({4, 3});
    adj[4].push_back({5, 1});
    adj[3].push_back({5, 2});
    
    dijkstra(adj, 0, V);
    
    return 0;
}
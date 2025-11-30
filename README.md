# Assignment 9: Graphs

Lab Assignment 9: Graph Algorithms

---

## Question 1: Breadth First Search (BFS)

**Explanation:** BFS is a graph traversal algorithm that explores nodes level by level starting from a source node. It uses a queue data structure to visit all neighbors of the current node before moving to the next level.

**Code:**
```cpp
#include <iostream>
#include <queue>
#include <vector>
using namespace std;

void BFS(vector<vector<int>>& adj, int start, int V) {
    vector<bool> visited(V, false);
    queue<int> q;
    
    visited[start] = true;
    q.push(start);
    
    cout << "BFS Traversal: ";
    while (!q.empty()) {
        int node = q.front();
        q.pop();
        cout << node << " ";
        
        for (int neighbor : adj[node]) {
            if (!visited[neighbor]) {
                visited[neighbor] = true;
                q.push(neighbor);
            }
        }
    }
    cout << endl;
}

int main() {
    int V = 6;
    vector<vector<int>> adj(V);
    
    adj[0].push_back(1);
    adj[0].push_back(2);
    adj[1].push_back(0);
    adj[1].push_back(3);
    adj[1].push_back(4);
    adj[2].push_back(0);
    adj[2].push_back(4);
    adj[3].push_back(1);
    adj[3].push_back(5);
    adj[4].push_back(1);
    adj[4].push_back(2);
    adj[4].push_back(5);
    adj[5].push_back(3);
    adj[5].push_back(4);
    
    BFS(adj, 0, V);
    
    return 0;
}
```

**Output:**
```
BFS Traversal: 0 1 2 3 4 5 
```

---

## Question 2: Depth First Search (DFS)

**Explanation:** DFS is a graph traversal algorithm that explores as far as possible along each branch before backtracking. It uses recursion (or a stack) to visit nodes in a depth-first manner.

**Code:**
```cpp
#include <iostream>
#include <vector>
using namespace std;

void DFSUtil(vector<vector<int>>& adj, int node, vector<bool>& visited) {
    visited[node] = true;
    cout << node << " ";
    
    for (int neighbor : adj[node]) {
        if (!visited[neighbor]) {
            DFSUtil(adj, neighbor, visited);
        }
    }
}

void DFS(vector<vector<int>>& adj, int start, int V) {
    vector<bool> visited(V, false);
    cout << "DFS Traversal: ";
    DFSUtil(adj, start, visited);
    cout << endl;
}

int main() {
    int V = 6;
    vector<vector<int>> adj(V);
    
    adj[0].push_back(1);
    adj[0].push_back(2);
    adj[1].push_back(0);
    adj[1].push_back(3);
    adj[1].push_back(4);
    adj[2].push_back(0);
    adj[2].push_back(4);
    adj[3].push_back(1);
    adj[3].push_back(5);
    adj[4].push_back(1);
    adj[4].push_back(2);
    adj[4].push_back(5);
    adj[5].push_back(3);
    adj[5].push_back(4);
    
    DFS(adj, 0, V);
    
    return 0;
}
```

**Output:**
```
DFS Traversal: 0 1 3 5 4 2 
```

---

## Question 3: Minimum Spanning Tree (Kruskal and Prim)

**Explanation:** A Minimum Spanning Tree connects all vertices in a weighted graph with minimum total edge weight. Kruskal's algorithm uses edge sorting and union-find, while Prim's algorithm grows the MST from a starting vertex using a greedy approach.

**Code:**
```cpp
#include <iostream>
#include <vector>
#include <algorithm>
#include <climits>
using namespace std;

struct Edge {
    int u, v, weight;
    bool operator<(const Edge& other) const {
        return weight < other.weight;
    }
};

class DSU {
    vector<int> parent, rank;
public:
    DSU(int n) {
        parent.resize(n);
        rank.resize(n, 0);
        for (int i = 0; i < n; i++)
            parent[i] = i;
    }
    
    int find(int x) {
        if (parent[x] != x)
            parent[x] = find(parent[x]);
        return parent[x];
    }
    
    void unite(int x, int y) {
        int px = find(x), py = find(y);
        if (px == py) return;
        if (rank[px] < rank[py]) swap(px, py);
        parent[py] = px;
        if (rank[px] == rank[py]) rank[px]++;
    }
};

void kruskal(vector<Edge>& edges, int V) {
    sort(edges.begin(), edges.end());
    DSU dsu(V);
    int mstWeight = 0;
    
    cout << "Kruskal's MST Edges:" << endl;
    for (const Edge& e : edges) {
        if (dsu.find(e.u) != dsu.find(e.v)) {
            dsu.unite(e.u, e.v);
            cout << e.u << " - " << e.v << " : " << e.weight << endl;
            mstWeight += e.weight;
        }
    }
    cout << "Total Weight: " << mstWeight << endl << endl;
}

void prim(vector<vector<pair<int, int>>>& adj, int V) {
    vector<int> key(V, INT_MAX);
    vector<bool> inMST(V, false);
    vector<int> parent(V, -1);
    
    key[0] = 0;
    int mstWeight = 0;
    
    for (int count = 0; count < V; count++) {
        int minKey = INT_MAX, u = -1;
        for (int v = 0; v < V; v++) {
            if (!inMST[v] && key[v] < minKey) {
                minKey = key[v];
                u = v;
            }
        }
        
        inMST[u] = true;
        mstWeight += key[u];
        
        for (auto& [v, weight] : adj[u]) {
            if (!inMST[v] && weight < key[v]) {
                key[v] = weight;
                parent[v] = u;
            }
        }
    }
    
    cout << "Prim's MST Edges:" << endl;
    for (int i = 1; i < V; i++) {
        cout << parent[i] << " - " << i << " : " << key[i] << endl;
    }
    cout << "Total Weight: " << mstWeight << endl;
}

int main() {
    int V = 5;
    vector<Edge> edges = {
        {0, 1, 2}, {0, 3, 6}, {1, 2, 3},
        {1, 3, 8}, {1, 4, 5}, {2, 4, 7}, {3, 4, 9}
    };
    
    vector<vector<pair<int, int>>> adj(V);
    for (const Edge& e : edges) {
        adj[e.u].push_back({e.v, e.weight});
        adj[e.v].push_back({e.u, e.weight});
    }
    
    kruskal(edges, V);
    prim(adj, V);
    
    return 0;
}
```

**Output:**
```
Kruskal's MST Edges:
0 - 1 : 2
1 - 2 : 3
1 - 4 : 5
0 - 3 : 6
Total Weight: 16

Prim's MST Edges:
0 - 1 : 2
1 - 2 : 3
0 - 3 : 6
1 - 4 : 5
Total Weight: 16
```

---

## Question 4: Dijkstra's Shortest Path Algorithm

**Explanation:** Dijkstra's algorithm finds the shortest path from a source vertex to all other vertices in a weighted graph with non-negative edge weights. It uses a priority queue to greedily select the vertex with the minimum distance.

**Code:**
```cpp
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
```

**Output:**
```
Shortest distances from node 0:
Node 0: 0
Node 1: 3
Node 2: 1
Node 3: 4
Node 4: 7
Node 5: 6
```

---

## Conclusion

This assignment provided comprehensive hands-on experience with fundamental graph algorithms. BFS and DFS are essential traversal techniques used in many graph applications including cycle detection, connectivity testing, and pathfinding. Minimum Spanning Tree algorithms (Kruskal's and Prim's) are crucial for network design and optimization problems where we need to connect all nodes with minimum cost. Dijkstra's algorithm is widely used in routing protocols, GPS navigation, and network routing to find shortest paths. Understanding these graph algorithms is fundamental for solving complex problems in computer networks, social networks, transportation systems, and many other domains. Each algorithm has specific use cases based on the graph properties and problem requirements.
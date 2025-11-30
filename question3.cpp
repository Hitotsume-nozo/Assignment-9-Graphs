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
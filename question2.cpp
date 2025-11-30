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
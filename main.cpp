#include<bits/stdc++.h>

using namespace std;


//ifstream fin("bfs.in");
//ofstream fout("bfs.out");

//ifstream fin("dfs.in");
//ofstream fout("dfs.out");

//ifstream fin("sortaret.in");
//ofstream fout("sortaret.out");

//ifstream fin("biconex.in");
//ofstream fout("biconex.out");

//ifstream fin("disjoint.in");
//ofstream fout("disjoint.out");

//ifstream fin("apm.in");
//ofstream fout("apm.out");

//ifstream fin("dijkstra.in");
//ofstream fout("dijkstra.out");

//ifstream fin("bellmanford.in");
//ofstream fout("bellmanford.out");

//ifstream fin("royfloyd.in");
//ofstream fout("royfloyd.out");

ifstream fin("darb.in");
ofstream fout("darb.out");

struct edge{
    int x, y;
    edge(int x1, int y1){
        x = x1;
        y = y1;
    }
    edge(){}
};

struct edge_with_cost{
    int x, y, cost;
    edge_with_cost(int x1, int y1, int c){
        x = x1;
        y = y1;
        cost = c;
    }
    edge_with_cost(){}
    bool operator < (const edge_with_cost &e) const {return cost < e.cost;}
};

class Graph{
private:
    int N, M;
    bool is_directed, has_costs;
    vector<vector<int>> adjacency_list;
    vector<edge_with_cost> weighted_edges_list;
    vector<vector<edge_with_cost>> adjacency_list_with_costs;


public:
    Graph(int N, int M, bool is_directed, bool has_costs);
    void Add_edge(int x, int y);

    void bfs(queue<int> &q, vector<bool> &visited, vector<int> &distance);
    vector<int> bfs_distance(int x);
    void dfs(int x, vector<bool> &visited);
    void bcc(int x, int parent, vector<int> &discovery_time, vector<int> &low, stack<edge> &edge_stack, int &time,
              vector<set<int>> &bc_components, vector<bool> &visited);
    void Topological_Sort(int x, vector<bool>& visited, vector<int>& sorted);

    int Find(int x, vector<int> &parent);
    void Union(int x, int y, vector<int> &parent, vector<int> &rank);
    void Add_edge_with_cost(int x, int y, int c);
    void Kruskal(vector<int> &parent, vector<int> &rank, int &cost, vector<edge> &APM);
    void Dijkstra(vector<int> &distance, priority_queue<pair<int, int>,vector<pair<int, int>>, greater<pair<int, int>>> &heap);
    vector<edge_with_cost> getEdges();
    void BellmanFord (vector<int> &distance, queue<int> &q);

    void RoyFloyd(vector<vector<int>> &matrix);

};

    Graph::Graph(int N, int M, bool is_directed, bool has_costs){
        this->N = N;
        this->M = M;
        this->is_directed = is_directed;
        this->has_costs = has_costs;
        adjacency_list.resize(N+1);
        adjacency_list_with_costs.resize(N+1);
    }

    void Graph::Add_edge(int x, int y){
        adjacency_list[x].push_back(y);
        if(!is_directed)
            adjacency_list[y].push_back(x);
    }

    void Graph::bfs(queue<int> &q, vector<bool> &visited, vector<int> &distance){

        while(!q.empty()){
            int current_node = q.front();
            for(int i=0; i<adjacency_list[current_node].size(); i++){
                if(!visited[adjacency_list[current_node][i]]){
                    q.push(adjacency_list[current_node][i]);
                    visited[adjacency_list[current_node][i]] = true;
                    distance[adjacency_list[current_node][i]] = distance[current_node]+1;
                }
            }
            q.pop();
        }

    };

    vector<int> Graph::bfs_distance(int x){
        queue<int> q;
        vector<bool> visited(N+1);
        vector<int> distance(N+1, -1);

        q.push(x);
        visited[x] = true;
        distance[x] = 0;
        bfs(q, visited, distance);
        return distance;
    };


void BFSInfoarena(){
    int N,M,S;
    fin>>N>>M>>S;
    Graph g(N,M,true,false);
    int x, y;
    for(int i=0; i<M; i++){
        fin >> x >> y;
        g.Add_edge(x, y);
    }
    vector<int> rezolvare = g.bfs_distance(S);
    for(int i = 1; i < rezolvare.size(); i++){
        fout<<rezolvare[i]<<" ";
    }
}


    void Graph::dfs(int x, vector<bool> &visited){
        visited[x] = true;
        for(int i = 0; i < adjacency_list[x].size(); i++){
            if (!visited[adjacency_list[x][i]]){
                dfs(adjacency_list[x][i], visited);
            }
        }
    }

void DFSInfoarena(){
    int N,M;
    fin>>N>>M;
    Graph g(N,M,false,false);
    vector<bool> visited(N+1);

    int x,y;
    for(int i = 0; i < M; i++){
        fin>>x>>y;
        g.Add_edge(x, y);
    }

    int counter = 0;
    for(int i = 1; i <= N; i++){
        if(!visited[i]){
            g.dfs(i, visited);
            counter++;
        }
    }

    fout<<counter;
}



    void Graph::Topological_Sort(int x, vector<bool>& visited, vector<int>& sorted){
        visited[x] = true;
        for(int i=0; i<adjacency_list[x].size(); i++){
            if (!visited[adjacency_list[x][i]])
                Topological_Sort(adjacency_list[x][i], visited, sorted);
        }
        sorted.push_back(x);
    }

void SortaretInfoarena(){
    int N,M;
    fin>>N>>M;
    Graph g(N,M,true,false);

    int x,y;
    for(int i = 0; i < M; i++){
        fin>>x>>y;
        g.Add_edge(x, y);
    }

    vector<bool> visited(N+1);
    vector<int> sorted;
    for(int i = 1; i <= N; i++){
        if(!visited[i])
            g.Topological_Sort(i, visited, sorted);
    }
    for(int i =  sorted.size()-1; i > -1; i--){
        fout<<sorted[i]<<" ";
    }
}

    void Graph::bcc(int x, int parent,  vector<int> &discovery_time, vector<int> &low, stack<edge> &edge_stack,
                     int &time, vector<set<int>> &bc_components, vector<bool> &visited){
        visited[x] = true;
        discovery_time[x] = low[x] = time++;

        for(int i = 0; i < adjacency_list[x].size(); i++){
            if(!visited[adjacency_list[x][i]]){
                edge_stack.push(edge(x, adjacency_list[x][i]));
                bcc(adjacency_list[x][i], x, discovery_time, low, edge_stack, time, bc_components, visited);

                if(low[x] > low[adjacency_list[x][i]])
                    low[x] = low[adjacency_list[x][i]];

                if(discovery_time[x] <= low[adjacency_list[x][i]]){
                    set<int> component;
                    int a, b;
                    do {
                        edge e=edge_stack.top();
                        edge_stack.pop();
                        a = e.x;
                        b = e.y;
                        component.insert(a);
                        component.insert(b);
                    }while(a != x || b != adjacency_list[x][i]);
                    bc_components.push_back(component);
                }
            }
            else{
                if(adjacency_list[x][i] != parent){
                    if(low[x] > discovery_time[adjacency_list[x][i]])
                        low[x] = discovery_time[adjacency_list[x][i]];
                }
            }
        }
    }

void BiconexInfoarena(){
    int N,M;
    fin>>N>>M;
    Graph g(N,M,true,false);

    int x,y;
    for(int i = 0; i < M; i++){
        fin>>x>>y;
        g.Add_edge(x, y);
    }

    vector<set<int>> bc_components;
    vector<int> discovery_time(N+1);
    vector<bool> visited(N+1);
    vector<int> low(N+1);
    stack<edge> edge_stack;
    int time = 0;

    for(int i = 1; i <= N; i++){
        if(!visited[i])
            g.bcc(i, 0, discovery_time, low, edge_stack, time, bc_components, visited);
    }

    fout << bc_components.size() << '\n';
    for(int i = 0; i < bc_components.size(); i++){
        for(set<int>::iterator iter = bc_components[i].begin(); iter != bc_components[i].end(); iter++)
            fout << *iter << " ";
        fout << '\n';
    }
}


    int Graph::Find(int x, vector<int> &parent){
        while (x != parent[x])
            x = parent[x];
        return x;
    }

    void Graph::Union(int x, int y, vector<int> &parent, vector<int> &rank){
        int px = Find(x, parent);
        int py = Find(y, parent);

        if(px == py)
            return;

        if(rank[px] <= rank[py]){
            parent[px] = py;
            rank[py] += rank[px];
        }
        else {
            parent[py] = px;
            rank[px] += rank[py];
        }
    }

void DisjointInfoarena(){
    int N,M;
    fin>>N>>M;
    Graph g(N,M,false,false);

    vector<int> parent(N+1);
    vector<int> rank(N+1, 1);

    for(int i = 0; i <= N; i++){
        parent[i] = i;
    }

    for(int i = 0; i < M; i++){
        int cod, x, y;
        fin >> cod >> x >> y;

        if(cod == 1){
            g.Union(x, y, parent, rank);
        }
        else{
            if (g.Find(x, parent) == g.Find(y, parent)){
                fout<< "DA"<<'\n';
            }
            else{
                fout<<"NU"<<'\n';
            }
        }
    }

}


    void Graph::Add_edge_with_cost(int x, int y, int c){
        adjacency_list_with_costs[x].push_back(edge_with_cost(x, y, c));
        weighted_edges_list.push_back(edge_with_cost(x, y, c));
        if(!is_directed)
            adjacency_list_with_costs[y].push_back(edge_with_cost(y,x,c));
    }


    void Graph::Kruskal(vector<int> &parent, vector<int> &rank, int &cost, vector<edge> &APM){

        sort(weighted_edges_list.begin(),weighted_edges_list.end());

        int edge_no = 0;

        for(int i = 0; i < M; i++){
            int px = Find(weighted_edges_list[i].x, parent);
            int py = Find(weighted_edges_list[i].y, parent);

            if (px == py)
                continue;

            APM[edge_no] = edge(weighted_edges_list[i].x, weighted_edges_list[i].y);
            edge_no++;
            cost += weighted_edges_list[i].cost;

            if (edge_no == N-1)
                break;

            Union(weighted_edges_list[i].x, weighted_edges_list[i].y, parent, rank);
        }

    }

void APMInfoarena(){
    int N,M;
    fin>>N>>M;
    Graph g(N,M,false,true);

    vector<int> parent(N+1);
    vector<int> rank(N+1, 1);
    vector<edge> APM(N-1);
    int cost = 0;

    for(int i = 0; i <= N; i++){
        parent[i] = i;
    }

    int x, y, c;
    for(int i = 0; i < M; i++){
        fin>>x>>y>>c;
        g.Add_edge_with_cost(x, y, c);
    }


    g.Kruskal(parent, rank, cost, APM);

    fout << cost << '\n';
    fout << APM.size() << '\n';
    for(int i = 0; i < APM.size(); i++){
        fout << APM[i].x << " " << APM[i].y << '\n';
    }

}

    void Graph::Dijkstra(vector<int> &distance, priority_queue<pair<int, int>,vector<pair<int, int>>, greater<pair<int, int>>> &heap){
        distance[1] = 0;
        heap.push({0,1});
        vector<bool> visited(N+1);

        while(!heap.empty()){
            int x = heap.top().second;
            cout << x;
            heap.pop();

            if(!visited[x])
                for (int i = 0; i < adjacency_list_with_costs[x].size(); i++){
                        cout<<adjacency_list_with_costs[x][i].y;
                    if(!visited[adjacency_list_with_costs[x][i].y])
                        if(distance[adjacency_list_with_costs[x][i].y] == -1 ||
                    distance[adjacency_list_with_costs[x][i].y] > adjacency_list_with_costs[x][i].cost + distance[x]){
                            distance[adjacency_list_with_costs[x][i].y] = adjacency_list_with_costs[x][i].cost + distance[x];
                            heap.push({distance[adjacency_list_with_costs[x][i].y], adjacency_list_with_costs[x][i].y});
                    }
                }
            visited[x] = 1;
        }
    }

void DijkstraInfoarena(){

    int N,M;
    fin>>N>>M;
    Graph g(N,M,true,true);

    vector<int> distance(N+1, -1);
    priority_queue<pair<int, int>,vector<pair<int, int>>, greater<pair<int, int>>> heap;

    int x, y, c;
    for(int i = 0; i < M; i++){
        fin>>x>>y>>c;
        g.Add_edge_with_cost(x, y, c);
    }

    g.Dijkstra(distance, heap);

    for(int i = 2; i<=N; i++){
        if (distance[i] == -1)
            fout << 0 << " ";
        else fout << distance[i] << " ";
    }

}

    vector<edge_with_cost> Graph::getEdges(){
        return weighted_edges_list;
    }

    void Graph::BellmanFord(vector<int> &distance, queue<int> &q){
        q.push(1);
        distance[1] = 0;
        vector<int> counter(N+1, 0);

        while(!q.empty()){
            int x = q.front();
            q.pop();
            counter[x] ++;
            if(counter[x] > N) return;

            for(int i = 0; i < adjacency_list_with_costs[x].size(); i++){
                if(distance[adjacency_list_with_costs[x][i].y] > distance[x] + adjacency_list_with_costs[x][i].cost){
                    distance[adjacency_list_with_costs[x][i].y] = distance[x] + adjacency_list_with_costs[x][i].cost;
                    q.push(adjacency_list_with_costs[x][i].y);
                }
            }
        }


    }


void BellmanFordInfoarena(){
    int N,M;
    fin>>N>>M;
    Graph g(N,M,true,true);

    vector<int> distance(N+1, INT_MAX);
    queue<int> q;

    int x, y, c;
    for(int i = 0; i < M; i++){
        fin>>x>>y>>c;
        g.Add_edge_with_cost(x, y, c);
    }

    g.BellmanFord(distance, q);

    for(int i = 0; i < M; i++){
        int a = g.getEdges()[i].x;
        int b = g.getEdges()[i].y;
        int c = g.getEdges()[i].cost;

        if(distance[a] !=  INT_MAX && distance[a] + c < distance[b]){
           fout << "Ciclu negativ!";
           return ;
           }
    }
    for(int i = 2; i <= N; i++){
        fout << distance[i] << " ";
    }

}

    void Graph::RoyFloyd(vector<vector<int>>& matrix){
        for(int k = 1; k <= N; k++)
            for(int i = 1; i <= N; i++)
                for(int j = 1; j <= N; j++)
                    if(matrix[i][k] && matrix[k][j] && (matrix[i][j] > matrix[i][k] + matrix[k][j] ||
                                                        !matrix[i][j]) && i != j)
                        matrix[i][j] = matrix[i][k] + matrix[k][j];
    }

void RoyFloydInfoarena(){
    int N;
    fin>>N;
    Graph g(N,0,false,true);

    const int INF = 1e8;
    vector<vector<int>> matrix(N+1, vector<int>(N+1));

    int cost;

    for(int i = 1; i <= N; i++)
        for(int j = 1; j<=N; j++){
            fin >> cost;
            matrix[i][j] = cost;
        }
    g.RoyFloyd(matrix);

    for(int i = 1; i <= N; i++){
        for(int j = 1; j<=N; j++){
            fout << matrix[i][j] << " ";
        }
        fout << '\n';
    }

}

void DarbInfoarena(){
    int N;
    fin>> N;
    Graph g(N, N-1, false, false);

    int x, y;
    for(int i=0; i<N-1; i++){
        fin >> x >> y;
        g.Add_edge(x, y);
    }

    vector<int> distance = g.bfs_distance(1);

    pair<int, int> x_dist = {0,distance[0]};

    for(int i = 0; i < distance.size(); i++)
        if (x_dist.second < distance[i])
            x_dist = {i, distance[i]};

    vector<int> distance2 = g.bfs_distance(x_dist.first);

    for(int i = 0; i < distance2.size(); i++)
        if(x_dist.second < distance2[i])
            x_dist = {i, distance2[i]};

    fout << x_dist.second + 1;
}


int main()
{
//    BFSInfoarena();
//    DFSInfoarena();
//    SortaretInfoarena();
//    BiconexInfoarena();

//    DisjointInfoarena();
//    APMInfoarena();
//    DijkstraInfoarena();
//    BellmanFordInfoarena();

//    RoyFloydInfoarena();
    DarbInfoarena();

    return 0;
}

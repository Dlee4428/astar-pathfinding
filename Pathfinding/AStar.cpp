#include "AStar.h"
#include "Map.h"
#include "SDL.h"

AStar::AStar(Map* m)
	:map(m),
	isSearching(false)
{
	graph = m->GetGraph();
}

AStar::~AStar()
{
}

bool AStar::IsSearching()
{
	return isSearching;
}

void AStar::Search(Node* start, Node* goal)
{
	isSearching = true;
	startNode = start;
	goalNode = goal;
	
	thread = SDL_CreateThread(SearchThread, "", this);
}

void AStar::OnSearchDone()
{
	isSearching = false;

	// Draw the shortest path
	for (auto p : pathFound)
	{
		map->SetPathMap(p->position, Map::RESULT_PATH_FOUND); // the second param value '2' means that it will draw
	}
	// Clear out here
	pathFound.clear();
	visited.clear();
	unvisited.clear();
	distanceDict.clear();
	actualDistanceDict.clear();
	predecessorDict.clear();
}

int AStar::SearchThread(void * data)
{
	AStar* astar = static_cast<AStar*>(data);

	if (!astar->startNode || !astar->goalNode)
	{
		astar->OnSearchDone();
		return 0;
	}

	// To do: Complete this function.
	for (auto& node : astar->graph->GetAllNodes()) {
		astar->distanceDict[&node] = std::numeric_limits<float>::max();
		astar->actualDistanceDict[&node] = std::numeric_limits<float>::max();
		astar->unvisited.push_back(&node);
	}
	astar->distanceDict[astar->startNode] = 0;
	astar->actualDistanceDict[astar->startNode] = 0;

	while (astar->unvisited.size() > 0) {
		SDL_Delay(100);
		Node* u = astar->GetClosestFromUnvisited();

		if (u->position.x == astar->goalNode->position.x && u->position.y == astar->goalNode->position.y) break;
		
		astar->map->SetPathMap(u->position, Map::SEARCH_IN_PROGRESS);
		astar->visited.push_back(u);	
		
		for (auto& v : astar->graph->GetAdjacentNodes(u)) {

			// Go through all the vector nodes till the end of the v
			if (std::find(astar->visited.begin(), astar->visited.end(), v) != astar->visited.end()) 
				continue;

			if (astar->distanceDict[v] > astar->actualDistanceDict[u] + astar->graph->GetDistance(u, v) + astar->graph->GetDistance(v, astar->goalNode)) 
			{
				astar->distanceDict[v] = astar->actualDistanceDict[u] + astar->graph->GetDistance(u, v) + astar->graph->GetDistance(v, astar->goalNode);
			}

			if (astar->actualDistanceDict[v] > astar->actualDistanceDict[u] + astar->graph->GetDistance(u, v)) 
			{
				astar->actualDistanceDict[v] = astar->actualDistanceDict[u] + astar->graph->GetDistance(u, v);
				astar->predecessorDict[v] = u;
			}
		}
	}
	// Generate the shortest path
	astar->pathFound.push_back(astar->goalNode);
	Node* p = astar->predecessorDict[astar->goalNode];

	while (p != astar->startNode) {
		astar->pathFound.push_back(p);
		p = astar->predecessorDict[p];
	}
	astar->pathFound.push_back(p);
	std::reverse(astar->pathFound.begin(), astar->pathFound.end());

	astar->OnSearchDone();
	return 0;
}

Node * AStar::GetClosestFromUnvisited()
{
	float shortest = std::numeric_limits<float>::max();
	Node* shortestNode = nullptr;

	// To do: Complete this function.
	for(auto& node : unvisited) 
	{
		if (shortest > distanceDict[node]) {
			shortest = distanceDict[node];
			shortestNode = node;
		}
	}
	unvisited.erase(std::find(unvisited.begin(), unvisited.end(), shortestNode));
	return shortestNode;
}

void AStar::ValidateDistanceDict(Node * n)
{
	float max = std::numeric_limits<float>::max();
	if (distanceDict.find(n) == distanceDict.end())
	{
		distanceDict[n] = max;
	}
	if (actualDistanceDict.find(n) == actualDistanceDict.end())
	{
		actualDistanceDict[n] = max;
	}
}

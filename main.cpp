/******************************************
 * Dijkstra Algorithm Sample
 * Written by K.Yamamoto
 ******************************************/
 /* TODO List */
 // Sepalate to files
 // Create makefile

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>

int const INF = 2147483647;

class Node
{
public:
    std::string label;
    int         distanceFromStart;

    Node(std::string label);
    void setNeighborInfo(Node* neighbor, int distance);
    bool operator<(const Node &right) const {return this->label < right.label;}
    // Getter
    std::vector<Node*> getNeighbors()                    { return neighbors; };
    int                getNeighborsDistance(Node* node)  { return distances[node]; };
    Node*              getPreviousNode()                 { return previousNode; };
    // Setter
    void setPreviousNode(Node* pn) { previousNode = pn; };
private:
    Node                *previousNode;
    std::vector<Node*>  neighbors;
    std::map<Node*, int> distances;
};

Node::Node(std::string label) {
    this->label       = label;
    distanceFromStart = INF;
}

void Node::setNeighborInfo(Node* neighbor, int distance) {
    neighbors.push_back(neighbor);
    distances.insert(std::make_pair(neighbor, distance));
};


class GraphReader
{
public:
    GraphReader() {};
    std::vector<Node> readNodes(std::string filename);
    std::vector<Node> readEdges(std::string filename, std::vector<Node>& nodes);
private:
};

std::vector<Node> GraphReader::readNodes(std::string filename) {
    std::vector<Node> nodes;
    std::ifstream     nodefile;
    std::string       buffer;
    std::string       nodelabel;

    nodefile.open(filename, std::ios::in);

    std::getline(nodefile, buffer);
    while (!nodefile.eof()) {
        nodes.push_back(buffer);
        std::getline(nodefile, buffer);
    }

    return nodes;
};

std::vector<Node> GraphReader::readEdges(std::string filename, std::vector<Node>& nodes) {
    const char delimiter = ' ';
    std::ifstream edgefile;
    std::string line_buffer, buffer;
    std::string nodelabel;

    edgefile.open(filename, std::ios::in);

    std::getline(edgefile, line_buffer);
    for (int i=0; i < nodes.size(); i++) {
        std::istringstream line_separater(line_buffer);
        for (int j=0; j < nodes.size(); j++) {
            std::getline(line_separater, buffer, delimiter);
            if(buffer != "No") {
                nodes[i].setNeighborInfo(&nodes[j], std::stoi(buffer));
            } 
        }
        std::getline(edgefile, line_buffer);
    }

    return nodes;
};


int getMinimumDistanceIndex(std::vector<Node*> queue);

int main(int argc, char* argv[])
{
    std::string nodefilename = argv[1];
    std::string edgefilename = argv[2];

    std::vector<Node> nodes;
    std::vector<Node*> queue;

    Node *start, *goal, *targetNode;

    /*******************/
    /* Read Graph Info */
    /*******************/
    GraphReader gr;
    nodes = gr.readNodes(nodefilename);
    nodes = gr.readEdges(edgefilename, nodes);

    nodes.front().distanceFromStart = 0;

    for(int i=0; i < nodes.size(); i++) {
        queue.push_back(&nodes[i]);
    }
    start = &nodes.front();
    goal  = &nodes.back();

    start->setPreviousNode(NULL);
    

    /*************/
    /* Main Loop */
    /*************/
    while(!queue.empty()) {

        // Remove node which has minimum distance from start, from Queue
        int removeIndex = getMinimumDistanceIndex(queue);
        targetNode = queue[removeIndex];
        queue.erase(queue.begin() + removeIndex);

        // TODO : Make function
        std::vector<Node*> neighbors;
        neighbors = targetNode->getNeighbors();
        for(int i=0; i < neighbors.size(); i++) {
            Node *neighbor = neighbors[i];
            int startToNeighbor = neighbor->distanceFromStart;
            int startToNeighborViaTarget = targetNode->distanceFromStart
                                         + neighbor->getNeighborsDistance(targetNode);
            if(startToNeighborViaTarget < startToNeighbor) {
                neighbor->distanceFromStart = startToNeighborViaTarget;
                neighbor->setPreviousNode(targetNode);
            }
        }
    }

    Node *tmp = goal;
    while(tmp != NULL) {
        std::cout << tmp->label << std::endl;
        tmp = tmp->getPreviousNode();
    }
    std::cout << "Total Distance : " << goal->distanceFromStart << std::endl;
}

int getMinimumDistanceIndex(std::vector<Node*> queue) {
    int distanceFromStartMin = INF;
    int removeIndex          = -1;
    for(int i=0; i < queue.size(); i++) {
        int distanceFromStart = queue[i]->distanceFromStart;
        if(distanceFromStart < distanceFromStartMin) {
            removeIndex = i;
            distanceFromStartMin = distanceFromStart;
        }
    }
    return removeIndex;
};

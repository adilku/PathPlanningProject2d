#ifndef SEARCH_H
#define SEARCH_H
#include "ilogger.h"
#include "searchresult.h"
#include "environmentoptions.h"
#include <list>
#include <vector>
#include <math.h>
#include <limits>
#include <chrono>
#include <optional>
#include <set>
#include <unordered_set>
#include <map>

class Search
{
    public:
        Search();
        ~Search(void);
        SearchResult startSearch(ILogger *Logger, const Map &Map, const EnvironmentOptions &options);

    protected:
        //CODE HERE

        //Hint 1. You definetely need class variables for OPEN and CLOSE

        //Hint 2. It's a good idea to define a heuristic calculation function, that will simply return 0
        //for non-heuristic search methods like Dijkstra

        //Hint 3. It's a good idea to define function that given a node (and other stuff needed)
        //will return it's sucessors, e.g. unordered list of nodes

        //Hint 4. working with OPEN and CLOSE is the core
        //so think of the data structures that needed to be used, about the wrap-up classes (if needed)
        //Start with very simple (and ineffective) structures like list or vector and make it work first
        //and only then begin enhancement!
        struct compare {
            bool operator()(const Node &lhs,
                            const Node &rhs) const {
                return lhs.F < rhs.F;
            }
        };


        SearchResult                    sresult; //This will store the search result
        std::list<Node>                 lppath, hppath; //
        std::list<Node> open;
        std ::multimap<std::pair<int, int>, std::multiset<Node>::iterator> open_map;
        std::multiset<Node, compare> open_heap;
        std::vector<std::vector<Node>> close;
        //CODE HERE to define other members of the class
        double get_heuristic(Point from, Point to, const EnvironmentOptions &options) const;


    std::vector<Node> CheckNeighbours(Node &v, const Map &map, const EnvironmentOptions &options);

    void makePrimaryPath(Node* curNode);

    void makeSecondaryPath();
};
#endif

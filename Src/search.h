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
#include <tuple>

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
                return std::tuple(lhs.F, lhs.g, lhs.i, lhs.j) < std::tuple(rhs.F, rhs.g, rhs.i, rhs.j);
            }
        };

        struct hash_pair {
            template <class T1, class T2>
            size_t operator()(const std::pair<T1, T2>& p) const
            {
                auto hash1 = std::hash<T1>{}(p.first);
                auto hash2 = std::hash<T2>{}(p.second);
                return hash1 ^ hash2;
            }
        };

        SearchResult                    sresult; //This will store the search result
        std::list<Node>                 lppath, hppath; //
        std ::unordered_map<std::pair<int, int>, std::set<Node>::iterator, hash_pair> open_map;
        std::set<Node, compare> open_heap;
        std::unordered_map<std::pair<int, int>, Node, hash_pair> close_map;
        //CODE HERE to define other members of the class
        double get_heuristic(Point from, Point to, const EnvironmentOptions &options) const;


    void CheckNeighbours(Node &v, const Map &map, const EnvironmentOptions &options, std::vector<Node> &neighbours);

    void makePrimaryPath(Node* curNode);

    void makeSecondaryPath();
};
#endif

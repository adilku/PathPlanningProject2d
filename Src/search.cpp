#include "search.h"
#include <chrono>


Search::Search() {
//set defaults here
}

Search::~Search() {}


double Search::get_heuristic(Point from, Point to, const EnvironmentOptions &options) const {
    if (options.metrictype == CN_SP_MT_DIAG) {
        return CN_CELL_SIZE * abs(abs(to.i - from.i) - abs(to.j - from.j)) +
               CN_SQRT_TWO * std::min(abs(to.i - from.i), abs(to.j - from.j));
    } else if (options.metrictype == CN_SP_MT_MANH) {
        return CN_CELL_SIZE * (abs(from.i - to.i) + abs(from.j - to.j));
    } else if (options.metrictype == CN_SP_MT_EUCL) {
        return CN_CELL_SIZE * sqrt((to.i - from.i) * (to.i - from.i) + (to.j - from.j) * (to.j - from.j));
    } else if (options.metrictype == CN_SP_MT_CHEB) {
        return std::max(abs(from.i - to.i), abs(from.j - to.j));
    }
    return 0;
}


std::vector<Node> Search::CheckNeighbours(Node &v, const Map &map, const EnvironmentOptions &options) {
    std::vector<Node> neighbours;
    double hWeight = options.hweight;
    for (auto &micro_node : DISALLOWED_DIAG_MOVES) {
        int i = micro_node.first;
        int j = micro_node.second;
        if (map.CellOnGrid(v.i + i, v.j + j) && map.CellIsTraversable(v.i + i, v.j + j)) {
            neighbours.emplace_back(v.i + i, v.j + j,
                                    v.g + CN_CELL_SIZE +
                                    hWeight * get_heuristic({v.i + i, v.j + j}, map.getCoordinatesGoal(), options),
                                    v.g + CN_CELL_SIZE,
                                    get_heuristic({v.i + i, v.j + j}, map.getCoordinatesGoal(), options), &v);
        }
    }
    if (options.allowdiagonal) {
        for (auto &cur_point : ALLOWED_DIAG_MOVES) {
            int i = cur_point.first;
            int j = cur_point.second;
            if (map.CellOnGrid(v.i + i, v.j + j) && map.CellIsTraversable(v.i + i, v.j + j)) {
                if (options.allowdiagonal) {
                    bool vert = map.CellOnGrid(v.i, v.j + j) && map.CellIsTraversable(v.i, v.j + j);
                    bool horiz = map.CellOnGrid(v.i + i, v.j) && map.CellIsTraversable(v.i + i, v.j);
                    if ((vert && !horiz && options.cutcorners) ||
                        (!vert && horiz && options.cutcorners) ||
                        (!vert && !horiz && options.allowsqueeze) ||
                        (vert && horiz)
                            ) {
                        neighbours.emplace_back(v.i + i, v.j + j,
                                                v.g + CN_SQRT_TWO +
                                                hWeight *
                                                get_heuristic({v.i + i, v.j + j}, map.getCoordinatesGoal(),
                                                              options),
                                                v.g + CN_SQRT_TWO,
                                                get_heuristic({v.i + i, v.j + j}, map.getCoordinatesGoal(), options),
                                                &v);
                    }
                }
            }
        }
    }
    return neighbours;
}


SearchResult Search::startSearch(ILogger *Logger, const Map &map, const EnvironmentOptions &options) {
    auto startTime = std::chrono::high_resolution_clock::now();

    Point start = map.getCoordinatesStart();
    Point goal = map.getCoordinatesGoal();

    Node *searchedGoal = nullptr;

    int cnt_steps = 0;


    auto temp = open_heap.insert(
            Node(start.i, start.j, 1 * get_heuristic(start, goal, options), 0, get_heuristic(start, goal, options),
                 nullptr
            ));

    while (!open_heap.empty()) {
        ++cnt_steps;
        auto cur_el = *open_heap.begin();
        open_heap.erase(open_heap.begin());
        open_map.erase({cur_el.i, cur_el.j});
        close_map[{cur_el.i, cur_el.j}] = cur_el;
        int cur_I = cur_el.i;
        int cur_J = cur_el.j;
        auto cur_v = &close_map[{cur_I, cur_J}];
        if (cur_v->i == goal.i && cur_v->j == goal.j) {
            searchedGoal = &close_map[{cur_v->i, cur_v->j}];
            break;
        }
        for (auto &neighbour : CheckNeighbours(*cur_v, map, options)) {
            if (close_map[{neighbour.i, neighbour.j}].i == -1) {
                auto it = open_map.find({neighbour.i, neighbour.j});
                if (it != open_map.end() && it->second->g > neighbour.g) {
                    open_heap.erase(it->second);
                    auto new_val = open_heap.insert(neighbour);
                    open_map.erase(it);
                    open_map.insert({{neighbour.i, neighbour.j}, new_val.first});
                } else if (it == open_map.end()) {
                    auto temp1 = open_heap.insert(neighbour);
                    open_map.insert({{neighbour.i, neighbour.j}, temp1.first});
                }
            }
        }
    }
    sresult.pathfound = (searchedGoal != nullptr);
    if (searchedGoal != nullptr) {
        sresult.pathlength = searchedGoal->g;
    } else {
        sresult.pathlength = 0;
    }
    sresult.nodescreated = open_heap.size() + cnt_steps;
    sresult.numberofsteps = cnt_steps;
    if (sresult.pathfound) {
        makePrimaryPath(searchedGoal);
        makeSecondaryPath();
    }
    auto endTime = std::chrono::high_resolution_clock::now();
    sresult.time = (std::chrono::duration<double>(endTime - startTime)).count();
    sresult.hppath = &hppath;
    sresult.lppath = &lppath;
    return sresult;
}

void Search::makePrimaryPath(Node *curNode) {
    while (curNode) {
        lppath.push_front(*curNode);
        curNode = curNode->parent;
    }
}

void Search::makeSecondaryPath() {
    if (!lppath.empty()) {
        hppath.push_front(*lppath.begin());
        if (lppath.size() != 1) {
            auto cur = std::next(lppath.begin());
            auto prev = lppath.begin();
            for (auto it = next(lppath.begin(), 2); it != lppath.end(); it = next(it)) {
                int dx_1 = it->i - cur->i;
                int dx_2 = cur->i - prev->i;
                int dy_1 = it->j - cur->j;
                int dy_2 = cur->j - prev->j;
                if (!(dx_1 == dx_2 && dy_1 == dy_2)) {
                    hppath.push_back(*cur);
                }
                prev = cur;
                cur = it;
            }
        }
    }
}

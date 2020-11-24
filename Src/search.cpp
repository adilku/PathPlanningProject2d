#include "search.h"


Search::Search() {
//set defaults here
}

Search::~Search() {}


double Search::get_heuristic(Point from, Point to, const EnvironmentOptions &options) const {
    if (options.metrictype == CN_SP_MT_DIAG) {
        return CN_CELL_SIZE * abs(abs(to.i - from.i) - abs(to.j - from.j)) +
               CN_SQRT_TWO * std::min(abs(to.i - from.i), abs(to.j - from.j));
    } else if (options.metrictype == CN_SP_MT_MANH) {
        return CN_CELL_SIZE * (abs(from.i - to.i) + abs(from.i - to.i));
    } else if (options.metrictype == CN_SP_MT_EUCL) {
        return CN_CELL_SIZE * sqrt((to.i - from.i) * (to.i - from.i) + (to.j - from.j));
    } else if (options.metrictype == CN_SP_MT_CHEB) {
        return std::max(abs(from.i - to.i), abs(from.j - to.j));
    }
    return 0;
}


std::vector<Node> Search::CheckNeighbours(Node &v, const Map &map, const EnvironmentOptions &options) {
    std::vector<Node> neighbours;
    for (auto &micro_node : DISALLOW_DIAG_MOVES) {
        int i = micro_node.first;
        int j = micro_node.second;
        if (map.CellOnGrid(v.i + i, v.j + j) && map.CellIsTraversable(v.i + i, v.j + j)) {
            neighbours.emplace_back(v.i + i, v.j + j,
                                    v.g + CN_CELL_SIZE +
                                    1 * get_heuristic({v.i + i, v.j + j}, map.getCoordinatesGoal(), options),
                                    v.g + CN_CELL_SIZE,
                                    get_heuristic({v.i + i, v.j + j}, map.getCoordinatesGoal(), options), &v);
        }
    }
    if (options.allowdiagonal) {
        for (auto &cur_point : ALLOW_DIAG_MOVES) {
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
                                                1 *
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

    //need to implement

    //Node temp;
    /*sresult.pathfound = ;
    sresult.nodescreated =  ;
    sresult.numberofsteps = ;
    sresult.time = ;
    sresult.hppath = &hppath; //Here is a constant pointer
    sresult.lppath = &lppath;*/

    Point start = map.getCoordinatesStart();
    Point goal = map.getCoordinatesGoal();
    int width = map.getMapWidth();
    int height = map.getMapHeight();

    Node *searchedGoal = nullptr;

    close.resize(height, std::vector<Node>(width));

    int cnt_steps = 0;

    open.emplace_back(start.i, start.j, 1 * get_heuristic(start, goal, options), 0, get_heuristic(start, goal, options),
                      nullptr
    );
    /*
    open_heap.insert(
            Node(start.i, start.j, 1 * Heuristic(start, goal, options), 0, get_heuristic(start, goal, options), nullptr
            ));
    */
    while (!open.empty()) {
        ++cnt_steps;
        auto cur_el = std::min_element(open.begin(), open.end());
        close[cur_el->i][cur_el->j] = *cur_el;
        int cur_I = cur_el->i;
        int cur_J = cur_el->j;
        auto cur_v = &close[cur_I][cur_J];
        if (cur_v->i == goal.i && cur_v->j == goal.j) {
            searchedGoal = &close[cur_v->i][cur_v->j];
            break;
        }
        open.erase(cur_el);
        for (auto &neighbour : CheckNeighbours(*cur_v, map, options)) {
            if (close[neighbour.i][neighbour.j].i == -1) {
                auto it = std::find_if(open.begin(), open.end(), [neighbour](Node el) {
                    return el.i == neighbour.i && el.j == neighbour.j;
                });
                if (it != open.end() && it->g > neighbour.g) {
                    *it = neighbour;
                } else if (it == open.end()) {
                    open.push_back(neighbour);
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
    sresult.nodescreated = open.size() + cnt_steps;
    sresult.numberofsteps = cnt_steps;
    if (sresult.pathfound) {
        makePrimaryPath(searchedGoal);
        makeSecondaryPath();
    }
    //sresult.time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time).count();
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

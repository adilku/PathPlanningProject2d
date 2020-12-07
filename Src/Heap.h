//
// Created by Adilkhan Kussidenov on 11/23/20.
//

#include "node.h"
#include "map.h"

#ifndef ADILKHAN_KUSSIDENOV_ASEARCH_HEAP_H
#define ADILKHAN_KUSSIDENOV_ASEARCH_HEAP_H


class Heap {
public:
    std::vector<Node> items;

    size_t heap_size() {
        return items.size();
    }

    auto insert(Node value) {
        items.push_back(value);
        int i = heap_size() - 1;
        int parent = (i - 1) / 2;
        while (i > 0 && items[parent].F > items[i].F) {
            std::swap(items[i], items[parent]);
            i = parent;
            parent = (i - 1) / 2;
        }
        return next(items.begin(), i);
    }

    void fix_heap(int i) {
        int left;
        int right;
        int small;
        while (true) {
            left = 2 * i + 1;
            right = 2 * i + 2;
            small = i;
            if (right < (int)heap_size() && items[right].F < items[small].F) {
                small = right;
            }
            if (left < (int)heap_size() && items[left].F < items[small].F) {
                small = left;
            }
            if (small == i) {
                break;
            }
            std::swap(items[i], items[small]);
            i = small;
        }
    }

    auto get_min() {
        Node minimum = items[0];
        items[0] = items.back();
        items.pop_back();
        fix_heap(0);
        return minimum;
    }

    auto get_iterator_min() {
        return items.begin();
    }

    void change_val_by_val(std::vector<Node>::iterator cur_it, Node value) {
        *cur_it = value;
        size_t index_swap = std::distance(items.begin(), cur_it);
        if (items[index_swap].F > items.back().F) {
            std::swap(items.back(), items[index_swap]);
        }
        fix_heap(0);
    }
};


#endif //ADILKHAN_KUSSIDENOV_ASEARCH_HEAP_H

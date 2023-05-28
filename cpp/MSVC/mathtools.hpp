/**
 * @file mathtools.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-03-30
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once


#include<iostream>
#include <algorithm>
#include <vector>


// Generate the next combination of k elements from a vector of n elements
inline bool next_combination(std::vector<int>& indices, int n, int k) {
    int i = k - 1;
    ++indices[i];
    while ((i > 0) && (indices[i] >= n - k + i + 1)) {
        --i;
        ++indices[i];
    }
    if (indices[0] > n - k) {
        return false;
    }
    for (i = i + 1; i < k; ++i) {
        indices[i] = indices[i - 1] + 1;
    }
    return true;
}
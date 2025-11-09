/*
 * PathOptimization.h
 * Implements the LSRB path simplification algorithm.
 */

#pragma once

#include <Arduino.h>
#include <string.h>

class PathOptimization {
public:
    PathOptimization();

    // Optimizes the path string in-place.
    // Runs iteratively until no more changes can be made.
    void optimize(char* path);

private:
    // Helper function to replace a substring
    void replaceSubstring(char* str, const char* find, const char* replace);
};
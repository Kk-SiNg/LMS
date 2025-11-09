/*
 * PathOptimization.h
 * Implements the LSRB path simplification algorithm.
 * Uses Arduino String class for safety and simplicity.
 */

#pragma once

#include <Arduino.h> // Required for String class

class PathOptimization {
public:
    PathOptimization();

    // Optimizes the path string in-place.
    // Runs iteratively until no more changes can be made.
    void optimize(String &path); // Changed to take a String reference
};
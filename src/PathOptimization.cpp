/*
 * PathOptimization.cpp
 * Implementation of iterative path simplification for path AND distance.
 */

#include "PathOptimization.h"

PathOptimization::PathOptimization() {}

/**
 * @param path The path string (e.g., "LBLSS") to be modified in-place.
 * @param segments The array of segment lengths corresponding to the path.
 * @param pathLength The length of the path (number of segments), passed by reference.
 */

void PathOptimization::optimize(String &path, long segments[10000], int &pathLength) {
    // We must build a new path and segment array, as in-place modification
    // with string/array shifting is too complex and slow.
    String newPath = "";
    long newSegments[10000]; // Assume max 100 segments
    int newIndex = 0;
    
    int i = 0;
    while (i < pathLength) {
        // Check for 3-character rules (e.g., "LBL", "LBR")
        if (i <= pathLength - 3) {
            String sub = path.substring(i, i + 3);
            long new_dist = segments[i];

            if (sub == "LBR") {
                newPath += 'B';
                newSegments[newIndex] = new_dist;
                i += 3; // Skip over the 3 processed chars
                newIndex++;
                continue; // Continue to next loop iteration
            }
            else if (sub == "LBS") {
                newPath += 'R';
                newSegments[newIndex] = new_dist;
                i += 3;
                newIndex++;
                continue;
            }
            else if (sub == "RBL") {
                newPath += 'B';
                newSegments[newIndex] = new_dist;
                i += 3;
                newIndex++;
                continue;
            }
            else if (sub == "SBL") {
                newPath += 'R';
                newSegments[newIndex] = new_dist;
                i += 3;
                newIndex++;
                continue;
            }
            else if (sub == "SBS") {
                newPath += 'B';
                newSegments[newIndex] = new_dist;
                i += 3;
                newIndex++;
                continue;
            }
            else if (sub == "LBL") {
                newPath += 'S';
                newSegments[newIndex] = new_dist;
                i += 3;
                newIndex++;
                continue;
            }
        }

        // If no 3-char rule matched, just copy the current step
        newPath += path[i];
        newSegments[newIndex] = segments[i];
        i++;
        newIndex++;
    }

    // Copy the newly optimized path and segments back
    path = newPath;
    pathLength = newIndex;
    segments = newSegments;
}
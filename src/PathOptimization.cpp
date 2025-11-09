/*
 * PathOptimization.cpp
 * Implementation of iterative path simplification.
 */

#include "PathOptimization.h"

PathOptimization::PathOptimization() {}

void PathOptimization::optimize(char* path) {
    bool changesMade;
    
    // Loop until a full pass makes no changes
    do {
        changesMade = false;
        char tempPath; // Temporary buffer
        strcpy(tempPath, path);
        
        // Rule 1: LBR -> B
        replaceSubstring(tempPath, "LBR", "B");
        // Rule 2: LBS -> R
        replaceSubstring(tempPath, "LBS", "R");
        // Rule 3: RBL -> B
        replaceSubstring(tempPath, "RBL", "B");
        // Rule 4: SBL -> R
        replaceSubstring(tempPath, "SBL", "R");
        // Rule 5: SBS -> B
        replaceSubstring(tempPath, "SBS", "B");
        // Rule 6: LBL -> S
        replaceSubstring(tempPath, "LBL", "S");

        if (strcmp(path, tempPath)!= 0) {
            changesMade = true;
            strcpy(path, tempPath); // Commit change
        }

    } while (changesMade);
}

// Standard C-string find-and-replace
void PathOptimization::replaceSubstring(char* str, const char* find, const char* replace) {
    char* pos;
    char temp;
    int findLen = strlen(find);
    int replaceLen = strlen(replace);

    while ((pos = strstr(str, find))!= NULL) {
        // Copy string before the match
        strncpy(temp, str, pos - str);
        temp[pos - str] = '\0';

        // Add the replacement string
        strcat(temp, replace);

        // Add the rest of the original string
        strcat(temp, pos + findLen);

        // Copy back to original string
        strcpy(str, temp);
    }
}
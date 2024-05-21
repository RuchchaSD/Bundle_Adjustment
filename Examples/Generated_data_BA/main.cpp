#include "Generated_dataSet_BA.h"
#include <iostream>

/**
 * @brief Main function to run the bundle adjustment example.
 *
 * creates an instance of Generated_dataSet_BA and calls the do_BA method to perform bundle adjustment.
 *
 */
int main(int argc, char** argv) {
    Generated_dataSet_BA dataSet;
    dataSet.do_BA();
    return 0;
}
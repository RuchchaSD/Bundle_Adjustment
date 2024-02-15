#ifndef Data_Generate_H
#define Data_Generate_H


#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <utility> // For std::pair
#include <iostream>

using StringPairVectors = std::pair<std::vector<std::string>, std::vector<std::string>>;

StringPairVectors readCsvColumns(const std::string& filePath) {
    std::ifstream file(filePath);
    std::vector<std::string> firstColumn, secondColumn;

    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filePath << std::endl;
        // Return empty vectors if file can't be opened
        return { firstColumn, secondColumn };
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value1, value2;

        // Get first column value
        if (std::getline(ss, value1, ',')) {
            firstColumn.push_back(value1);
        }

        // Get second column value
        if (std::getline(ss, value2, ',')) {
            secondColumn.push_back(value2);
        }
        // Assuming you only care about the first two columns
    }

    file.close();
    return { firstColumn, secondColumn };
}


void writeResultsCsv(const std::string& filePath, const std::string& name = "Results", const std::vector<double>& results = {}) {
    std::ofstream file(filePath, std::ios::app);//make this only update the file
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filePath << std::endl;
        return;
    }
    file << name << ",";
    for (size_t i = 0; i < results.size() - 1; i++) {
        file << results[i] << ",";
    }
    file << results[results.size() - 1] << std::endl;
    file.close();
}
#endif // !Data_Generate_H



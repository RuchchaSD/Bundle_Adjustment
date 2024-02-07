#include "data_generate.h"

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
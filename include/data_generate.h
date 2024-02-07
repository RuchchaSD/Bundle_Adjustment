#ifndef Data_Generate_H
#define Data_Generate_H


#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <utility> // For std::pair
#include <iostream>

using StringPairVectors = std::pair<std::vector<std::string>, std::vector<std::string>>;
StringPairVectors readCsvColumns(const std::string& filePath);



#endif // !Data_Generate_H
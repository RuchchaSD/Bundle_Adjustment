#include "data_generate.h"


int main(int argc, char** argv) {
	std::string prefix = "..\\";
	std::string filePath = prefix + "..\\..\\..\\..\\Other\\data\\results.csv";

	std::vector<double> Results = { 1,2,3 };
	std::string name = "Test 1";
	writeResultsCsv(filePath, name, Results);

	Results = { 4,5.4,9 };
	name = "Test 2";
	writeResultsCsv(filePath, name, Results);

	Results = {8.45,6456,8.8655,4655};
	name = "Test 3";
	writeResultsCsv(filePath, name, Results);
	return 0;
}

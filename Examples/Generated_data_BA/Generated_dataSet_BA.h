#pragma once
#include <iostream>
#include <vector>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <utility> 
#include <fstream>
#include <sstream>
#include "FactoryProducer.h"
#include "Settings.h"
#include "Reprojection_Generated_dataset.h"
#include "Timer.h"

using StringPairVectors = std::pair<std::vector<std::string>, std::vector<std::string>>;

class Generated_dataSet_BA
{
private:
	std::shared_ptr<Timer> timer;
	
	StringPairVectors readCsvColumns(const std::string& filePath);
	std::vector<std::vector<std::string>> readCSV(const std::string& filename);
	Eigen::VectorXd convertToVector(std::vector<std::string> data, int start, int end);
	void writeResultsCsv(const std::string& filePath, const std::string& name = "Results", const std::vector<double>& results = {});

	std::unique_ptr<OptimizerSettings> makeSettings();
public:
	Generated_dataSet_BA();
	~Generated_dataSet_BA();

	void do_BA();


};


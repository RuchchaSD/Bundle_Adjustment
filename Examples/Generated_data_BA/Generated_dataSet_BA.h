//Generated_dataSet_BA.h
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

/**
 * @brief Class for performing bundle adjustment on a generated dataset.
 *
 * This class reads data from CSV files, sets up the optimizer, and performs bundle adjustment.
 * It includes methods for reading data, converting data, and setting up the optimizer.
 */
class Generated_dataSet_BA
{
private:
    std::shared_ptr<Timer> timer; ///< Shared pointer to a Timer object.

    /**
     * @brief Reads the first two columns from a CSV file.
     * @param filePath Path to the CSV file.
     * @return A pair of vectors containing the first and second columns.
     */
    StringPairVectors readCsvColumns(const std::string& filePath);

    /**
     * @brief Reads the entire CSV file into a 2D vector.
     * @param filename Path to the CSV file.
     * @return A 2D vector where each element is a row from the CSV file.
     */
    std::vector<std::vector<std::string>> readCSV(const std::string& filename);

    /**
     * @brief Converts a vector of strings to an Eigen vector.
     * @param data Vector of strings.
     * @param start Start index for conversion.
     * @param end End index for conversion.
     * @return Converted Eigen vector.
     */
    Eigen::VectorXd convertToVector(std::vector<std::string> data, int start, int end);

    /**
     * @brief Writes results to a CSV file.
     * @param filepath Path to the CSV file.
     * @param columnNames Vector of column names.
     * @param data 2D vector of data to write.
     */
    void writeResultsCsv(const std::string& filepath, const std::vector<std::string>& columnNames, const std::vector<std::vector<double>>& data = {});

    /**
     * @brief Asks the user for the file location.
     * @param fileName Name of the file.
     * @return The file location provided by the user.
     */
    std::string askFileLocation(const std::string& fileName);

    /**
     * @brief Asks the user a question and retrieves the answer.
     * @param question The question to ask.
     * @return The answer provided by the user.
     */
    std::string askQuestion(const std::string& question);

    /**
     * @brief Creates and configures the optimizer settings.
     * @return A unique pointer to the configured optimizer settings.
     */
    std::unique_ptr<OptimizerSettings> makeSettings();

public:
    Generated_dataSet_BA();
    ~Generated_dataSet_BA();

    /**
     * @brief Performs bundle adjustment using the provided dataset.
     *
     * This method reads the dataset, sets up the optimizer, and runs the optimization process.
     * It includes reading pose, landmark, and projection data, configuring the optimizer, and adding vertices and edges.
     */
    void do_BA();
};

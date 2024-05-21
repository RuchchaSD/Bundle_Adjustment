//Generated_dataSet_BA.cpp

#include "Generated_dataSet_BA.h"

/**
 * @brief Reads the first two columns from a CSV file.
 *
 * This method opens the specified CSV file, reads its content, and extracts the first two columns into separate vectors.
 *
 * @param filePath Path to the CSV file.
 * @return A pair of vectors containing the first and second columns.
 * 
 */
StringPairVectors Generated_dataSet_BA::readCsvColumns(const std::string& filePath)
{
    std::ifstream file(filePath);
    std::vector<std::string> firstColumn, secondColumn;

    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filePath << std::endl;
        return { firstColumn, secondColumn };
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value1, value2;

        if (std::getline(ss, value1, ',')) {
            firstColumn.push_back(value1);
        }

        if (std::getline(ss, value2, ',')) {
            secondColumn.push_back(value2);
        }
    }

    file.close();
    return { firstColumn, secondColumn };
}

/**
 * @brief Reads the entire CSV file into a 2D vector.
 *
 * This method reads the specified CSV file and stores each row as a vector of strings in a 2D vector.
 *
 * @param filename Path to the CSV file.
 * @return A 2D vector where each element is a row from the CSV file.
 */
std::vector<std::vector<std::string>> Generated_dataSet_BA::readCSV(const std::string& filename)
{
    std::vector<std::vector<std::string>> data;
    std::ifstream file(filename);

    if (!file.is_open()) {
        throw std::runtime_error("Could not open file. Check whether the location is correct.");
    }

    std::string line;
    while (getline(file, line)) {
        std::vector<std::string> row;
        std::stringstream lineStream(line);
        std::string cell;

        while (getline(lineStream, cell, ',')) {
            row.push_back(cell);
        }
        if (data.size()) {
            if (data[0].size() != row.size()) {
                int errorRow = data.size() + 1;
                throw std::runtime_error("The number of columns in the CSV file is not consistent at row: " + std::to_string(errorRow));
            }
        }
        data.push_back(row);
    }
    file.close();
    return data;
}

/**
 * @brief Converts a vector of strings to an Eigen vector.
 *
 * This method converts a subset of a vector of strings to an Eigen vector by converting each string to a double.
 *
 * @param data Vector of strings.
 * @param start Start index for conversion.
 * @param end End index for conversion.
 * @return Converted Eigen vector.
 */
Eigen::VectorXd Generated_dataSet_BA::convertToVector(std::vector<std::string> data, int start, int end)
{
#ifndef NDEBUG
    assert(end - start > 0 && end <= data.size() && start >= 0);
#endif
    Eigen::VectorXd vec(end - start);
    for (int i = start; i < end; i++) {
        vec[i - start] = std::stod(data[i]);
    }
    return vec;
}

/**
 * @brief Writes results to a CSV file.
 *
 * This method writes the provided data and column names to a specified CSV file.
 *
 * @param filepath Path to the CSV file.
 * @param columnNames Vector of column names.
 * @param data 2D vector of data to write.
 */
void Generated_dataSet_BA::writeResultsCsv(const std::string& filepath, const std::vector<std::string>& columnNames, const std::vector<std::vector<double>>& data)
{
    for (const auto& row : data) {
        if (row.size() != columnNames.size()) {
            std::cerr << "Error: All data rows must have the same number of elements as there are column names." << std::endl;
            return;
        }
    }

    std::ofstream file(filepath, std::ios::out | std::ios::trunc);

    if (!file.is_open()) {
        std::cerr << "Failed to open or create the file." << std::endl;
        return;
    }

    bool firstColumnName = true;
    for (const auto& name : columnNames) {
        if (!firstColumnName) file << ",";
        file << name;
        firstColumnName = false;
    }
    file << "\n";

    for (const auto& row : data) {
        bool firstColumn = true;
        for (double value : row) {
            if (!firstColumn) file << ",";
            file << value;
            firstColumn = false;
        }
        file << "\n";
    }

    file.close();
    std::cout << "File written successfully." << std::endl;
}

/**
 * @brief Asks the user for the file location.
 *
 * This method prompts the user to enter the location of a specified file and returns the input.
 *
 * @param fileName Name of the file.
 * @return The file location provided by the user.
 */
std::string Generated_dataSet_BA::askFileLocation(const std::string& fileName)
{
    return askQuestion("Enter the location for the file \"" + fileName + "\": ");
}

/**
 * @brief Asks the user a question and retrieves the answer.
 *
 * This method displays a question to the user, retrieves the input, and returns it.
 *
 * @param question The question to ask.
 * @return The answer provided by the user.
 */
std::string Generated_dataSet_BA::askQuestion(const std::string& question)
{
    std::cout << question << std::endl;
    std::string answer;
    std::getline(std::cin, answer);
    return answer;
}

/**
 * @brief Creates and configures the optimizer settings.
 *
 * This method sets up the optimizer settings for bundle adjustment, including parameters for the algorithm,
 * robust kernel, verbosity level, and more.
 *
 * @return A unique pointer to the configured optimizer settings.
 */
std::unique_ptr<OptimizerSettings> Generated_dataSet_BA::makeSettings()
{
    std::unique_ptr<OptimizerSettings> settings = std::make_unique<OptimizerSettings>();

    settings->vertexType1Size = 6;
    settings->vertexType2Size = 3;
    settings->edgeSize = 2;

    settings->Algorithm = "LM";
    settings->MaxIterations = 100;
    settings->maxRepeats = 10;

    settings->Robust = true;
    settings->RobustParameter = 10;
    settings->RobustType = "Huber";

    settings->VerbosityLvl = 2;
    settings->DebugMode = true;

    settings->isSparse = true;
    settings->isMalginalized = true;
    settings->isFixedAvailable = false;
    settings->needRemove = false;

    std::shared_ptr<ReprojectionBase> reprojection = std::make_shared<Reprojection_generated_dataset>();
    settings->setReprojection(reprojection);
    settings->setParameterUpdate(nullptr);
    settings->setRobustKernel(nullptr);
    return settings;
}

Generated_dataSet_BA::Generated_dataSet_BA()
{
    timer = std::make_shared<Timer>();

    timer->setVariables({ "Total", "Initialize", "Optimize", "Finalize", "Jacobian", "Error", "Hessian", "bVector", "update" });
}

Generated_dataSet_BA::~Generated_dataSet_BA()
{
}

/**
 * @brief Performs bundle adjustment using the provided dataset.
 *
 * This method reads pose, landmark, and projection data from CSV files, sets up the optimizer,
 * and runs the optimization process. It configures the optimizer with the necessary settings,
 * adds vertices and edges, and performs optimization.
 */
void Generated_dataSet_BA::do_BA()
{
    std::string poseFile = "D:/University/C++/Source/repos/temp/Other/data/poses.csv";
    std::string landmarkFile = "D:/University/C++/Source/repos/temp/Other/data/landmarks.csv";
    std::string projectionsFile = "D:/University/C++/Source/repos/temp/Other/data/projection.csv";

    std::string answer = askQuestion("Data in installation directory? (y/n)");

    if (answer == "y" || answer == "Y") {
        poseFile = "poses.csv";
        landmarkFile = "landmarks.csv";
        projectionsFile = "projection.csv";
    }
    else if (answer == "n" || answer == "N") {
        poseFile = askFileLocation("poses.csv");
        landmarkFile = askFileLocation("landmarks.csv");
        projectionsFile = askFileLocation("projection.csv");
    }
    else if (answer != "d" && answer != "D") {
        std::cerr << "Invalid input. Exiting..." << std::endl;
        return;
    }

    std::cout << "Reading poseFile from: " << poseFile << std::endl;
    auto poseData = readCSV(poseFile);
    std::cout << "Reading landmarkFile from: " << landmarkFile << std::endl;
    auto landmarkData = readCSV(landmarkFile);
    std::cout << "Reading projectionsFile from: " << projectionsFile << std::endl;
    auto projectionData = readCSV(projectionsFile);

    std::cout << "Data read successfully!\n\n" << std::endl;

    size_t num_cameras = poseData.size() - 1;
    size_t num_landmarks = landmarkData.size() - 1;
    size_t num_observations = projectionData.size() - 1;

    Eigen::VectorXd initialError(num_cameras * 6 + num_landmarks * 3);
    Eigen::VectorXd finalError(num_cameras * 6 + num_landmarks * 3);

    for (int i = 1; i < num_cameras + 1; i++) {
        initialError.segment(6 * (i - 1), 6) = convertToVector(poseData[i], 7, 13) - convertToVector(poseData[i], 1, 7);
        finalError.segment(6 * (i - 1), 6) = -convertToVector(poseData[i], 1, 7);
    }

    for (int i = 1; i < num_landmarks + 1; i++) {
        initialError.segment(6 * num_cameras + 3 * (i - 1), 3) = convertToVector(landmarkData[i], 4, 7) - convertToVector(landmarkData[i], 1, 4);
        finalError.segment(6 * num_cameras + 3 * (i - 1), 3) = -convertToVector(landmarkData[i], 1, 4);
    }

    std::cout << "Number of cameras: " << num_cameras << " | ";
    std::cout << "Number of landmarks: " << num_landmarks << "\n";
    std::cout << "Number of observations: " << num_observations << "\n";

    OptimizerFactoryProducer factoryProducer;

    std::unique_ptr<OptimizerFactory> optimizerFactory = factoryProducer.getOptimizerFactory("Orb New");

    std::unique_ptr<OptimizerSettings> settings = makeSettings();

    std::unique_ptr<OptimizerBase> optimizerAlgorithm = optimizerFactory->getOptimizer(std::move(settings));

    optimizerAlgorithm->setTimer(timer);

    for (int i = 1; i < num_cameras + 1; i++) {
        std::shared_ptr<Eigen::VectorXd> temp = std::make_shared<Eigen::VectorXd>(convertToVector(poseData[i], 7, 13));
        optimizerAlgorithm->addVertex_type1(std::stoi(poseData[i][0]), std::move(temp), false);
    }

    for (int i = 1; i < num_landmarks + 1; i++) {
        std::shared_ptr<Eigen::VectorXd> temp = std::make_shared<Eigen::VectorXd>(convertToVector(landmarkData[i], 4, 7));
        optimizerAlgorithm->addVertex_type2(std::stoi(landmarkData[i][0]) + num_cameras, std::move(temp), false);
    }

    for (int i = 1; i < num_observations + 1; i++) {
        std::shared_ptr<Eigen::VectorXd> tempObservation = std::make_shared<Eigen::VectorXd>(convertToVector(projectionData[i], 5, 7));
        std::shared_ptr<Eigen::VectorXd> tempSigma = std::make_shared<Eigen::VectorXd>(convertToVector(projectionData[i], 3, 5));
        optimizerAlgorithm->addEdge(std::stoi(projectionData[i][0]), std::stoi(projectionData[i][1]), std::stoi(projectionData[i][2]) + num_cameras,
            std::move(tempObservation), std::move(tempSigma));
    }

    poseData.clear();
    landmarkData.clear();
    projectionData.clear();

    timer->startClock("Total");
    timer->startClock("Initialize");

    optimizerAlgorithm->initialize();

    timer->pauseClock("Initialize");
    timer->startClock("Optimize");

    optimizerAlgorithm->Optimize(100);

    timer->pauseClock("Optimize");

    optimizerAlgorithm->finalize();

    timer->startClock("Finalize");

    double Total_seconds = timer->getTimeInSeconds("Total");
    double initialize_seconds = timer->getTimeInSeconds("Initialize");
    double optimize_seconds = timer->getTimeInSeconds("Optimize");
    double finalize_seconds = timer->getTimeInSeconds("Finalize");
    std::cout << "\n                                             =======Time Report=======        \n";

#ifdef GET_TIME
    std::cout << "Jacobian Time : " << timer->getTimeInSeconds("Jacobian") * 100 / optimize_seconds <<
        "% | error_time: " << timer->getTimeInSeconds("Error") * 100 / optimize_seconds << "% | update_time: " << timer->getTimeInSeconds("Update") * 100 / optimize_seconds <<
        "% | revert_time: " << timer->getTimeInSeconds("revert") * 100 / optimize_seconds << "% | solve_time: " << timer->getTimeInSeconds("solve") * 100 / optimize_seconds << "% | Robust_time: " << timer->getTimeInSeconds("Robust") * 100 / optimize_seconds << "% \n";
    std::cout << "A_time: " << timer->getTimeInSeconds("Hessian") * 100 / optimize_seconds << "% | b_time: " << timer->getTimeInSeconds("bVector") * 100 / optimize_seconds << "% | calculation_time: " << timer->getTimeInSeconds("Calculations") * 100 / optimize_seconds << "% \n";
#endif

    std::cout << "\nElapsed time: " << Total_seconds << " s" << " | Optimize time : " << optimize_seconds << " s" << "\nInitialize Percentage : " << initialize_seconds * 100 / Total_seconds << " % | Optimize Percentage : " << optimize_seconds * 100 / Total_seconds << " % | Finalize Percentage : " << finalize_seconds * 100 / Total_seconds << " % \n";

    for (int i = 1; i < num_cameras + 1; i++) {
        finalError.segment(6 * (i - 1), 6) += *optimizerAlgorithm->getVertex_type1_Parameters(i - 1);
    }

    for (int i = 1; i < num_landmarks + 1; i++) {
        finalError.segment(6 * num_cameras + 3 * (i - 1), 3) += *optimizerAlgorithm->getVertex_type2_Parameters(i - 1 + num_cameras);
    }

    std::cout << "\n\nInitial error norm: " << initialError.norm() << " Initial max Error: " << initialError.maxCoeff() << std::endl;

    optimizerAlgorithm.reset();

    std::cout << "Final error norm: " << finalError.norm() << " Final max Error: " << finalError.maxCoeff() << std::endl;
    std::cout << "Optimization finished!" << std::endl;
}

#include "Generated_dataSet_BA.h"

StringPairVectors Generated_dataSet_BA::readCsvColumns(const std::string& filePath)
{
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

std::vector<std::vector<std::string>> Generated_dataSet_BA::readCSV(const std::string& filename)
{
    std::vector<std::vector<std::string>> data; // Vector to store the entire CSV data
    std::ifstream file(filename); // Open the CSV file

    // Check if the file is open
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file. check whether the location is correct");
    }

    std::string line;
    // Read data line by line
    while (getline(file, line)) {
        std::vector<std::string> row;
        std::stringstream lineStream(line);
        std::string cell;

        // Split the line into cells
        while (getline(lineStream, cell, ',')) {
            row.push_back(cell); // Add each cell to the row
        }
        if (data.size()) {
            if (data[0].size() != row.size()) {
                int errorRow = data.size() + 1;
                throw std::runtime_error("The number of columns in the csv file is not consistent at row: " + std::to_string(errorRow));
            }
        }
        data.push_back(row); // Add the row to the main vector
    }
    file.close(); // Close the file
    return data; // Return the CSV data
}

Eigen::VectorXd Generated_dataSet_BA::convertToVector(std::vector<std::string> data, int start, int end)
{

#ifndef NDEBUG
    assert(end - start > 0 && end <= data.size() && start >= 0);
#endif // !NDEBUG
    Eigen::VectorXd vec(end - start);
    for (int i = start; i < end; i++) {
        vec[i - start] = std::stod(data[i]);
    }
    return vec;
}

void Generated_dataSet_BA::writeResultsCsv(const std::string& filePath, const std::string& name, const std::vector<double>& results)
{
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

std::unique_ptr<OptimizerSettings> Generated_dataSet_BA::makeSettings()
{
    std::unique_ptr<OptimizerSettings> settings = std::make_unique<OptimizerSettings>();

    settings->vertexType1Size = 6; // 6 parameters for camera pose
    settings->vertexType2Size = 3; // 3 parameters for landmark position
    settings->edgeSize = 2; // 2 observations per edge

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
    settings->setParameterUpdate(nullptr); // no custom parameter update(custom parameter update is needed when parameters are not closed under addition)
    settings->setRobustKernel(nullptr); // no custom robust kernel
    return settings;
}

Generated_dataSet_BA::Generated_dataSet_BA()
{
    timer = std::make_shared<Timer>();

    timer->setVariables({ "Total", "Initialize", "Optimize", "Finalize", "Jacobian", "Error", "Hessian", "bVector", "update"});
}
Generated_dataSet_BA::~Generated_dataSet_BA()
{
}
void Generated_dataSet_BA::do_BA()
{
    //read data - change destination to data/poses.csv, data/landmarks.csv, data/projection.csv
    std::string poseFile = "D:/University/C++/Source/repos/Bundle_Adjustment/Other/data/poses.csv";
    std::string landmarkFile = "D:/University/C++/Source/repos/Bundle_Adjustment/Other/data/landmarks.csv";
    std::string projectionsFile = "D:/University/C++/Source/repos/Bundle_Adjustment/Other/data/projection.csv";


    auto poseData = readCSV(poseFile);
    auto landmarkData = readCSV(landmarkFile);
    auto projectionData = readCSV(projectionsFile);

    size_t num_cameras = poseData.size() - 1; // -1 for the header
    size_t num_landmarks = landmarkData.size() - 1;
    size_t num_observations = projectionData.size() - 1;

    Eigen::VectorXd initialError(num_cameras * 6 + num_landmarks * 3);
    Eigen::VectorXd finalError(num_cameras * 6 + num_landmarks * 3);

    for (int i = 1; i < num_cameras + 1; i++) { // +1 for the header
        initialError.segment(6 * (i - 1), 6) = convertToVector(poseData[i], 7, 13) - convertToVector(poseData[i], 1, 7);
        finalError.segment(6 * (i - 1), 6) = - convertToVector(poseData[i], 1, 7);
    }

    for (int i = 1; i < num_landmarks + 1; i++) { // +1 for the header
        initialError.segment(6 * num_cameras + 3 * (i - 1), 3) = convertToVector(landmarkData[i], 4, 7) - convertToVector(landmarkData[i], 1, 4);
        finalError.segment(6 * num_cameras + 3 * (i - 1), 3) = - convertToVector(landmarkData[i], 1, 4);
    }

    std::cout << "Number of cameras: " << num_cameras<<" | ";
    std::cout << "Number of landmarks: " << num_landmarks<<"\n";
    std::cout << "Number of observations: " << num_observations << "\n";

	//set optimizerAlgorithm
    OptimizerFactoryProducer factoryProducer;

    std::unique_ptr<OptimizerFactory> optimizerFactory = factoryProducer.getOptimizerFactory("Orb New");

    std::unique_ptr<OptimizerSettings> settings = makeSettings();

    std::unique_ptr<OptimizerBase> optimizerAlgorithm = optimizerFactory->getOptimizer(std::move(settings));

    optimizerAlgorithm->setTimer(timer);

    //add vertices and edges to optimizerAlgorithm
    for (int i = 1; i < num_cameras + 1; i++) { // +1 for the header
        //can use shared pointer inside a map(id,parameters) to access the parameters after optimization
        std::shared_ptr<Eigen::VectorXd> temp = std::make_shared<Eigen::VectorXd>(convertToVector(poseData[i], 7, 13));
        optimizerAlgorithm->addVertex_type1(std::stoi(poseData[i][0]), std::move(temp), false);
    }

    for (int i = 1; i < num_landmarks + 1; i++) { // +1 for the header
        std::shared_ptr<Eigen::VectorXd> temp = std::make_shared<Eigen::VectorXd>(convertToVector(landmarkData[i], 4, 7));
        optimizerAlgorithm->addVertex_type2(std::stoi(landmarkData[i][0]) + num_cameras, std::move(temp), false);
    }

    for (int i = 1; i < num_observations + 1; i++) { // +1 for the header
        //optimizerAlgorithm->addEdge(std::stoi(projectionData[i][0]), convertToVector(projectionData[i], 5, 7), convertToVector(projectionData[i], 3, 5),
            //std::stoi(projectionData[i][1]), std::stoi(projectionData[i][2]) + num_cameras);
        std::shared_ptr<Eigen::VectorXd> tempObservation = std::make_shared<Eigen::VectorXd>(convertToVector(projectionData[i], 5, 7));
        std::shared_ptr<Eigen::VectorXd> tempSigma = std::make_shared<Eigen::VectorXd>(convertToVector(projectionData[i], 3, 5));
        optimizerAlgorithm->addEdge(std::stoi(projectionData[i][0]), std::stoi(projectionData[i][1]), std::stoi(projectionData[i][2]) + num_cameras
            ,std::move(tempObservation),std::move(tempSigma));
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

    //measure the time for all the optimization process
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
    //std::cout << "Tt time: " << optimizerAlgorithm->calculation_time * 100 / optimize_seconds << "% \n";
#endif // GET_TIME
    //print the elapsed time in seconds
    std::cout << "\nElapsed time: " << Total_seconds << " s" << " | Optimize time : " << optimize_seconds << " s" << "\nInitialize Precentage : " << initialize_seconds * 100 / Total_seconds << " % | Optimize Precentage : " << optimize_seconds * 100 / Total_seconds << " % | Finalize Precentage : " << finalize_seconds * 100 / Total_seconds << " % \n";

    //std::cout << "\nAfter optimization:" << std::endl;


    for (int i = 1; i < num_cameras + 1; i++) { // +1 for the header
        finalError.segment(6 * (i - 1), 6) += *optimizerAlgorithm->getVertex_type1_Parameters(i - 1);
    }

    for (int i = 1; i < num_landmarks + 1; i++) { // +1 for the header
        finalError.segment(6 * num_cameras + 3 * (i - 1), 3) += *optimizerAlgorithm->getVertex_type2_Parameters(i - 1 + num_cameras);
    }

    std::cout << "\n\nInitial error norm: " << initialError.norm() << " Initial max Error: " << initialError.maxCoeff() << std::endl;

    optimizerAlgorithm.reset();


    std::cout << "Final error norm: " << finalError.norm() << " Final max Error: " << finalError.maxCoeff() << std::endl;
    std::cout << "Optimization finished!" << std::endl;
	
}

#include "BaseDataStructure.h"
#include <fstream>
#include <iostream>

/**
 * @brief This data structure is used to extract data from edges.
 */
struct EdgeData
{
    int vertex1Location; ///< Location of the Pose Vertex
    int vertex2Location; ///< Location of the Landmark Vertex
    int edgeLocation;    ///< Location of the Edge
    
    std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> Jacobian1Block; ///< Jacobian 1 Block
    std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> Jacobian2Block; ///< Jacobian 2 Block
    std::shared_ptr<Eigen::Map<Eigen::VectorXd>> residualVec;      ///< Residual Vector
    std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> WBlock;         ///< W Block
};

/**
 * @brief This data structure is used to store the Hessian block data.
 */
struct HessianBlock
{
    int rowLocation;   ///< Row location of the Hessian block
    int columnLocation;///< Column location of the Hessian block
    std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> ABlock; ///< A Block (part of the Hessian)
};

/**
 * @brief This data structure is used to store the b vector data.
 */
struct bVec
{
    int vertexLocation; ///< Location of the vertex in the b vector
    std::shared_ptr<Eigen::Map<Eigen::VectorXd>> bBlock; ///< b Block (vector)
};

/**
 * @brief This class stores and manages edge data.
 */
class EdgeDataDirectory : public BaseDataStructure 
{
    std::vector<EdgeData> edgeData; ///< Vector of EdgeData

public:

    /**
     * @brief Default constructor.
     */
    EdgeDataDirectory() : BaseDataStructure() {}

    /**
     * @brief Destructor.
     */
    ~EdgeDataDirectory() {}

    /**
     * @brief Initializes the data structure.
     */
    void initialize() override
    {
        edgeData.clear();
    };

    /**
     * @brief Finalizes the data structure.
     */
    void finalize() override
    {
        edgeData.clear();
    };

    /**
     * @brief Sets the edge data.
     * @param vertex1Location Location of the first vertex.
     * @param vertex2Location Location of the second vertex.
     * @param edgeLocation Location of the edge.
     * @param Jacobian1Block Shared pointer to the Jacobian 1 block.
     * @param Jacobian2Block Shared pointer to the Jacobian 2 block.
     * @param residualVec Shared pointer to the residual vector.
     * @param WBlock Shared pointer to the W block.
     */
    void setEdgeData(int vertex1Location, 
                     int vertex2Location, int edgeLocation, 
                     std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> Jacobian1Block, 
                     std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> Jacobian2Block, 
                     std::shared_ptr<Eigen::Map<Eigen::VectorXd>> residualVec,
                     std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> WBlock)
    {
        if (!Jacobian1Block || !Jacobian2Block || !residualVec || !WBlock) {
            throw std::invalid_argument("Null pointer passed to setEdgeData");
        }
        
        EdgeData edgeDataObject;
        edgeDataObject.vertex1Location = vertex1Location;
        edgeDataObject.vertex2Location = vertex2Location;
        edgeDataObject.edgeLocation = edgeLocation;
        edgeDataObject.Jacobian1Block = Jacobian1Block;
        edgeDataObject.Jacobian2Block = Jacobian2Block;
        edgeDataObject.residualVec = residualVec;
        edgeDataObject.WBlock = WBlock;
        edgeData.push_back(edgeDataObject);
    }

    /**
     * @brief Adds an EdgeData object to the directory.
     * @param edgeDataObject EdgeData object containing edge information.
     */
    void setEdgeData(EdgeData edgeDataObject)
    {
        if (!edgeDataObject.Jacobian1Block || !edgeDataObject.Jacobian2Block || 
            !edgeDataObject.residualVec || !edgeDataObject.WBlock) {
            throw std::invalid_argument("Null pointer passed in EdgeData object");
        }
        
        edgeData.push_back(edgeDataObject);
    }

    /**
     * @brief Prints edge data to a specified text file.
     * @param filename The name of the file to which edge data is written.
     */
    void printEdgeData(const std::string& filename)
    {
        std::ofstream outFile(filename);
        if (!outFile) {
            std::cerr << "Error opening file: " << filename << std::endl;
            return;
        }

        for (const auto& edge : edgeData) {
            outFile << "Vertex1 Location: " << edge.vertex1Location << "\n"
                    << "Vertex2 Location: " << edge.vertex2Location << "\n"
                    << "Edge Location: " << edge.edgeLocation << "\n"
                    << "Jacobian1 Block:\n" << *edge.Jacobian1Block << "\n"
                    << "Jacobian2 Block:\n" << *edge.Jacobian2Block << "\n"
                    << "Residual Vector:\n" << *edge.residualVec << "\n"
                    << "W Block:\n" << *edge.WBlock << "\n"
                    << "--------------------------------------\n";
        }

        outFile.close();
    }

};

/**
 * @brief This class stores and manages Hessian blocks.
 */
class HessianBlockDirectory : public BaseDataStructure
{
    std::vector<HessianBlock> hessianBlocks; ///< Vector of Hessian blocks

public:

    /**
     * @brief Default constructor.
     */
    HessianBlockDirectory() : BaseDataStructure() {}

    /**
     * @brief Destructor.
     */
    ~HessianBlockDirectory() {}

    /**
     * @brief Initializes the data structure.
     */
    void initialize() override
    {
        hessianBlocks.clear();
    };

    /**
     * @brief Finalizes the data structure.
     */
    void finalize() override
    {
        hessianBlocks.clear();
    };

    /**
     * @brief Sets the Hessian block data.
     * @param rowLocation Location of the row.
     * @param columnLocation Location of the column.
     * @param ABlock Shared pointer to the A block.
     */
    void setHessianBlock(int rowLocation, int columnLocation, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> ABlock)
    {
        if (!ABlock) {
            throw std::invalid_argument("Null pointer passed to setHessianBlock");
        }

        HessianBlock hessianBlock;
        hessianBlock.rowLocation = rowLocation;
        hessianBlock.columnLocation = columnLocation;
        hessianBlock.ABlock = ABlock;
        hessianBlocks.push_back(hessianBlock);
    }

    /**
     * @brief Adds a HessianBlock object to the directory.
     * @param hessianBlock HessianBlock object containing Hessian information.
     */
    void setHessianBlock(HessianBlock hessianBlock)
    {
        if (!hessianBlock.ABlock) {
            throw std::invalid_argument("Null pointer passed in HessianBlock object");
        }

        hessianBlocks.push_back(hessianBlock);
    }

    /**
     * @brief Prints Hessian blocks to a specified text file.
     * @param filename The name of the file to which Hessian blocks are written.
     */
    void printHessianBlocks(const std::string& filename)
    {
        std::ofstream outFile(filename);
        if (!outFile) {
            std::cerr << "Error opening file: " << filename << std::endl;
            return;
        }

        for (const auto& block : hessianBlocks) {
            outFile << "Row Location: " << block.rowLocation << "\n"
                    << "Column Location: " << block.columnLocation << "\n"
                    << "A Block:\n" << *block.ABlock << "\n"
                    << "--------------------------------------\n";
        }

        outFile.close();
    }

};

/**
 * @brief This class stores and manages b vector blocks.
 */
class bVectorDirectory : public BaseDataStructure
{
    std::vector<bVec> bVectors; ///< Vector of b vectors

public:

    /**
     * @brief Default constructor.
     */
    bVectorDirectory() : BaseDataStructure() {}

    /**
     * @brief Destructor.
     */
    ~bVectorDirectory() {}

    /**
     * @brief Initializes the data structure.
     */
    void initialize() override
    {
        bVectors.clear();
    };

    /**
     * @brief Finalizes the data structure.
     */
    void finalize() override
    {
        bVectors.clear();
    };

    /**
     * @brief Sets the b vector data.
     * @param vertexLocation Location of the vertex.
     * @param bBlock Shared pointer to the b block.
     */
    void setbVector(int vertexLocation, std::shared_ptr<Eigen::Map<Eigen::VectorXd>> bBlock)
    {
        if (!bBlock) {
            throw std::invalid_argument("Null pointer passed to setbVector");
        }

        bVec bVector;
        bVector.vertexLocation = vertexLocation;
        bVector.bBlock = bBlock;
        bVectors.push_back(bVector);
    }

    /**
     * @brief Adds a bVec object to the directory.
     * @param bVector bVec object containing b vector information.
     */
    void setbVector(bVec bVector)
    {
        if (!bVector.bBlock) {
            throw std::invalid_argument("Null pointer passed in bVec object");
        }

        bVectors.push_back(bVector);
    }

    /**
     * @brief Prints b vectors to a specified text file.
     * @param filename The name of the file to which b vectors are written.
     */
    void printbVectors(const std::string& filename)
    {
        std::ofstream outFile(filename);
        if (!outFile) {
            std::cerr << "Error opening file: " << filename << std::endl;
            return;
        }

        for (const auto& bVector : bVectors) {
            outFile << "Vertex Location: " << bVector.vertexLocation << "\n"
                    << "b Vector:\n" << *bVector.bBlock << "\n"
                    << "--------------------------------------\n";
        }

        outFile.close();
    }

};

/**
 * @brief This class extracts and manages optimization-related data.
 */
class DataExtractor : public BaseDataStructure
{
    std::shared_ptr<EdgeDataDirectory> edgeDataDirectory; ///< Edge Data Directory
    std::shared_ptr<HessianBlockDirectory> hessianBlockDirectory; ///< Hessian Block Directory
    std::shared_ptr<bVectorDirectory> bVectorDirectoryObject; ///< b Vector Directory

public:

    /**
     * @brief Default constructor.
     */
    DataExtractor() : BaseDataStructure() {
        edgeDataDirectory = std::make_shared<EdgeDataDirectory>();
        hessianBlockDirectory = std::make_shared<HessianBlockDirectory>();
        bVectorDirectoryObject = std::make_shared<bVectorDirectory>();
    }

    /**
     * @brief Destructor.
     */
    ~DataExtractor() {}

    /**
     * @brief Initializes the data structure.
     */
    void initialize() override
    {
    };

    /**
     * @brief Finalizes the data structure.
     */
    void finalize() override
    {
        edgeDataDirectory->finalize();
        hessianBlockDirectory->finalize();
        bVectorDirectoryObject->finalize();
    };

    /**
     * @brief Sets the edge data.
     * @param vertex1Location Location of the first vertex.
     * @param vertex2Location Location of the second vertex.
     * @param edgeLocation Location of the edge.
     * @param Jacobian1Block Shared pointer to the Jacobian 1 block.
     * @param Jacobian2Block Shared pointer to the Jacobian 2 block.
     * @param residualVec Shared pointer to the residual vector.
     * @param WBlock Shared pointer to the W block.
     */
    void setEdgeData(int vertex1Location, 
                     int vertex2Location, int edgeLocation, 
                     std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> Jacobian1Block, 
                     std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> Jacobian2Block, 
                     std::shared_ptr<Eigen::Map<Eigen::VectorXd>> residualVec,
                     std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> WBlock)
    {
        if (!Jacobian1Block || !Jacobian2Block || !residualVec || !WBlock) {
            throw std::invalid_argument("Null pointer passed to setEdgeData in DataExtractor");
        }
        
        edgeDataDirectory->setEdgeData(vertex1Location, vertex2Location, edgeLocation, Jacobian1Block, Jacobian2Block, residualVec, WBlock);
    }

    /**
     * @brief Sets the Hessian block data.
     * @param rowLocation Location of the row.
     * @param columnLocation Location of the column.
     * @param ABlock Shared pointer to the A block.
     */
    void setHessianBlock(int rowLocation, int columnLocation, std::shared_ptr<Eigen::Block<Eigen::MatrixXd>> ABlock)
    {
        if (!ABlock) {
            throw std::invalid_argument("Null pointer passed to setHessianBlock in DataExtractor");
        }
        
        hessianBlockDirectory->setHessianBlock(rowLocation, columnLocation, ABlock);
    }

    /**
     * @brief Sets the b vector data.
     * @param vertexLocation Location of the vertex.
     * @param bBlock Shared pointer to the b block.
     */
    void setbVector(int vertexLocation, std::shared_ptr<Eigen::Map<Eigen::VectorXd>> bBlock)
    {
        if (!bBlock) {
            throw std::invalid_argument("Null pointer passed to setbVector in DataExtractor");
        }
        
        bVectorDirectoryObject->setbVector(vertexLocation, bBlock);
    }

    /**
     * @brief Prints all data to separate text files for the current iteration.
     * @param iteration The current iteration number (used to generate filenames).
     */
    void printAllData(int iteration)
    {
        edgeDataDirectory->printEdgeData("edges_" + std::to_string(iteration) + ".txt");
        hessianBlockDirectory->printHessianBlocks("hessian_blocks_" + std::to_string(iteration) + ".txt");
        bVectorDirectoryObject->printbVectors("b_vectors_" + std::to_string(iteration) + ".txt");
    }

};

/**
 * @brief This class represents a basic optimizer using the DataExtractor.
 */
class Optimizer
{
    DataExtractor dataExtractor; ///< DataExtractor object to manage and log data.

public:
    /**
     * @brief Default constructor for the Optimizer.
     */
    Optimizer() : dataExtractor() {}

    /**
     * @brief Initializes the optimizer and data extractor.
     */
    void initialize()
    {
        dataExtractor.initialize();
    }

    /**
     * @brief Runs the optimization process for a specified number of iterations.
     * @param numIterations Number of iterations to run the optimizer.
     */
    void optimize(int numIterations)
    {
        for (int iteration = 0; iteration < numIterations; ++iteration)
        {
            // Example: Simulate retrieving data from the optimizer's state
            int vertex1Location = 0; // Example value
            int vertex2Location = 1; // Example value
            int edgeLocation = iteration; // Using iteration as example edge location

            Eigen::MatrixXd exampleMatrix = Eigen::MatrixXd::Random(3, 3);
            Eigen::VectorXd exampleVector = Eigen::VectorXd::Random(3);

            // Wrap Eigen objects in shared_ptrs to use with DataExtractor
            auto jacobian1Block = std::make_shared<Eigen::Block<Eigen::MatrixXd>>(exampleMatrix.block(0, 0, 2, 2));
            auto jacobian2Block = std::make_shared<Eigen::Block<Eigen::MatrixXd>>(exampleMatrix.block(1, 1, 2, 2));
            auto residualVec = std::make_shared<Eigen::Map<Eigen::VectorXd>>(exampleVector.data(), exampleVector.size());
            auto wBlock = std::make_shared<Eigen::Block<Eigen::MatrixXd>>(exampleMatrix.block(0, 0, 3, 3));

            auto aBlock = std::make_shared<Eigen::Block<Eigen::MatrixXd>>(exampleMatrix.block(0, 0, 3, 3));
            auto bBlock = std::make_shared<Eigen::Map<Eigen::VectorXd>>(exampleVector.data(), exampleVector.size());

            // Set data in DataExtractor
            dataExtractor.setEdgeData(vertex1Location, vertex2Location, edgeLocation, jacobian1Block, jacobian2Block, residualVec, wBlock);
            dataExtractor.setHessianBlock(vertex1Location, vertex2Location, aBlock);
            dataExtractor.setbVector(vertex1Location, bBlock);

            // Perform an optimization step (this is a placeholder for the actual optimization logic)
            std::cout << "Performing optimization step " << iteration << std::endl;

            // Print the data to text files after each iteration
            dataExtractor.printAllData(iteration);
        }
    }

    /**
     * @brief Finalizes the optimizer and data extractor.
     */
    void finalize()
    {
        dataExtractor.finalize();
    }
};

/**
 * @brief Main function to execute the optimization process.
 * @return Exit status of the program.
 */
int main()
{
    Optimizer optimizer;

    // Initialize the optimizer
    optimizer.initialize();

    // Run the optimization for a given number of iterations
    int numIterations = 10;
    optimizer.optimize(numIterations);

    // Finalize the optimizer
    optimizer.finalize();

    return 0;
}

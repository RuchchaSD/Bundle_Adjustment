@mainpage Bundle Adjustment
# Bundle Adjustment

## Overview

This project implements a bundle adjustment system used in SLAM (Simultaneous Localization and Mapping) applications. The system is designed to optimize the positions of landmarks and the poses of sensors to minimize the reprojection error of observed landmarks. The implementation is modular, using various classes to manage different aspects of the optimization process.

## Main Components

### Timer

The `Timer` class is a utility for measuring time intervals. It provides functionalities to start, pause, reset, and get elapsed time in seconds, milliseconds, and minutes.

### Propertise

The `Propertise` structure holds properties and settings for SLAM optimization. It includes parameters such as the number of edges, vertex sizes, algorithm settings, and flags for various states (e.g., debug mode, sparse mode).

### Vertex

The `Vertex` class represents a vertex in the optimization problem. It holds parameters and provides methods to get and set the vertex's ID, global ID, fixed status, parameters, and parameter maps.

### Edge

The `Edge` class represents an edge in the optimization problem. It connects two vertices and includes observations, covariance, sigma, and their respective maps.

### BaseDataStructure

The `BaseDataStructure` class is a base class for data structures used in SLAM optimization. It provides methods to initialize and finalize the data structure, and to set and get properties and timers.

### ReprojectionBase

The `ReprojectionBase` class is a base class for reprojection operations. It provides a pure virtual function to reproject estimates, which derived classes must implement.

### ParameterUpdateBase

The `ParameterUpdateBase` class is a base class for parameter update strategies. It provides a virtual function to update parameters based on provided update values.

### RobustKernelBase

The `RobustKernelBase` class is a base class for robust kernel functions. It includes methods to initialize and finalize the kernel, apply it to residuals and Jacobians, and get the weights vector.

### HuberKernel

The `HuberKernel` class derives from `RobustKernelBase` and implements the Huber robust kernel function, which calculates weights based on residual values.

### InformationMatrixBase

The `InformationMatrixBase` class handles information matrices in SLAM. It provides methods to retrieve information values and their square roots.

### InformationMatrixCompressed

The `InformationMatrixCompressed` class derives from `InformationMatrixBase` and manages a compressed information matrix. It includes methods to initialize, finalize, update, and print the information matrix.

### ParameterVectorBase

The `ParameterVectorBase` class is a base class for handling parameters in SLAM optimization. It includes methods to set and get parameter vectors, update vectors, and perform parameter updates.

### JacobianBase

The `JacobianBase` class is a base class for Jacobian computation. It includes methods to set robust kernels, information vectors, reprojection strategies, and update the Jacobian.

### ResidualsBase

The `ResidualsBase` class is a base class for residuals computation. It provides methods to set robust kernels, information vectors, reprojection strategies, and update residuals.

### HessianBase

The `HessianBase` class is a base class for handling Hessian matrices. It includes methods to update the Hessian and retrieve maximum diagonal values.

### SolverBase

The `SolverBase` class is a base class for solvers in SLAM optimization. It provides a pure virtual function to solve the optimization problem.

### SolverOrb

The `SolverOrb` class derives from `SolverBase` and handles the optimization for ORB-SLAM. It includes methods to set Hessian blocks, B vectors, update vectors, and solve the system.

### SolverMarginalized

The `SolverMarginalized` class derives from `SolverOrb` and handles marginalized optimization in ORB-SLAM. It includes additional methods to manage B inverses, Y blocks, and active structures.

## Dependencies

This project depends on the C++ Standard Library and the Eigen 3 library for linear algebra operations.

## Getting Started

To get started with this project, ensure that you have the Eigen 3 library installed. You can then compile and run the project using a C++ compiler.

## License

//Todo

## Authors

This project is developed and maintained by Ruchith Wickramanayake.



#include "optimiser.h"

optimiser::optimiser(modelTranslator *_modelTranslator, physicsSimulator *_physicsSimulator, fileHandler *_yamlReader, differentiator *_differentiator){
    std::cout << "initialised optimiser \n";
    activeModelTranslator = _modelTranslator;
    activePhysicsSimulator = _physicsSimulator;
    activeYamlReader = _yamlReader;
    activeDifferentiator = _differentiator;

    dof = activeModelTranslator->dof;
    num_ctrl = activeModelTranslator->num_ctrl;
}

bool optimiser::checkForConvergence(double oldCost, double newCost){
    double costGrad = (oldCost - newCost)/newCost;

    if(costGrad < epsConverge){
        return true;
    }
    return false;
}

void optimiser::setupTestingExtras(int _trajecNumber, int _interpMethod, int _keyPointsMethod, int _minN, bool _approxBackwardsPass){
    currentTrajecNumber = _trajecNumber;
    interpMethod = _interpMethod;
    keyPointsMethod = _keyPointsMethod;
    min_interval = _minN;
    approximate_backwardsPass = _approxBackwardsPass;
}

void optimiser::returnOptimisationData(double &_optTime, double &_costReduction, double &_avgPercentageDerivs, double &_avgTimeGettingDerivs, int &_numIterations){

    for(int i = 0; i < numDerivsPerIter.size(); i++){
        avgNumDerivs += numDerivsPerIter[i];
    }
    avgNumDerivs = avgNumDerivs/numDerivsPerIter.size();

    for(int i = 0; i < timeDerivsPerIter.size(); i++){
        avgTimePerDerivs += timeDerivsPerIter[i];
    }
    avgTimePerDerivs = avgTimePerDerivs/timeDerivsPerIter.size();

    _optTime = optTime;
    _costReduction = costReduction;
    _avgPercentageDerivs = ((double) avgNumDerivs / (double)numberOfTotalDerivs) * 100.0f;
    _avgTimeGettingDerivs = avgTimePerDerivs;
    _numIterations = numIterationsForConvergence;
}

// ------------------------------------------- STEP 1 FUNCTIONS (GET DERIVATIVES) ----------------------------------------------
void optimiser::generateDerivatives(){
    auto start = high_resolution_clock::now();
    // STEP 1 - Linearise dynamics and calculate first + second order cost derivatives for current trajectory
    // generate the dynamics evaluation waypoints
    std::vector<std::vector<int>> keyPoints = generateKeyPoints(X_old, U_old);

    // Calculate derivatives via finite differnecing / analytically for cost functions if available
    if(keyPointsMethod != iterative_error){
        auto start_fd_time = high_resolution_clock::now();
        getDerivativesAtSpecifiedIndices(keyPoints);
        auto stop_fd_time = high_resolution_clock::now();
        auto duration_fd_time = duration_cast<microseconds>(stop_fd_time - start_fd_time);
    }
    else{
        getCostDerivs();
    }

//    cout << "A[horizonLength - 1] " << A[horizonLength - 1] << endl;

//    activeYamlReader->generalSaveMatrices(l_x, "l_x");
//    activeYamlReader->generalSaveMatrices(l_xx, "l_xx");

//    cout << "l_x: " << l_x[0] << endl;
//    cout << "l_x[horizonLength]: " << l_x[horizonLength] << endl;
//
//    cout << "l_u: " << l_u[0] << endl;
//    cout << "l_u[horizonLength]: " << l_u[horizonLength] << endl;
//
//    cout << "l_uu: " << l_uu[0] << endl;
//    cout << "l_uu[horizonLength]: " << l_uu[horizonLength] << endl;
//
//    cout << "l_xx: " << l_xx[0] << endl;
//    cout << "l_xx[horizonLength]: " << l_xx[horizonLength] << endl;


    // Interpolate derivatvies as required for a full set of derivatives
//    if((keyPointsMethod == setInterval) && (min_interval == 1)){
//
//    }
//    else{
//        interpolateDerivatives(keyPoints);
//    }

    // time how long it takes to inteprolate
    auto testStart = high_resolution_clock::now();
    interpolateDerivatives(keyPoints);
    auto testStop = high_resolution_clock::now();
    auto testDuration = duration_cast<microseconds>(testStop - testStart);

    int totalNumColumnsDerivs = 0;
    for(int i = 0; i < keyPoints.size(); i++){
        totalNumColumnsDerivs += keyPoints[i].size();

    }

    cout << "percentage of derivs calculated: " << (totalNumColumnsDerivs / numberOfTotalDerivs) * 100.0f << endl;

//    cout << "A[0] " << A[0] << endl;
//    cout << "A[1] " << A[1] << endl;
//
//    cout << "B[0] " << B[0] << endl;
//    cout << "B[1] " << B[1] << endl;
//    cout << "B[2] " << B[2] << endl;
//    cout << "B[horizon - 1] " << B[horizonLength - 1] << endl;
//    cout << "B[hori] " << B[horizonLength] << endl;
//
//    // Same for A
//    cout << "A[0] " << A[0] << endl;
//    cout << "A[1] " << A[1] << endl;
//    cout << "A[2] " << A[2] << endl;
//    cout << "A[horizon - 1] " << A[horizonLength - 1] << endl;
//    cout << "A[hori] " << A[horizonLength] << endl;


//        A.resize(initControls.size());
//        activeYamlReader->saveTrajecInfomation(A, B, X_old, U_old, activeModelTranslator->modelName, 1);

    if(filteringMatrices){
        filterMatrices();
    }

//        A.resize(initControls.size());
//        activeYamlReader->saveTrajecInfomation(A, B, X_old, U_old, activeModelTranslator->modelName, 2);

    auto stop = high_resolution_clock::now();
    auto linDuration = duration_cast<microseconds>(stop - start);
    time_getDerivs_ms = linDuration.count() / 1000.0f;

    numDerivsPerIter.push_back(totalNumColumnsDerivs);
    timeDerivsPerIter.push_back(linDuration.count() / 1000000.0f);
}

std::vector<std::vector<int>> optimiser::generateKeyPoints(std::vector<MatrixXd> trajecStates, std::vector<MatrixXd> trajecControls){
    // Loop through the trajectory and decide what indices should be evaluated via finite differencing
    std::vector<std::vector<int>> evaluationWaypoints;
    std::vector<int> oneRow;
    for(int i = 0; i < dof; i++){
        oneRow.push_back(i);
    }
    evaluationWaypoints.push_back(oneRow);

    if(keyPointsMethod == setInterval){
        for(int i = 1; i < horizonLength - 1; i++){

            if(i % min_interval == 0){
                std::vector<int> oneRow;
                for(int j = 0; j < dof; j++){
                    oneRow.push_back(j);
                }
                evaluationWaypoints.push_back(oneRow);
            }
            else{
                std::vector<int> oneRow;
                evaluationWaypoints.push_back(oneRow);
            }
        }

//        int numEvals = horizonLength / min_interval;
//        for(int i = 1; i < numEvals; i++){
//            if(i * min_interval < horizonLength){
//                std::vector<int> oneRow(dof, i * min_interval);
//                evaluationWaypoints.push_back(oneRow);
//            }
//        }

    }
    else if(keyPointsMethod == adaptive_jerk){
        std::vector<MatrixXd> jerkProfile = generateJerkProfile();
        evaluationWaypoints = generateKeyPointsAdaptive(jerkProfile);

    }
    else if(keyPointsMethod == adaptive_accel){
        std::vector<MatrixXd> accelProfile = generateAccelProfile();
        evaluationWaypoints = generateKeyPointsAdaptive(accelProfile);
    }
    else if(keyPointsMethod == iterative_error){
        computedKeyPoints.clear();
        activeDifferentiator->initModelForFiniteDifferencing();
        evaluationWaypoints = generateKeyPointsIteratively();
        activeDifferentiator->resetModelAfterFiniteDifferencing();

    }
    else{
        std::cout << "ERROR: keyPointsMethod not recognised \n";
    }

    //Check if last element in evaluationWaypoints is horizonLength - 1
    if(evaluationWaypoints.back().size() != 0){
        if(evaluationWaypoints.back()[0] != horizonLength - 1){
            std::vector<int> oneRow;
            for(int i = 0; i < dof; i++){
                oneRow.push_back(i);
            }
            evaluationWaypoints.push_back(oneRow);
        }
    }
    else{
        std::vector<int> oneRow;
        for(int i = 0; i < dof; i++){
            oneRow.push_back(i);
        }
        evaluationWaypoints.push_back(oneRow);
    }


    return evaluationWaypoints;
}

std::vector<std::vector<int>> optimiser::generateKeyPointsAdaptive(std::vector<MatrixXd> trajecProfile){
    std::vector<std::vector<int>> evaluationWaypoints;

    std::vector<int> oneRow;
    for(int i = 0; i < dof; i++){
        oneRow.push_back(i);
    }
    evaluationWaypoints.push_back(oneRow);

    int lastIndices[dof];
    for(int i = 0; i < dof; i++){
        lastIndices[i] = 0;
    }

    for(int i = 0; i < trajecProfile.size(); i++){
        std::vector<int> currentRow;
        int dofCounter = 0;

        // Loop through all possible robots
        for(int j = 0; j < activeModelTranslator->myStateVector.robots.size(); j++) {
            // Loop through all joints of each robot
            for(int k = 0; k < activeModelTranslator->myStateVector.robots[j].jointNames.size(); k++) {

                // Check if lastUpdate is above minN
                if((i - lastIndices[dofCounter]) >= min_interval) {
                    // Check if the jerk is above the threshold
                    if (trajecProfile[i](dofCounter, 0) > activeModelTranslator->myStateVector.robots[j].jointJerkThresholds[k]) {
                        currentRow.push_back(dofCounter);
                        lastIndices[dofCounter] = i;
                    }
                }

                // Check if lastUpdate is above maxN
                if((i - lastIndices[dofCounter]) >= max_interval){
                    currentRow.push_back(dofCounter);
                    lastIndices[dofCounter] = i;
                }

                dofCounter++;
            }
        }

        // Loop through all bodies in simulation state
        for(int j = 0; j < activeModelTranslator->myStateVector.bodiesStates.size(); j++) {
            //Loop through linear states
            for (int k = 0; k < 3; k++) {
                // If we are considering this dof in the problem
                if (activeModelTranslator->myStateVector.bodiesStates[j].activeLinearDOF[k]) {

                    // Check if lastUpdate is above minN
                    if((i - lastIndices[dofCounter]) >= min_interval){
                        if (trajecProfile[i](dofCounter, 0) >
                            activeModelTranslator->myStateVector.bodiesStates[j].linearJerkThreshold[k]) {

                            currentRow.push_back(dofCounter);
                            lastIndices[dofCounter] = i;
                        }
                    }

                    // Check if lastUpdates is above maxN
                    if((i - lastIndices[dofCounter]) >= max_interval){
                        currentRow.push_back(dofCounter);
                        lastIndices[dofCounter] = i;
                    }

                    dofCounter++;
                }
            }

            //Loop through angular states
            for (int k = 0; k < 3; k++) {
                // Check if we are considering this dof in the problem
                if (activeModelTranslator->myStateVector.bodiesStates[j].activeAngularDOF[k]) {

                    // Check if lastUpdate is above minN
                    if((i - lastIndices[dofCounter]) >= min_interval){
                        if (trajecProfile[i](dofCounter, 0) >
                            activeModelTranslator->myStateVector.bodiesStates[j].angularJerkThreshold[k]) {

                            currentRow.push_back(dofCounter);
                            lastIndices[dofCounter] = i;
                        }
                    }

                    // Check if lastUpdate is above maxN
                    if((i - lastIndices[dofCounter]) >= max_interval){
                        currentRow.push_back(dofCounter);
                        lastIndices[dofCounter] = i;
                    }

                    dofCounter++;
                }
            }
        }

        // Always append current row, even if empty.
        evaluationWaypoints.push_back(currentRow);
    }

    return evaluationWaypoints;
}

std::vector<std::vector<int>> optimiser::generateKeyPointsIteratively(){
    std::vector<std::vector<int>> evalPoints;
    bool binsComplete[dof];
    std::vector<indexTuple> indexTuples;
    int startIndex = 0;
    int endIndex = horizonLength - 1;

    // Initialise variables
    for(int i = 0; i < dof; i++){
        binsComplete[i] = false;
        computedKeyPoints.push_back(std::vector<int>());

    }

    for(int i = 0; i < horizonLength; i++){

        evalPoints.push_back(std::vector<int>());
    }

    // Loop through all dofs in the system
    #pragma omp parallel for
    for(int i = 0; i < dof; i++){
//        std::cout << "---------------------  Generating key points for dof --------------------------------- " << i << std::endl;
        std::vector<indexTuple> listOfIndicesCheck;
        indexTuple initialTuple;
        initialTuple.startIndex = startIndex;
        initialTuple.endIndex = endIndex;
        listOfIndicesCheck.push_back(initialTuple);

        std::vector<indexTuple> subListIndices;
        std::vector<int> subListWithMidpoints;

        while(!binsComplete[i]){
            bool allChecksComplete = true;

            for(int j = 0; j < listOfIndicesCheck.size(); j++) {

                int midIndex = (listOfIndicesCheck[j].startIndex + listOfIndicesCheck[j].endIndex) / 2;
//                cout << "index tuple: " << listOfIndicesCheck[j].startIndex << " " << listOfIndicesCheck[j].endIndex << endl;
                bool approximationGood = checkDoFColumnError(listOfIndicesCheck[j], i);

                if (!approximationGood) {
                    allChecksComplete = false;
                    indexTuple tuple1;
                    tuple1.startIndex = listOfIndicesCheck[j].startIndex;
                    tuple1.endIndex = midIndex;
                    indexTuple tuple2;
                    tuple2.startIndex = midIndex;
                    tuple2.endIndex = listOfIndicesCheck[j].endIndex;
                    subListIndices.push_back(tuple1);
                    subListIndices.push_back(tuple2);
                } else {
                    subListWithMidpoints.push_back(listOfIndicesCheck[j].startIndex);
                    subListWithMidpoints.push_back(midIndex);
                    subListWithMidpoints.push_back(listOfIndicesCheck[j].endIndex);
                }
            }

            if(allChecksComplete){
                binsComplete[i] = true;
//                evalPoints.push_back(subListWithMidpoints);

                subListWithMidpoints.clear();
            }

            listOfIndicesCheck = subListIndices;
//        for(int k = 0; k < listOfIndicesCheck.size(); k++){
//            cout << listOfIndicesCheck[k].startIndex << " " << listOfIndicesCheck[k].endIndex << "\n";
//        }
            subListIndices.clear();

        }
    }

    // Loop over the horizon
    for(int i = 0; i < horizonLength; i++){
        // Loop over the dofs
        for(int j = 0; j < dof; j++){
            // Loop over the computed key points per dof
//            cout << "Computed key points for dof " << j << " are: ";
            for(int k = 0; k < computedKeyPoints[j].size(); k++){
                // If the current index is a computed key point
//                cout << computedKeyPoints[j][k] << " ";

                if(i == computedKeyPoints[j][k]){
//                    cout << "Adding key point " << i << " to dof " << j << "\n";
                    evalPoints[i].push_back(j);
                }

            }
//            cout << "\n";
        }
    }

    // Sort list into order
    for(int i = 0; i < dof; i++){
        std::sort(evalPoints[i].begin(), evalPoints[i].end());
    }

    // Remove duplicates
    for(int i = 0; i < dof; i++){
        evalPoints[i].erase(std::unique(evalPoints[i].begin(), evalPoints[i].end()), evalPoints[i].end());
    }


    // Print the key points
//    for(int i = 0; i < horizonLength; i++){
//        cout << "Key points for index " << i << ":";
//        for(int j = 0; j < evalPoints[i].size(); j++){
//            cout << " " << evalPoints[i][j];
//        }
//        cout << "\n";
//
//    }

    return evalPoints;
}

bool optimiser::checkDoFColumnError(indexTuple indices, int dofIndex){

    MatrixXd midColumnsApprox[2];
    for(int i = 0; i < 2; i++){
        midColumnsApprox[i] = MatrixXd::Zero(activeModelTranslator->stateVectorSize, 1);
    }

//    if(dofIndex == 0){
//        cout << "start index: " << indices.startIndex << " mid index: " << " end index: " << indices.endIndex << "\n";
//    }


    int midIndex = (indices.startIndex + indices.endIndex) / 2;
    if((indices.endIndex - indices.startIndex) < min_interval){
        return true;
    }

    MatrixXd blank1, blank2, blank3, blank4;

    bool startIndexExists = false;
    bool midIndexExists = false;
    bool endIndexExists = false;

    int counterTooSmall = 0;
    int counterTooLarge = 0;

    for(int i = 0; i < computedKeyPoints[dofIndex].size(); i++){
        if(computedKeyPoints[dofIndex][i] == indices.startIndex){
            startIndexExists = true;
        }

        if(computedKeyPoints[dofIndex][i] == midIndex){
            midIndexExists = true;
        }

        if(computedKeyPoints[dofIndex][i] == indices.endIndex){
            endIndexExists = true;
        }
    }

    std::vector<int> cols;
    cols.push_back(dofIndex);

    if(!startIndexExists){
//        if(dofIndex == 0){
//            cout << "startIndex is being calced" << "\n";
//        }
        activeDifferentiator->getDerivatives(A[indices.startIndex], B[indices.startIndex], cols, blank1, blank2, blank3, blank4, false, indices.startIndex, false);
        computedKeyPoints[dofIndex].push_back(indices.startIndex);
    }

    if(!midIndexExists){
//        if(dofIndex == 0){
//            cout << "midIndex is being calced" << "\n";
//        }
        activeDifferentiator->getDerivatives(A[midIndex], B[midIndex], cols, blank1, blank2, blank3, blank4, false, midIndex, false);
        computedKeyPoints[dofIndex].push_back(midIndex);
    }

    if(!endIndexExists){
//        if(dofIndex == 0){
//            cout << "endIndex is being calced" << "\n";
//        }
        activeDifferentiator->getDerivatives(A[indices.endIndex], B[indices.endIndex], cols, blank1, blank2, blank3, blank4, false, indices.endIndex, false);
        computedKeyPoints[dofIndex].push_back(indices.endIndex);
    }

    midColumnsApprox[0] = (A[indices.startIndex].block(0, dofIndex, dof*2, 1) + A[indices.endIndex].block(0, dofIndex, dof*2, 1)) / 2;
    midColumnsApprox[1] = (A[indices.startIndex].block(0, dofIndex + dof, dof*2, 1) + A[indices.endIndex].block(0, dofIndex + dof, dof*2, 1)) / 2;

//    if(dofIndex == 0){
//        cout << "matrixMidTrue: \n" << A[midIndex].block(0, dofIndex, dof*2, 1) << "\n";
////        cout << "matrixMidTrue: \n" << A[midIndex] << "\n";
//        cout << "matrixMidApprox: \n" << midColumnsApprox[0] << "\n";
//    }


    bool approximationGood = false;
    int dof = activeModelTranslator->dof;
    double errorSum = 0.0f;
    int counter = 0;

    for(int i = 0; i < 2; i++){
        int A_col_indices[2] = {dofIndex, dofIndex + dof};
        for(int j = dof; j < activeModelTranslator->stateVectorSize; j++){
            double sqDiff = pow((A[midIndex](j, A_col_indices[i]) - midColumnsApprox[i](j, 0)),2);

            if(sqDiff > 0.5){
                sqDiff = 0.0f;
                counterTooLarge++;
            }
            else if(sqDiff < 0.00001){
                sqDiff = 0.0f;
                counterTooSmall++;
            }
            else{

                counter++;
            }
            errorSum += sqDiff;

        }
//        cout << "errorSum: " << errorSum << "\n";
    }

    double averageError;
    if(counter > 0){
        averageError = errorSum / counter;
    }
    else{
        averageError = 0.0f;
    }
//    if(dofIndex == 0){
//        cout << "average error: " << averageError << "\n";
//    }

//    cout << "average error: " << averageError << "\n";
//    cout << "num valid: " << counter << "\n";
//    cout << "num too small: " << counterTooSmall << "\n";
//    cout << "num too large: " << counterTooLarge << "\n";
    if(averageError < 0.005){
        approximationGood = true;
    }
    else{
//        cout << "matrix mid approx" << matrixMidApprox << "\n";
//        cout << "matrix mid true" << A[midIndex] << "\n";
    }

//    if(counter == 0){
//        cout << "start index: " << indices.startIndex << " mid index: " << midIndex << " end index: " << indices.endIndex << "\n";
//        cout << "matrix mid approx" << matrixMidApprox << "\n";
//        cout << "matrix mid true" << A[midIndex] << "\n";
//    }

    return approximationGood;
}

bool optimiser::checkOneMatrixError(indexTuple indices){
//    MatrixXd matrixMidApprox(activeModelTranslator->stateVectorSize, activeModelTranslator->stateVectorSize);
//
//    int midIndex = (indices.startIndex + indices.endIndex) / 2;
//    if((indices.endIndex - indices.startIndex) < 5){
//        return true;
//    }
//
//    MatrixXd blank1, blank2, blank3, blank4;
//
//    bool startIndexExists;
//    bool midIndexExists;
//    bool endIndexExists;
//
//    int counterTooSmall = 0;
//    int counterTooLarge = 0;
//
////    cout << "start index: " << indices.startIndex << " mid index: " << midIndex << " end index: " << indices.endIndex << "\n";
//
//    for(int i = 0; i < computedKeyPoints.size(); i++){
//        if(computedKeyPoints[i] == indices.startIndex){
//            startIndexExists = true;
//        }
//
//        if(computedKeyPoints[i] == midIndex){
//            midIndexExists = true;
//        }
//
//        if(computedKeyPoints[i] == indices.endIndex){
//            endIndexExists = true;
//        }
//    }
//
//    if(!startIndexExists){
//        activeDifferentiator->getDerivatives(A[indices.startIndex], B[indices.startIndex], blank1, blank2, blank3, blank4, false, indices.startIndex, false);
//        computedKeyPoints.push_back(indices.startIndex);
//    }
//
//    if(!midIndexExists){
//        activeDifferentiator->getDerivatives(A[midIndex], B[midIndex], blank1, blank2, blank3, blank4, false, midIndex, false);
//        computedKeyPoints.push_back(midIndex);
//    }
//
//    if(!endIndexExists){
//        activeDifferentiator->getDerivatives(A[indices.endIndex], B[indices.endIndex], blank1, blank2, blank3, blank4, false, indices.endIndex, false);
//        computedKeyPoints.push_back(indices.endIndex);
//    }
//
//    matrixMidApprox = (A[indices.startIndex] + A[indices.endIndex]) / 2;
//
////    cout << "matrixMidTrue: \n" << matrixMidTrue << "\n";
////    cout << "matrixMidApprox: \n" << matrixMidApprox << "\n";
//
//    bool approximationGood = false;
//    int dof = activeModelTranslator->dof;
//    double errorSum = 0.0f;
//    int counter = 0;
//
//    for(int i = dof; i < activeModelTranslator->stateVectorSize; i++){
//        for(int j = 0; j < activeModelTranslator->stateVectorSize; j++){
//            double sqDiff = pow((A[midIndex](i, j) - matrixMidApprox(i, j)),2);
//
//            if(sqDiff > 0.5){
//                sqDiff = 0.0f;
//                counterTooLarge++;
//            }
//            else if(sqDiff < 0.00001){
//                sqDiff = 0.0f;
//                counterTooSmall++;
//            }
//            else{
//
//                counter++;
//            }
//            errorSum += sqDiff;
//        }
//    }
//
//    double averageError;
//    if(counter > 0){
//        averageError = errorSum / counter;
//    }
//    else{
//        averageError = 0.0f;
//    }
//
////    cout << "average error: " << averageError << "\n";
////    cout << "num valid: " << counter << "\n";
////    cout << "num too small: " << counterTooSmall << "\n";
////    cout << "num too large: " << counterTooLarge << "\n";
//    if(averageError < 0.002){
//        approximationGood = true;
//    }
//    else{
////        cout << "matrix mid approx" << matrixMidApprox << "\n";
////        cout << "matrix mid true" << A[midIndex] << "\n";
//    }
//
////    if(counter == 0){
////        cout << "start index: " << indices.startIndex << " mid index: " << midIndex << " end index: " << indices.endIndex << "\n";
////        cout << "matrix mid approx" << matrixMidApprox << "\n";
////        cout << "matrix mid true" << A[midIndex] << "\n";
////    }
//
//    return approximationGood;
}

void optimiser::getCostDerivs(){
#pragma omp parallel for
    for(int i = 0; i < horizonLength; i++){
        if(i == 0){
            activeModelTranslator->costDerivatives(X_old[0], U_old[0], X_old[0], U_old[0], l_x[i], l_xx[i], l_u[i], l_uu[i], false);
        }
        else{
            activeModelTranslator->costDerivatives(X_old[i], U_old[i], X_old[i-1], U_old[i-1], l_x[i], l_xx[i], l_u[i], l_uu[i], false);
        }

    }

    activeModelTranslator->costDerivatives(X_old[horizonLength], U_old[horizonLength - 1], X_old[horizonLength - 1], U_old[horizonLength - 1],
                                           l_x[horizonLength], l_xx[horizonLength], l_u[horizonLength], l_uu[horizonLength], true);
}

void optimiser::getDerivativesAtSpecifiedIndices(std::vector<std::vector<int>> keyPoints){

    activeDifferentiator->initModelForFiniteDifferencing();

    #pragma omp parallel for
    for(int i = 0; i < keyPoints.size(); i++){

        int timeIndex = i;
        std::vector<int> columns = keyPoints[i];
//        cout << "columns.size(): index " << i << ": " << columns.size() << "\n";
        // If there are no keypoints at a certain data index, dont compute derivatives
        if(columns.size() == 0){
            continue;
        }
        bool terminal = false;
        if(timeIndex == horizonLength - 1){
            terminal = true;
        }
        activeDifferentiator->getDerivatives(A[timeIndex], B[timeIndex], columns, l_x[timeIndex], l_u[timeIndex], l_xx[timeIndex], l_uu[timeIndex], activeYamlReader->costDerivsFD, timeIndex, terminal);

    }

//    activeYamlReader->generalSaveMatrices(l_x, "l_x_fd");
//    activeYamlReader->generalSaveMatrices(l_xx, "l_xx_fd");

    activeDifferentiator->resetModelAfterFiniteDifferencing();

    if(!activeYamlReader->costDerivsFD){
        #pragma omp parallel for
        for(int i = 0; i < horizonLength; i++){
            if(i == 0){
                activeModelTranslator->costDerivatives(X_old[0], U_old[0], X_old[0], U_old[0], l_x[i], l_xx[i], l_u[i], l_uu[i], false);
            }
            else{
                activeModelTranslator->costDerivatives(X_old[i], U_old[i], X_old[i-1], U_old[i-1], l_x[i], l_xx[i], l_u[i], l_uu[i], false);
            }
        }
    }

    activeModelTranslator->costDerivatives(X_old[horizonLength], U_old[horizonLength - 1], X_old[horizonLength - 1], U_old[horizonLength - 1],
                                           l_x[horizonLength], l_xx[horizonLength], l_u[horizonLength], l_uu[horizonLength], true);
}

void optimiser::interpolateDerivatives(std::vector<std::vector<int>> keyPoints){
    // Create an array to track startIndices of next interpolation for each dof
    int startIndices[dof];
    for(int i = 0; i < dof; i++){
        startIndices[i] = 0;
    }

    // Loop through all the time indices - can skip the first
    // index as we preload the first index as the start index for all dofs.
    for(int t = 1; t < horizonLength; t++){
        // Loop through all the dofs
        for(int i = 0; i < dof; i++){
            // Check the current vector at that time segment for the current dof
            std::vector<int> columns = keyPoints[t];

            // If there are no keypoints, continue onto second run of the loop
            if(columns.size() == 0){
                continue;
            }

            for(int j = 0; j < columns.size(); j++){

                MatrixXd startB;
                MatrixXd endB;
                MatrixXd addB;
                // If there is a match, interpolate between the start index and the current index
                // For the given columns
                if(i == columns[j]){
                    MatrixXd startACol1 = A[startIndices[i]].block(dof, i, dof, 1);
                    MatrixXd endACol1 = A[t].block(dof, i, dof, 1);
                    MatrixXd addACol1 = (endACol1 - startACol1) / (t - startIndices[i]);

                    // Same again for column 2 which is dof + i
                    MatrixXd startACol2 = A[startIndices[i]].block(dof, i + dof, dof, 1);
                    MatrixXd endACol2 = A[t].block(dof, i + dof, dof, 1);
                    MatrixXd addACol2 = (endACol2 - startACol2) / (t - startIndices[i]);


                    if(i < num_ctrl){
                        startB = B[startIndices[i]].block(dof, i, dof, 1);
                        endB = B[t].block(dof, i, dof, 1);
                        addB = (endB - startB) / (t - startIndices[i]);
                    }

                    for(int k = startIndices[i]; k < t; k++){
                        A[k].block(dof, i, dof, 1) = startACol1 + ((k - startIndices[i]) * addACol1);

                        if(i < num_ctrl){
                            B[k].block(dof, i, dof, 1) = startB + ((k - startIndices[i]) * addB);
                        }

                        A[k].block(dof, i + dof, dof, 1) = startACol2 + ((k - startIndices[i]) * addACol2);

                    }

                    startIndices[i] = t;
                }
            }
        }
    }
}

std::vector<MatrixXd> optimiser::generateJerkProfile(){

    MatrixXd jerk(activeModelTranslator->dof, 1);

    MatrixXd state1(activeModelTranslator->stateVectorSize, 1);
    MatrixXd state2(activeModelTranslator->stateVectorSize, 1);
    MatrixXd state3(activeModelTranslator->stateVectorSize, 1);

    std::vector<MatrixXd> jerkProfile;

    for(int i = 0; i < horizonLength - 2; i++){
        state1 = X_old[i];
        state2 = X_old[i + 1];
        state3 = X_old[i + 2];

        MatrixXd accell1 = state2 - state1;
        MatrixXd accell2 = state3 - state2;

        for(int j = 0; j < activeModelTranslator->dof; j++){
            jerk(j, 0) = accell2(j+dof, 0) - accell1(j+dof, 0);
        }

        jerkProfile.push_back(jerk);
    }

    return jerkProfile;
}

std::vector<MatrixXd> optimiser::generateAccelProfile(){

    MatrixXd accel(activeModelTranslator->dof, 1);

    MatrixXd state1(activeModelTranslator->stateVectorSize, 1);
    MatrixXd state2(activeModelTranslator->stateVectorSize, 1);

    std::vector<MatrixXd> accelProfile;

    for(int i = 0; i < horizonLength - 1; i++){
        state1 = X_old[i];
        state2 = X_old[i + 1];

        MatrixXd accelState = state2 - state1;

        for(int j = 0; j < activeModelTranslator->dof; j++){
            accel(j, 0) = accelState(j+dof, 0);
        }

        accelProfile.push_back(accel);
    }

    return accelProfile;
}

void optimiser::filterMatrices(){

    for(int i = dof; i < 2 * dof; i++){
        for(int j = 0; j < 2 * dof; j++){
            std::vector<double> unfiltered;
            std::vector<double> filtered;

            for(int k = 0; k < horizonLength; k++){
                unfiltered.push_back(A[k](i, j));
            }
            filtered = filterIndividualValue(unfiltered);
            for(int k = 0; k < horizonLength; k++){
                A[k](i, j) = filtered[k];
            }
        }
    }
}

std::vector<double> optimiser::filterIndividualValue(std::vector<double> unfiltered){
    double yn1 = unfiltered[0];
    double xn1 = unfiltered[0];
    double a = 0.25;

    std::vector<double> filtered;
    for(int i = 0; i < unfiltered.size(); i++){
        double xn = unfiltered[i];
//        double yn = -0.7254*yn1 + 0.8627*xn + 0.8627*xn1;

        double yn = ((1-a)*yn1) + a*((xn + xn1)/2);

        xn1 = xn;
        yn1 = yn;

        filtered.push_back(yn);
    }
    return filtered;
}



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

void optimiser::setupTestingExtras(int _trajecNumber, int _interpMethod, int _keyPointsMethod, int minN){
    currentTrajecNumber = _trajecNumber;
    interpMethod = _interpMethod;
    keyPointsMethod = _keyPointsMethod;

    min_interval = minN;
}

void optimiser::returnOptimisationData(double &_optTime, double &_costReduction, int &_avgNumDerivs, double &_avgTimeGettingDerivs){

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
    _avgNumDerivs = avgNumDerivs;
    _avgTimeGettingDerivs = avgTimePerDerivs;
}

// ------------------------------------------- STEP 1 FUNCTIONS (GET DERIVATIVES) ----------------------------------------------
void optimiser::generateDerivatives(){
    auto start = high_resolution_clock::now();
    // STEP 1 - Linearise dynamics and calculate first + second order cost derivatives for current trajectory
    // generate the dynamics evaluation waypoints
    std::vector<std::vector<int>> keyPoints = generateKeyPoints(X_old, U_old);
//    for(int j = 0; j < keyPoints.size(); j++){
//        cout << "col " << j << ": " << keyPoints[j].size() << endl;
//    }
//    cout << endl;
//    cout << "keypoints size: " << keyPoints.size() << endl;

    // Calculate derivatives via finite differnecing / analytically for cost functions if available
    if(keyPointsMethod != iterative_error){
        getDerivativesAtSpecifiedIndices(keyPoints);
    }
    else{
        getCostDerivs();
    }

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

    interpolateDerivatives(keyPoints);

    cout << "f_x[0] " << f_x[0] << endl;
    cout << "f_x[1] " << f_x[1] << endl;

//        f_x.resize(initControls.size());
//        activeYamlReader->saveTrajecInfomation(f_x, f_u, X_old, U_old, activeModelTranslator->modelName, 1);

    if(filteringMatrices){
        filterMatrices();
    }

//        f_x.resize(initControls.size());
//        activeYamlReader->saveTrajecInfomation(f_x, f_u, X_old, U_old, activeModelTranslator->modelName, 2);

    auto stop = high_resolution_clock::now();
    auto linDuration = duration_cast<microseconds>(stop - start);
    cout << "number of derivatives calculated via fd: " << keyPoints.size() << endl;
    cout << "calc derivatives took: " << linDuration.count() / 1000000.0f << " s\n";

    numDerivsPerIter.push_back(keyPoints.size());
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
        for(int i = 1; i < horizonLength; i++){

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
//        evaluationWaypoints = generateKeyPointsIteratively();
        activeDifferentiator->resetModelAfterFiniteDifferencing();

    }
    else{
        std::cout << "ERROR: keyPointsMethod not recognised \n";
    }

    //Check if last element in evaluationWaypoints is horizonLength - 1
    if(evaluationWaypoints.back().size() != 0){
        if(evaluationWaypoints.back()[0] != horizonLength - 1){
            std::vector<int> oneRow(dof, horizonLength - 1);
            evaluationWaypoints.push_back(oneRow);
        }
    }
    else{
        std::vector<int> oneRow(dof, horizonLength - 1);
        evaluationWaypoints.push_back(oneRow);
    }


    return evaluationWaypoints;
}

std::vector<std::vector<int>> optimiser::generateKeyPointsAdaptive(std::vector<MatrixXd> trajecProfile){
    std::vector<std::vector<int>> evaluationWaypoints;
    std::vector<int> oneRow(dof, 0);
    evaluationWaypoints.push_back(oneRow);

    int counter = 0;
    for(int i = 0; i < trajecProfile.size(); i++){
        std::vector<int> currentRow;
        counter++;
        int dofCounter = 0;

        if(counter > min_interval){
            bool addKeyPoint = false;
            // Loop through all possible robots
            for(int j = 0; j < activeModelTranslator->myStateVector.robots.size(); j++) {
                bool breakEarly = false;
                // Loop through all joints of each robot
                for(int k = 0; k < activeModelTranslator->myStateVector.robots[j].jointNames.size(); k++) {
                    // Check if the jerk is above the threshold
                    if (trajecProfile[i](dofCounter, 0) > activeModelTranslator->myStateVector.robots[j].jointJerkThresholds[k]) {
                        dofCounter++;
                        currentRow.push_back(dofCounter);
                    }
                }
            }

            // Loop through all bodies in simulation state
            for(int j = 0; j < activeModelTranslator->myStateVector.bodiesStates.size(); j++){
                bool breakEarly = false;
                //Loop through linear states
                for(int k = 0; k < 3; k++){
                    if(activeModelTranslator->myStateVector.bodiesStates[j].activeLinearDOF[k]){
                        if(trajecProfile[i](dofCounter, 0) > activeModelTranslator->myStateVector.bodiesStates[j].linearJerkThreshold[k]){
                            dofCounter++;
                            currentRow.push_back(dofCounter);
                        }
                    }
                }

                //Loop through angular states
                for(int k = 0; k < 3; k++){
                    if(activeModelTranslator->myStateVector.bodiesStates[j].activeAngularDOF[k]){
                        if(trajecProfile[i](dofCounter, 0) > activeModelTranslator->myStateVector.bodiesStates[j].angularJerkThreshold[k]){
                            dofCounter++;
                            currentRow.push_back(dofCounter);
                        }
                    }
                }

            }

            // Always append current row, even if empty.
            evaluationWaypoints.push_back(currentRow);

        }

        if(counter > max_interval){
            evaluationWaypoints.push_back(currentRow);
            counter = 0;
        }
    }

    return evaluationWaypoints;
}

std::vector<int> optimiser::generateKeyPointsIteratively(){
    std::vector<int> evalPoints;
    bool binsComplete = false;
    std::vector<indexTuple> indexTuples;
    int startIndex = 0;
    int endIndex = horizonLength - 1;

    std::vector<indexTuple> listOfIndicesCheck;
    indexTuple initialTuple;
    initialTuple.startIndex = startIndex;
    initialTuple.endIndex = endIndex;

    std::vector<indexTuple> subListIndices;
    std::vector<int> subListWithMidpoints;

    listOfIndicesCheck.push_back(initialTuple);

    while(!binsComplete){
        bool allChecksComplete = true;

        for(int j = 0; j < listOfIndicesCheck.size(); j++) {

            int midIndex = (listOfIndicesCheck[j].startIndex + listOfIndicesCheck[j].endIndex) / 2;
            bool approximationGood = checkOneMatrixError(listOfIndicesCheck[j]);

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
            binsComplete = true;
            for(int k = 0; k < subListWithMidpoints.size(); k++){
                evalPoints.push_back(subListWithMidpoints[k]);
            }

            subListWithMidpoints.clear();
        }

        listOfIndicesCheck = subListIndices;
//        for(int k = 0; k < listOfIndicesCheck.size(); k++){
//            cout << listOfIndicesCheck[k].startIndex << " " << listOfIndicesCheck[k].endIndex << "\n";
//        }
        subListIndices.clear();
    }

    // Sort list into order
    std::sort(evalPoints.begin(), evalPoints.end());

    // Remove duplicates
    evalPoints.erase(std::unique(evalPoints.begin(), evalPoints.end()), evalPoints.end());
//
//    for(int i = 0; i < evalPoints.size(); i++){
//        cout << "evalPoints[" << i << "] = " << evalPoints[i] << "\n";
//    }

    return evalPoints;

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

//    #pragma omp parallel for
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

    for(int i = 0; i < dof; i++){

        int startIndex = 0;
        int endIndex = 0;
        bool pairFound = false;

        // Loop through all the time indices
        for(int t = 0; t < horizonLength; t++){
            std::vector<int> columns = keyPoints[t];

            if(columns.size() == 0){
                continue;
            }

            for(int j = 0; j < columns.size(); j++){
                if(i == columns[j]){
                    endIndex = t;
                    pairFound = true;
                }
            }

            if(pairFound){
                // Interpolate between start index and endIndex for column i and i + dof
                int interpolationSize = endIndex - startIndex;
                MatrixXd startA = A[startIndex].replicate(1, 1);
                MatrixXd startB = B[startIndex].replicate(1, 1);

                MatrixXd endA = A[endIndex].replicate(1, 1);
                MatrixXd endB = B[endIndex].replicate(1, 1);

                MatrixXd addA(2*dof, 2*dof);
                MatrixXd addB(2*dof, num_ctrl);

                for(int k = 0; k < interpolationSize; k++){
                    f_x[startIndex + k].block(0, i, 2*dof, 1) = startA.block(0, i, 2*dof, 1) + (k+1)*(endA.block(0, i, 2*dof, 1) - startA.block(0, i, 2*dof, 1))/interpolationSize;
                    // Same for B
                    if(i < num_ctrl){
                        f_u[startIndex + k].block(0, i, 2*dof, 1) = startB.block(0, i, 2*dof, 1) + (k+1)*(endB.block(0, i, 2*dof, 1) - startB.block(0, i, 2*dof, 1))/interpolationSize;
                    }

                    f_x[startIndex + k].block(0, i+dof, 2*dof, 1) = startA.block(0, i+dof, 2*dof, 1) + (k+1)*(endA.block(0, i+dof, 2*dof, 1) - startA.block(0, i+dof, 2*dof, 1))/interpolationSize;
                }
            }
        }
    }

//    for(int t = 0; t < horizonLength; t++){
//        std::vector<int> columns = keyPoints[t];
//
//        if(columns.size() == 0){
//            continue;
//        }
//
//        for(int i = 0; i < columns.size(); i++){
//
//        }
//    }

//    // ------------------------- DYNAMICS DERIVATIVES --------------------------------
//    // Interpolate all the derivatives that were not calculated via finite differencing
//    for(int t = 0; t < calculatedIndices.size()-1; t++){
//
//        MatrixXd addA(2*dof, 2*dof);
//        MatrixXd addB(2*dof, num_ctrl);
//        int nextInterpolationSize = calculatedIndices[t+1] - calculatedIndices[t];
//        int startIndex = calculatedIndices[t];
//        int endIndex = calculatedIndices[t+1];
//
//        MatrixXd startA = A[startIndex].replicate(1, 1);
//        MatrixXd endA = A[endIndex].replicate(1, 1);
//        MatrixXd diffA = endA - startA;
//        addA = diffA / nextInterpolationSize;
//
//        MatrixXd startB = B[startIndex].replicate(1, 1);
//        MatrixXd endB = B[endIndex].replicate(1, 1);
//        MatrixXd diffB = endB - startB;
//        addB = diffB / nextInterpolationSize;
//
//        // Interpolate A and B matrices
//        for(int i = 0; i < nextInterpolationSize; i++){
//            f_x[startIndex + i] = A[startIndex].replicate(1,1) + (addA * i);
//            f_u[startIndex + i] = B[startIndex].replicate(1,1) + (addB * i);
////            cout << "f_x[" << startIndex + i << "] = " << f_x[startIndex + i] << endl;
//        }
//    }
//
//    f_x[horizonLength - 1] = f_x[horizonLength - 2].replicate(1,1);
//    f_u[horizonLength - 1] = f_u[horizonLength - 2].replicate(1,1);
//
//    // ------------------------- COST DERIVATIVES --------------------------------
//    // Interpolate all the derivatives that were not calculated via finite differencing
//    if(activeYamlReader->costDerivsFD){
//        for(int t = 0; t < calculatedIndices.size()-1; t++){
//
//            MatrixXd addl_x(2*dof, 1);
//            MatrixXd addl_xx(2*dof, 2*dof);
//            MatrixXd addl_u(num_ctrl, 1);
//            MatrixXd addl_uu(num_ctrl, num_ctrl);
//
//
//            int nextInterpolationSize = calculatedIndices[t+1] - calculatedIndices[t];
//            int startIndex = calculatedIndices[t];
//            int endIndex = calculatedIndices[t+1];
//
//
//            MatrixXd startl_x = l_x[startIndex].replicate(1, 1);
//            MatrixXd endl_x = l_x[endIndex].replicate(1, 1);
//            MatrixXd diffl_x = endl_x - startl_x;
//            addl_x = diffl_x / nextInterpolationSize;
//
//            MatrixXd startl_xx = l_xx[startIndex].replicate(1, 1);
//            MatrixXd endl_xx = l_xx[endIndex].replicate(1, 1);
//            MatrixXd diffl_xx = endl_xx - startl_xx;
//            addl_xx = diffl_xx / nextInterpolationSize;
//
//            MatrixXd startl_u = l_u[startIndex].replicate(1, 1);
//            MatrixXd endl_u = l_u[endIndex].replicate(1, 1);
//            MatrixXd diffl_u = endl_u - startl_u;
//            addl_u = diffl_u / nextInterpolationSize;
//
//            MatrixXd startl_uu = l_uu[startIndex].replicate(1, 1);
//            MatrixXd endl_uu = l_uu[endIndex].replicate(1, 1);
//            MatrixXd diffl_uu = endl_uu - startl_uu;
//            addl_uu = diffl_uu / nextInterpolationSize;
//
//
//            // Interpolate A and B matrices
//            for(int i = 0; i < nextInterpolationSize; i++){
//                l_x[startIndex + i] = l_x[startIndex].replicate(1,1) + (addl_x * i);
//                l_u[startIndex + i] = l_u[startIndex].replicate(1,1) + (addl_u * i);
//                l_xx[startIndex + i] = l_xx[startIndex].replicate(1,1) + (addl_xx * i);
//                l_uu[startIndex + i] = l_uu[startIndex].replicate(1,1) + (addl_uu * i);
////            cout << "f_x[" << startIndex + i << "] = " << f_x[startIndex + i] << endl;
//            }
//        }
//    }
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
                unfiltered.push_back(f_x[k](i, j));
            }
            filtered = filterIndividualValue(unfiltered);
            for(int k = 0; k < horizonLength; k++){
                f_x[k](i, j) = filtered[k];
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


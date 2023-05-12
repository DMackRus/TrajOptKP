#include "interpolated_iLQR.h"

interpolatediLQR::interpolatediLQR(modelTranslator *_modelTranslator, physicsSimulator *_physicsSimulator, differentiator *_differentiator, int _maxHorizon, visualizer *_visualizer, fileHandler *_yamlReader) : optimiser(_modelTranslator, _physicsSimulator){

    activeDifferentiator = _differentiator;
    maxHorizon = _maxHorizon;
    activeVisualizer = _visualizer;
    activeYamlReader = _yamlReader;

    // initialise all vectors of matrices
    for(int i = 0; i < maxHorizon; i++){
        // Cost matrices
        l_x.push_back(MatrixXd(2*dof, 1));
        l_xx.push_back(MatrixXd(2*dof, 2*dof));
        l_u.push_back(MatrixXd(num_ctrl, 1));
        l_uu.push_back(MatrixXd(num_ctrl, num_ctrl));

        // Dynamics derivatives matrices
        A.push_back(MatrixXd(2*dof, 2*dof));
        B.push_back(MatrixXd(2*dof, num_ctrl));

        A[i].block(0, 0, dof, dof).setIdentity();
        A[i].block(0, dof, dof, dof).setIdentity();
        A[i].block(0, dof, dof, dof) *= MUJOCO_DT;
        B[i].setZero();

        f_x.push_back(MatrixXd(2*dof, 2*dof));
        f_u.push_back(MatrixXd(2*dof, num_ctrl));

        K.push_back(MatrixXd(num_ctrl, 2*dof));
        k.push_back(MatrixXd(num_ctrl, 1));

        U_old.push_back(MatrixXd(num_ctrl, 1));
        U_new.push_back(MatrixXd(num_ctrl, 1));
        X_old.push_back(MatrixXd(2*dof, 1));
        X_new.push_back(MatrixXd(2*dof, 1));

        std::vector<MatrixXd> U_temp;
        for(int j = 0; j < 8; j++){
            U_temp.push_back(MatrixXd(num_ctrl, 1));
        }

        U_alpha.push_back(U_temp);

    }

    // One more state than control
    X_old.push_back(MatrixXd(2*dof, 1));
    X_new.push_back(MatrixXd(2*dof, 1));

    min_interval = _yamlReader->minInterval;
    max_interval = _yamlReader->maxInterval;
    keyPointsMethod = _yamlReader->keyPointMethod;

    filteringMatrices = activeYamlReader->filtering;

}

double interpolatediLQR::rolloutTrajectory(int initialDataIndex, bool saveStates, std::vector<MatrixXd> initControls){
    double cost = 0.0f;

    if(initialDataIndex != MAIN_DATA_STATE){
        activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, initialDataIndex);
    }

    MatrixXd Xt(activeModelTranslator->stateVectorSize, 1);
    MatrixXd X_last(activeModelTranslator->stateVectorSize, 1);
    MatrixXd Ut(activeModelTranslator->num_ctrl, 1);
    MatrixXd U_last(activeModelTranslator->num_ctrl, 1);

    X_old[0] = activeModelTranslator->returnStateVector(MAIN_DATA_STATE);
    if(activePhysicsSimulator->checkIfDataIndexExists(0)){
        activePhysicsSimulator->copySystemState(0, MAIN_DATA_STATE);
    }
    else{
        activePhysicsSimulator->appendSystemStateToEnd(MAIN_DATA_STATE);
    }

    for(int i = 0; i < initControls.size(); i++){
        // set controls
        activeModelTranslator->setControlVector(initControls[i], MAIN_DATA_STATE);

        // Integrate simulator
        activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);

        // return cost for this state
        Xt = activeModelTranslator->returnStateVector(MAIN_DATA_STATE);
        Ut = activeModelTranslator->returnControlVector(MAIN_DATA_STATE);
        double stateCost;
        
        if(i == initControls.size() - 1){
            stateCost = activeModelTranslator->costFunction(Xt, Ut, X_last, U_last, true);
        }
        else{
            stateCost = activeModelTranslator->costFunction(Xt, Ut, X_last, U_last, false);
        }

        // If required to save states to trajectory tracking, then save state
        if(saveStates){
            X_old[i + 1] = Xt.replicate(1, 1);
            U_old[i] = Ut.replicate(1, 1);
            if(activePhysicsSimulator->checkIfDataIndexExists(i + 1)){
                activePhysicsSimulator->copySystemState(i + 1, MAIN_DATA_STATE);
            }
            else{
                activePhysicsSimulator->appendSystemStateToEnd(MAIN_DATA_STATE);
            }
            
        }
        cost += (stateCost * MUJOCO_DT);
    }

    cout << "cost of initial trajectory was: " << cost << endl;
    initialCost = cost;
    costHistory.push_back(cost);

    return cost;
}

// ------------------------------------------------------------------------------------------------------
//
//  optimise - Optimise a sequence of controls for a given problem
//  @Params:
//  initialDataIndex - the data index of the system state that the optimisation problem should start from
//  initControls - The initial controls for the problem
//  maxIterations - The maximum iterations of the solver before it should return a new set of controls
//  horizonLength - How far into the future the optimiser should look when optimising the controls
//
//  @Returns:
//  optimisedControls - New optimised controls that give a lower cost than the initial controls
//
// -------------------------------------------------------------------------------------------------------
std::vector<MatrixXd> interpolatediLQR::optimise(int initialDataIndex, std::vector<MatrixXd> initControls, int maxIter, int minIter, int _horizonLength){
    cout << " ---------------- optimisation begins -------------------" << endl;
    cout << "Trajectory Number: " << currentTrajecNumber << endl;
    cout << "minN " << min_interval << "  keypointsMethod: " <<  keyPointsMethodsStrings[keyPointsMethod] << "  interpMethod: " << interpMethodsStrings[interpMethod] << endl;

    auto optStart = high_resolution_clock::now();
    
    // - Initialise variables
    std::vector<MatrixXd> optimisedControls;
    horizonLength = _horizonLength;
    lambda = 0.1;
    double oldCost = 0.0f;
    double newCost = 0.0f;
    bool costReducedLastIter = true;

    // Clear data saving variables
    costHistory.clear();
    optTime = 0;
    costReduction = 1.0f;
    numDerivsPerIter.clear();
    timeDerivsPerIter.clear();
    avgNumDerivs = 0;
    avgTimePerDerivs = 0.0f;

    oldCost = rolloutTrajectory(MAIN_DATA_STATE, true, initControls);
    activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, 0);

    // Optimise for a set number of iterations
    for(int i = 0; i < maxIter; i++){

        //STEP 1 - If forwards pass changed the trajectory, -
        if(costReducedLastIter){
            generateDerivatives();
        }

        // STEP 2 - BackwardsPass using the calculated derivatives to calculate an optimal feedback control law
        bool validBackwardsPass = false;
        bool lambdaExit = false;

        auto bp_start = high_resolution_clock::now();
        while(!validBackwardsPass){
            cout << "lamda is: " << lambda << "\n";
            validBackwardsPass = backwardsPass_Quu_reg();
//            K.resize(initControls.size());
//            k.resize(initControls.size());
//            activeYamlReader->generalSaveMatrices(K, "K_normal");
//            activeYamlReader->generalSaveMatrices(k, "k_normal");
//            K.resize(3000);
//            k.resize(3000);
//            validBackwardsPass = backwardsPass_Quu_reg_parallel();
//            K.resize(initControls.size());
//            k.resize(initControls.size());
//            activeYamlReader->generalSaveMatrices(K, "K_parallel");
//            activeYamlReader->generalSaveMatrices(k, "k_parallel");

            if(!validBackwardsPass){
                if(lambda < maxLambda){
                    lambda *= lambdaFactor;
                }
                else{
                    lambdaExit = true;
                    break;

                }
            }
            else{
                if(lambda > minLambda){
                    lambda /= lambdaFactor;
                }
            }
        }
        auto bp_stop = high_resolution_clock::now();
        auto bpDuration = duration_cast<microseconds>(bp_stop - bp_start);
        cout << "bp took: " << bpDuration.count() / 1000000.0f << " s\n";
//        cout << "K[1000] " << K[1000] << endl;

        if(!lambdaExit){
            bool costReduced;
            // STEP 3 - Forwards Pass - use the optimal control feedback law and rollout in simulation and calculate new cost of trajectory
            auto fp_start = high_resolution_clock::now();
//            newCost = forwardsPass(oldCost, costReduced);
            newCost = forwardsPassParallel(oldCost, costReduced);
            auto fp_stop = high_resolution_clock::now();
            auto fpDuration = duration_cast<microseconds>(fp_stop - fp_start);
            cout << "forward pass took: " << fpDuration.count() / 1000000.0f << " s\n";
            cout << " ---------------- new cost is: " << newCost << " -------------------" << endl;

            costHistory.push_back(newCost);

            // STEP 4 - Check for convergence
            bool converged = checkForConvergence(oldCost, newCost);

            if(newCost < oldCost){
                oldCost = newCost;
                costReducedLastIter = true;

            }
            else{
                costReducedLastIter = false;
                if(lambda < maxLambda){
                    lambda *= lambdaFactor;
                    lambda *= lambdaFactor;
                }
            }


            if(converged && (i >= minIter)){
                std::cout << "converged after " << i << " iterations" << std::endl;
                break;
            }
        }
        else{
            cout << "exiting optimisation due to lambda > lambdaMax \n";
            break;
        }
    }

    costReduction = 1 - (newCost / initialCost);
    auto optFinish = high_resolution_clock::now();
    auto optDuration = duration_cast<microseconds>(optFinish - optStart);
    optTime = optDuration.count() / 1000000.0f;
    cout << "optimisation took: " << optTime<< " s\n";
    cout << " ---------------- optimisation complete -------------------" << endl;

    // Load the initial data back into main data
    activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, 0);

    for(int i = 0; i < horizonLength; i++){
        optimisedControls.push_back(U_old[i]);
    }

    if(saveTrajecInfomation){
        //remove std vector elements after size of init controls
        f_x.resize(initControls.size());
        activeYamlReader->saveTrajecInfomation(f_x, f_u, X_old, U_old, activeModelTranslator->modelName, 0);
    }

    if(saveCostHistory){
        cout << "saving cost history \n";
        if(filteringMatrices){
            activeYamlReader->saveCostHistory(costHistory, "filtering", currentTrajecNumber);
        }
        else{
            activeYamlReader->saveCostHistory(costHistory, "no_filtering", currentTrajecNumber);
        }

    }



    return optimisedControls;
}
// ------------------------------------------- STEP 1 FUNCTIONS (GET DERIVATIVES) ----------------------------------------------
void interpolatediLQR::generateDerivatives(){
    auto start = high_resolution_clock::now();
    // STEP 1 - Linearise dynamics and calculate first + second order cost derivatives for current trajectory
    // generate the dynamics evaluation waypoints
    std::vector<int> keyPoints = generateKeyPoints(X_old, U_old);
    for(int j = 0; j < keyPoints.size(); j++){
        cout << keyPoints[j] << " ";
    }
    cout << endl;
    cout << "keypoints size: " << keyPoints.size() << endl;

    // Calculate derivatives via finite differnecing / analytically for cost functions if available
    if(keyPointsMethod != iterative_error){
        getDerivativesAtSpecifiedIndices(keyPoints);
    }
    else{
        getCostDerivs();
    }


    // Interpolate derivatvies as required for a full set of derivatives
    interpolateDerivatives(keyPoints);

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

std::vector<int> interpolatediLQR::generateKeyPoints(std::vector<MatrixXd> trajecStates, std::vector<MatrixXd> trajecControls){
    // Loop through the trajectory and decide what indices should be evaluated via finite differencing
    std::vector<int> evaluationWaypoints;
    evaluationWaypoints.push_back(0);

    if(keyPointsMethod == setInterval){
        int numEvals = horizonLength / min_interval;
        for(int i = 1; i < numEvals; i++){
            if(i * min_interval < horizonLength){
                evaluationWaypoints.push_back(i * min_interval);
            }
        }

    }
    else if(keyPointsMethod == adaptive_jerk){
        int counter = 0;
        std::vector<MatrixXd> jerkProfile = generateJerkProfile();
        for(int i = 0; i < horizonLength; i++){
            counter++;

            if(counter > min_interval){
                for(int j = 0; j < activeModelTranslator->dof; j++){
                    if(jerkProfile.size() > i) {

                        if (jerkProfile[i](j, 0) > 0.002) {
                            evaluationWaypoints.push_back(i);
                            counter = 0;
                            break;
                        }
                    }
                }
            }

            if(counter > max_interval){
                evaluationWaypoints.push_back(i);
                counter = 0;
            }

        }

    }
    else if(keyPointsMethod == adaptive_accel){
        int counter = 0;
        std::vector<MatrixXd> jerkProfile = generateJerkProfile();
        for(int i = 0; i < horizonLength; i++){
            counter++;

            if(counter > min_interval){
                for(int j = 0; j < activeModelTranslator->dof; j++){
                    if(jerkProfile.size() > i) {

                        if (jerkProfile[i](j, 0) > 0.002) {
                            evaluationWaypoints.push_back(i);
                            counter = 0;
                            break;
                        }
                    }
                }
            }

            if(counter > max_interval){
                evaluationWaypoints.push_back(i);
                counter = 0;
            }
        }
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
    if(evaluationWaypoints.back() != horizonLength - 1){
        evaluationWaypoints.push_back(horizonLength - 1);
    }

    return evaluationWaypoints;
}

std::vector<int> interpolatediLQR::generateKeyPointsIteratively(){
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

bool interpolatediLQR::checkOneMatrixError(indexTuple indices){
    MatrixXd matrixMidApprox(activeModelTranslator->stateVectorSize, activeModelTranslator->stateVectorSize);

    int midIndex = (indices.startIndex + indices.endIndex) / 2;
    if((indices.endIndex - indices.startIndex) < 5){
        return true;
    }

    bool startIndexExists;
    bool midIndexExists;
    bool endIndexExists;

    int counterTooSmall = 0;
    int counterTooLarge = 0;

//    cout << "start index: " << indices.startIndex << " mid index: " << midIndex << " end index: " << indices.endIndex << "\n";

    for(int i = 0; i < computedKeyPoints.size(); i++){
        if(computedKeyPoints[i] == indices.startIndex){
            startIndexExists = true;
        }

        if(computedKeyPoints[i] == midIndex){
            midIndexExists = true;
        }

        if(computedKeyPoints[i] == indices.endIndex){
            endIndexExists = true;
        }
    }

    if(!startIndexExists){
        activeDifferentiator->getDerivatives(A[indices.startIndex], B[indices.startIndex], false, indices.startIndex);
        computedKeyPoints.push_back(indices.startIndex);
    }

    if(!midIndexExists){
        activeDifferentiator->getDerivatives(A[midIndex], B[midIndex], false, midIndex);
        computedKeyPoints.push_back(midIndex);
    }

    if(!endIndexExists){
        activeDifferentiator->getDerivatives(A[indices.endIndex], B[indices.endIndex], false, indices.endIndex);
        computedKeyPoints.push_back(indices.endIndex);
    }

    matrixMidApprox = (A[indices.startIndex] + A[indices.endIndex]) / 2;

//    cout << "matrixMidTrue: \n" << matrixMidTrue << "\n";
//    cout << "matrixMidApprox: \n" << matrixMidApprox << "\n";

    bool approximationGood = false;
    int dof = activeModelTranslator->dof;
    double errorSum = 0.0f;
    int counter = 0;

    for(int i = dof; i < activeModelTranslator->stateVectorSize; i++){
        for(int j = 0; j < activeModelTranslator->stateVectorSize; j++){
            double sqDiff = pow((A[midIndex](i, j) - matrixMidApprox(i, j)),2);

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
    }

    double averageError;
    if(counter > 0){
        averageError = errorSum / counter;
    }
    else{
        averageError = 0.0f;
    }

//    cout << "average error: " << averageError << "\n";
//    cout << "num valid: " << counter << "\n";
//    cout << "num too small: " << counterTooSmall << "\n";
//    cout << "num too large: " << counterTooLarge << "\n";
    if(averageError < 0.002){
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

std::vector<MatrixXd> interpolatediLQR::generateJerkProfile(){

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

void interpolatediLQR::getCostDerivs(){
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

void interpolatediLQR::getDerivativesAtSpecifiedIndices(std::vector<int> indices){

    activeDifferentiator->initModelForFiniteDifferencing();

    #pragma omp parallel for
    for(int i = 0; i < indices.size(); i++){

        int index = indices[i];
        activeDifferentiator->getDerivatives(A[index], B[index], false, index);

    }

    activeDifferentiator->resetModelAfterFiniteDifferencing();

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

void interpolatediLQR::interpolateDerivatives(std::vector<int> calculatedIndices){

    // Interpolate all the derivatvies that were not calculated via finite differencing
    for(int t = 0; t < calculatedIndices.size()-1; t++){

        MatrixXd addA(2*dof, 2*dof);
        MatrixXd addB(2*dof, num_ctrl);
        int nextInterpolationSize = calculatedIndices[t+1] - calculatedIndices[t];
        int startIndex = calculatedIndices[t];
        int endIndex = calculatedIndices[t+1];

        MatrixXd startA = A[startIndex].replicate(1, 1);
        MatrixXd endA = A[endIndex].replicate(1, 1);
        MatrixXd diffA = endA - startA;
        addA = diffA / nextInterpolationSize;

        MatrixXd startB = B[startIndex].replicate(1, 1);
        MatrixXd endB = B[endIndex].replicate(1, 1);
        MatrixXd diffB = endB - startB;
        addB = diffB / nextInterpolationSize;

        // Interpolate A and B matrices
        for(int i = 0; i < nextInterpolationSize; i++){
            f_x[startIndex + i] = A[startIndex].replicate(1,1) + (addA * i);
            f_u[startIndex + i] = B[startIndex].replicate(1,1) + (addB * i);
//                cout << "f_x[" << startIndex + i << "] = " << f_x[startIndex + i] << endl;
        }
    }

    f_x[horizonLength] = f_x[horizonLength - 1].replicate(1,1);
    f_u[horizonLength] = f_u[horizonLength - 1].replicate(1,1);
}

void interpolatediLQR::filterMatrices(){

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

std::vector<double> interpolatediLQR::filterIndividualValue(std::vector<double> unfiltered){
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


// ------------------------------------------- STEP 2 FUNCTIONS (BACKWARDS PASS) ----------------------------------------------
bool interpolatediLQR::backwardsPass_Quu_reg(){
    MatrixXd V_x(2*dof, 2*dof);
    V_x = l_x[horizonLength];
    MatrixXd V_xx(2*dof, 2*dof);
    V_xx = l_xx[horizonLength];
    int Quu_pd_check_counter = 0;
    int number_steps_between_pd_checks = 100;

    double timeInverse = 0.0f;

    for(int t = horizonLength - 1; t > -1; t--){
        MatrixXd Q_x(2*dof, 1);
        MatrixXd Q_u(num_ctrl, 1);
        MatrixXd Q_xx(2*dof, 2*dof);
        MatrixXd Q_uu(num_ctrl, num_ctrl);
        MatrixXd Q_ux(num_ctrl, 2*dof);

        Quu_pd_check_counter++;

        Q_u = l_u[t] + (f_u[t].transpose() * V_x);

        Q_x = l_x[t] + (f_x[t].transpose() * V_x);

        Q_ux = (f_u[t].transpose() * (V_xx * f_x[t]));

        Q_uu = l_uu[t] + (f_u[t].transpose() * (V_xx * f_u[t]));

        Q_xx = l_xx[t] + (f_x[t].transpose() * (V_xx * f_x[t]));

        MatrixXd Q_uu_reg = Q_uu.replicate(1, 1);

        for(int i = 0; i < Q_uu.rows(); i++){
            Q_uu_reg(i, i) += lambda;
        }

        if(Quu_pd_check_counter >= number_steps_between_pd_checks){
            if(!isMatrixPD(Q_uu_reg)){
                cout << "iteration " << t << endl;
                cout << "f_x[t - 3] " << f_x[t - 3] << endl;
                cout << "f_x[t - 2] " << f_x[t - 2] << endl;
                cout << "f_x[t - 1] " << f_x[t - 1] << endl;
                cout << "f_x[t] " << f_x[t] << endl;
                cout << "Q_uu_reg " << Q_uu_reg << endl;
                return false;
            }
            Quu_pd_check_counter = 0;
        }

        //  time this using chrono
        auto timeStart = std::chrono::high_resolution_clock::now();

        auto temp = (Q_uu_reg).ldlt();
        MatrixXd I(num_ctrl, num_ctrl);
        I.setIdentity();
        MatrixXd Q_uu_inv = temp.solve(I);

        auto timeEnd = std::chrono::high_resolution_clock::now();
        auto timeTaken = std::chrono::duration_cast<std::chrono::microseconds>(timeEnd - timeStart);
        timeInverse += timeTaken.count()/ 1000000.0f;
//        cout << "time taken for inverse: " << timeTaken.count()/ 1000000.0f << endl;

        k[t] = -Q_uu_inv * Q_u;
        K[t] = -Q_uu_inv * Q_ux;

        V_x = Q_x + (K[t].transpose() * (Q_uu * k[t])) + (K[t].transpose() * Q_u) + (Q_ux.transpose() * k[t]);
        V_xx = Q_xx + (K[t].transpose() * (Q_uu * K[t])) + (K[t].transpose() * Q_ux) + (Q_ux.transpose() * K[t]);

        V_xx = (V_xx + V_xx.transpose()) / 2;

        // if(t > horizonLength - 5){
        //     cout << "------------------ iteration " << t << " ------------------" << endl;
        //     cout << "l_x " << l_x[t] << endl;
        //     cout << "l_xx " << l_xx[t] << endl;
        //     cout << "l_u " << l_u[t] << endl;
        //     cout << "l_uu " << l_uu[t] << endl;
        //     cout << "Q_ux " << Q_ux << endl;
        //     cout << "f_u[t] " << f_u[t] << endl;
        //     cout << "Q_uu " << Q_uu << endl;
        //     cout << "Q_uu_inv " << Q_uu_inv << endl;
        //     cout << "Q_x " << Q_x << endl;
        //     cout << "Q_xx " << Q_xx << endl;
        //     cout << "V_xx " << V_xx << endl;
        //     cout << "V_x " << V_x << endl;
        //     cout << "K[t] " << K[t] << endl;
        // }
       
    }

    cout << "time taken for inverse: " << timeInverse << endl;

    return true;
}

bool interpolatediLQR::backwardsPass_Quu_skips(){
    MatrixXd V_x(2*dof, 2*dof);
    V_x = l_x[horizonLength];
    MatrixXd V_xx(2*dof, 2*dof);
    V_xx = l_xx[horizonLength];
    int Quu_pd_check_counter = 0;
    int number_steps_between_pd_checks = 100;

    int t = horizonLength - 1;
    MatrixXd Q_x(2*dof, 1);
    MatrixXd Q_u(num_ctrl, 1);
    MatrixXd Q_xx(2*dof, 2*dof);
    MatrixXd Q_uu(num_ctrl, num_ctrl);
    MatrixXd Q_ux(num_ctrl, 2*dof);

    while(t > 0){

        Quu_pd_check_counter ++;

        Q_u = l_u[t] + (f_u[t].transpose() * V_x);

        Q_x = l_x[t] + (f_x[t].transpose() * V_x);

        Q_ux = (f_u[t].transpose() * (V_xx * f_x[t]));

        Q_uu = l_uu[t] + (f_u[t].transpose() * (V_xx * f_u[t]));

        Q_xx = l_xx[t] + (f_x[t].transpose() * (V_xx * f_x[t]));

        MatrixXd Q_uu_reg = Q_uu.replicate(1, 1);

        for(int i = 0; i < Q_uu.rows(); i++){
            Q_uu_reg(i, i) += lambda;
        }

        if(Quu_pd_check_counter >= number_steps_between_pd_checks){
            if(!isMatrixPD(Q_uu_reg)){
                cout << "iteration " << t << endl;
                cout << "f_x[t - 3] " << f_x[t - 3] << endl;
                cout << "f_x[t - 2] " << f_x[t - 2] << endl;
                cout << "f_x[t - 1] " << f_x[t - 1] << endl;
                cout << "f_x[t] " << f_x[t] << endl;
                cout << "Q_uu_reg " << Q_uu_reg << endl;
                return false;
            }
            Quu_pd_check_counter = 0;
        }

        auto temp = (Q_uu_reg).ldlt();
        MatrixXd I(num_ctrl, num_ctrl);
        I.setIdentity();
        MatrixXd Q_uu_inv = temp.solve(I);

//        cout << "time taken for inverse: " << timeTaken.count()/ 1000000.0f << endl;

        k[t] = -Q_uu_inv * Q_u;
        K[t] = -Q_uu_inv * Q_ux;

        V_x = Q_x + (K[t].transpose() * (Q_uu * k[t])) + (K[t].transpose() * Q_u) + (Q_ux.transpose() * k[t]);
        V_xx = Q_xx + (K[t].transpose() * (Q_uu * K[t])) + (K[t].transpose() * Q_ux) + (Q_ux.transpose() * K[t]);

        V_xx = (V_xx + V_xx.transpose()) / 2;

    }

    return true;
}

bool interpolatediLQR::backwardsPass_Quu_reg_parallel(){

    MatrixXd V_x[8];
    MatrixXd V_xx[8];

    int endPoints[9];
    endPoints[0] = -1;

    for(int i = 0; i < 8; i++){
        V_x[i].resize(2*dof, 1);
        V_xx[i].resize(2*dof, 2*dof);

        int index = (i + 1) * (horizonLength / 8);
        cout << "index: " << index << endl;

//        V_x[i] = l_x[index];
//        V_xx[i] = l_xx[index];
        V_x[i] = l_x[horizonLength];
        V_xx[i] = l_xx[horizonLength];

        endPoints[i+1] = index;
    }

//    cout << "endPoints: " << endPoints[0] << " " << endPoints[1] << " " << endPoints[2] << " " << endPoints[3] << " " << endPoints[4] << " " << endPoints[5] << " " << endPoints[6] << " " << endPoints[7] << " " << endPoints[8] << endl;

#pragma omp parallel for
    for(int i = 0; i < 8; i++){
        MatrixXd Q_x(2 * dof, 1);
        MatrixXd Q_u(num_ctrl, 1);
        MatrixXd Q_xx(2 * dof, 2 * dof);
        MatrixXd Q_uu(num_ctrl, num_ctrl);
        MatrixXd Q_ux(num_ctrl, 2 * dof);

        for(int t = endPoints[i+1]; t > endPoints[i]; t--){
//            cout << "t: " << t << endl;
            Q_u = l_u[t] + (f_u[t].transpose() * V_x[i]);

            Q_x = l_x[t] + (f_x[t].transpose() * V_x[i]);

            Q_ux = (f_u[t].transpose() * (V_xx[i] * f_x[t]));

            Q_uu = l_uu[t] + (f_u[t].transpose() * (V_xx[i] * f_u[t]));

            Q_xx = l_xx[t] + (f_x[t].transpose() * (V_xx[i] * f_x[t]));

            MatrixXd Q_uu_reg = Q_uu.replicate(1, 1);

            for (int i = 0; i < Q_uu.rows(); i++) {
                Q_uu_reg(i, i) += lambda;
            }

            auto temp = (Q_uu_reg).ldlt();
            MatrixXd I(num_ctrl, num_ctrl);
            I.setIdentity();
            MatrixXd Q_uu_inv = temp.solve(I);

            k[t] = -Q_uu_inv * Q_u;
            K[t] = -Q_uu_inv * Q_ux;

            V_x[i] = Q_x + (K[t].transpose() * (Q_uu * k[t])) + (K[t].transpose() * Q_u) + (Q_ux.transpose() * k[t]);
            V_xx[i] = Q_xx + (K[t].transpose() * (Q_uu * K[t])) + (K[t].transpose() * Q_ux) + (Q_ux.transpose() * K[t]);

            V_xx[i] = (V_xx[i] + V_xx[i].transpose()) / 2;

        }
    }
    return true;
}

bool interpolatediLQR::isMatrixPD(Ref<MatrixXd> matrix){
    bool matrixPD = true;
    //TODO - implement cholesky decomp for PD check and maybe use result for inverse Q_uu

    Eigen::LLT<Eigen::MatrixXd> lltOfA(matrix); // compute the Cholesky decomposition of the matrix
    if(lltOfA.info() == Eigen::NumericalIssue)
    {
        matrixPD = false;
    }

    return matrixPD;
}

// ------------------------------------------- STEP 3 FUNCTIONS (FORWARDS PASS) ----------------------------------------------
double interpolatediLQR::forwardsPass(double oldCost, bool &costReduced){
    float alpha = 1.0;
    double newCost = 0.0;
    bool costReduction = false;
    int alphaCount = 0;
    float alphaReduc = 0.1;
    int alphaMax = (1 / alphaReduc) - 1;

    MatrixXd Xt(2 * dof, 1);
    MatrixXd X_last(2 * dof, 1);
    MatrixXd Ut(num_ctrl, 1);
    MatrixXd U_last(num_ctrl, 1);

    while(!costReduction){

        // Copy intial data state into main data state for rollout
        activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, 0);

        newCost = 0;
        MatrixXd stateFeedback(2*dof, 1);
        MatrixXd _X(2*dof, 1);
        MatrixXd X_new(2*dof, 1);
        MatrixXd _U(num_ctrl, 1);

//        #pragma omp parallel for
        for(int t = 0; t < horizonLength; t++) {
            // Step 1 - get old state and old control that were linearised around
            _X = activeModelTranslator->returnStateVector(t);
            //_U = activeModelTranslator->returnControlVector(t);
            _U = U_old[t].replicate(1, 1);

            X_new = activeModelTranslator->returnStateVector(MAIN_DATA_STATE);
            // Calculate difference from new state to old state
            stateFeedback = X_new - _X;

            MatrixXd feedBackGain = K[t] * stateFeedback;

            // Calculate new optimal controls
            U_new[t] = _U + (alpha * k[t]) + feedBackGain;

            // Clamp torque within limits
            if(activeModelTranslator->myStateVector.robots[0].torqueControlled){
                for(int i = 0; i < num_ctrl; i++){
                    if (U_new[t](i) > activeModelTranslator->myStateVector.robots[0].torqueLimits[i]) U_new[t](i) = activeModelTranslator->myStateVector.robots[0].torqueLimits[i];
                    if (U_new[t](i) < -activeModelTranslator->myStateVector.robots[0].torqueLimits[i]) U_new[t](i) = -activeModelTranslator->myStateVector.robots[0].torqueLimits[i];
                }

            }

//            cout << "old control: " << endl << U_old[t] << endl;
//            cout << "state feedback" << endl << stateFeedback << endl;
//            cout << "new control: " << endl << U_new[t] << endl;

            activeModelTranslator->setControlVector(U_new[t], MAIN_DATA_STATE);
            Xt = activeModelTranslator->returnStateVector(MAIN_DATA_STATE);

            Ut = U_new[t].replicate(1, 1);
//            cout << "U_new: " << endl << U_new[t] << endl;

            double newStateCost;
            // Terminal state
            if(t == horizonLength - 1){
                newStateCost = activeModelTranslator->costFunction(Xt, Ut, X_last, U_last, true);
            }
            else{
                newStateCost = activeModelTranslator->costFunction(Xt, Ut, X_last, U_last, false);
            }

            newCost += (newStateCost * MUJOCO_DT);

            activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);

//             if(t % 20 == 0){
//                 const char* fplabel = "fp";
//                 activeVisualizer->render(fplabel);
//             }

            X_last = Xt.replicate(1, 1);
            U_last = Ut.replicate(1, 1);

        }

        cout << "cost from alpha: " << alphaCount << ": " << newCost << endl;

        if(newCost < oldCost){
            costReduction = true;
            costReduced = true;
        }
        else{
            alpha = alpha - 0.1;
            alphaCount++;
            if(alphaCount >= alphaMax){
                break;
            }
        }
    }

    // If the cost was reduced
    if(newCost < oldCost){
        activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, 0);

        for(int i = 0; i < horizonLength; i++){

            activeModelTranslator->setControlVector(U_new[i], MAIN_DATA_STATE);
            activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);

            // Log the old state
             X_old.at(i + 1) = activeModelTranslator->returnStateVector(MAIN_DATA_STATE);

             activePhysicsSimulator->copySystemState(i+1, MAIN_DATA_STATE);

             U_old[i] = U_new[i].replicate(1, 1);

        }

        return newCost;
    }

    return oldCost;
}

double interpolatediLQR::forwardsPassParallel(double oldCost, bool &costReduced){
    float alpha = 1.0;
    double newCost = 0.0;
    bool costReduction = false;
    int alphaCount = 0;
    float alphaReduc = 0.1;
    int alphaMax = (1 / alphaReduc) - 1;

//    double alphas[8] = {0.125, 0.25, 0.375, 0.5, 0.675, 0.75, 0.875, 1.0};
    double alphas[8] = {1.0, 0.875, 0.75, 0.675, 0.5, 0.375, 0.25, 0.125};
    double newCosts[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    for(int i = 0; i < 8; i++){
        activePhysicsSimulator->copySystemState(i+1, 0);
    }

    MatrixXd initState = activeModelTranslator->returnStateVector(1);

    #pragma omp parallel for
    for(int i = 0; i < 8; i++){
        MatrixXd stateFeedback(2*dof, 1);
        MatrixXd _X(2*dof, 1);
        MatrixXd X_new(2*dof, 1);
        MatrixXd _U(num_ctrl, 1);
        MatrixXd Xt(2 * dof, 1);
        MatrixXd X_last(2 * dof, 1);
        MatrixXd Ut(num_ctrl, 1);
        MatrixXd U_last(num_ctrl, 1);

        for(int t = 0; t < horizonLength; t++) {
            // Step 1 - get old state and old control that were linearised around
            _X = X_old[t].replicate(1, 1);
            //_U = activeModelTranslator->returnControlVector(t);
            _U = U_old[t].replicate(1, 1);

            X_new = activeModelTranslator->returnStateVector(i+1);

            // Calculate difference from new state to old state
            stateFeedback = X_new - _X;

            MatrixXd feedBackGain = K[t] * stateFeedback;

            // Calculate new optimal controls
            U_alpha[t][i] = _U + (alphas[i] * k[t]) + feedBackGain;

            // Clamp torque within limits
            if(activeModelTranslator->myStateVector.robots[0].torqueControlled){
                for(int k = 0; k < num_ctrl; k++){
                    if (U_alpha[t][i](k) > activeModelTranslator->myStateVector.robots[0].torqueLimits[k]) U_alpha[t][i](k) = activeModelTranslator->myStateVector.robots[0].torqueLimits[k];
                    if (U_alpha[t][i](k) < -activeModelTranslator->myStateVector.robots[0].torqueLimits[k]) U_alpha[t][i](k) = -activeModelTranslator->myStateVector.robots[0].torqueLimits[k];
                }
            }

            activeModelTranslator->setControlVector(U_alpha[t][i], i+1);
//            Xt = activeModelTranslator->returnStateVector(i+1);
//            //cout << "Xt: " << Xt << endl;
//
            Ut = U_alpha[t][i].replicate(1, 1);
//
            double newStateCost;
            // Terminal state
            if(t == horizonLength - 1){
                newStateCost = activeModelTranslator->costFunction(X_new, Ut, X_last, U_last, true);
            }
            else{
                newStateCost = activeModelTranslator->costFunction(X_new, Ut, X_last, U_last, false);
            }

            newCosts[i] += (newStateCost * MUJOCO_DT);

            activePhysicsSimulator->stepSimulator(1, i+1);

//             if(t % 20 == 0){
//                 const char* fplabel = "fp";
//                 activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, i+1);
//
//                 activeVisualizer->render(fplabel);
//             }

            X_last = Xt.replicate(1, 1);
            U_last = Ut.replicate(1, 1);
        }
    }

    double bestAlphaCost = newCosts[0];
    int bestAlphaIndex = 0;
    for(int i = 0; i < 8; i++){
        if(newCosts[i] < bestAlphaCost){
            bestAlphaCost = newCosts[i];
            bestAlphaIndex = i;
        }
    }

    newCost = bestAlphaCost;
    cout << "best alpha cost = " << bestAlphaCost << " at alpha: " << alphas[bestAlphaIndex] << endl;
    activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, 0);

    // If the cost was reduced
    if(newCost < oldCost){
        initState = activeModelTranslator->returnStateVector(MAIN_DATA_STATE);

        for(int i = 0; i < horizonLength; i++){

            activeModelTranslator->setControlVector(U_alpha[i][bestAlphaIndex], MAIN_DATA_STATE);
            activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);

            // Log the old state
            X_old.at(i + 1) = activeModelTranslator->returnStateVector(MAIN_DATA_STATE);

            activePhysicsSimulator->copySystemState(i+1, MAIN_DATA_STATE);

            U_old[i] = U_alpha[i][bestAlphaIndex].replicate(1, 1);

        }

        return newCost;
    }

    return oldCost;
}


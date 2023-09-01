
#include "optimiser.h"

optimiser::optimiser(std::shared_ptr<modelTranslator> _modelTranslator, std::shared_ptr<physicsSimulator> _physicsSimulator, std::shared_ptr<fileHandler> _yamlReader, std::shared_ptr<differentiator> _differentiator){
    activeModelTranslator = _modelTranslator;
    activePhysicsSimulator = _physicsSimulator;
    activeYamlReader = _yamlReader;
    activeDifferentiator = _differentiator;

    dof = activeModelTranslator->dof;
    num_ctrl = activeModelTranslator->num_ctrl;

    // Set up the derivative interpolator from YAML settings
    activeDerivativeInterpolator.keypoint_method = activeModelTranslator->keypointMethod;
    activeDerivativeInterpolator.minN = activeModelTranslator->minN;
    activeDerivativeInterpolator.maxN = activeModelTranslator->maxN;
    activeDerivativeInterpolator.jerkThresholds = activeModelTranslator->jerkThresholds;
    // TODO - fix this - add acell thresholds to yaml
    activeDerivativeInterpolator.accelThresholds = activeModelTranslator->jerkThresholds;
    activeDerivativeInterpolator.iterativeErrorThreshold = activeModelTranslator->iterativeErrorThreshold;
    activeDerivativeInterpolator.magVelChangeThresholds = activeModelTranslator->magVelChangeThresholds;

}

bool optimiser::checkForConvergence(double oldCost, double newCost){
    double costGrad = (oldCost - newCost)/newCost;

    if(costGrad < epsConverge){
        return true;
    }
    return false;
}

void optimiser::setTrajecNumber(int _trajecNumber) {
    currentTrajecNumber = _trajecNumber;
}

void optimiser::returnOptimisationData(double &_optTime, double &_costReduction, double &_avgPercentageDerivs, double &_avgTimeGettingDerivs, int &_numIterations){

    _optTime = optTime;
    _costReduction = costReduction;
    _avgPercentageDerivs = avgPercentDerivs;
    _avgTimeGettingDerivs = avgTime_getDerivs_ms;
    _numIterations = numIterationsForConvergence;
}

derivative_interpolator optimiser::returnDerivativeInterpolator(){
    return activeDerivativeInterpolator;
}

void optimiser::setDerivativeInterpolator(derivative_interpolator _derivativeInterpolator){
    activeDerivativeInterpolator = _derivativeInterpolator;
}

void optimiser::generateDerivatives(){
    auto start = high_resolution_clock::now();
    // STEP 1 - Linearise dynamics and calculate first + second order cost derivatives for current trajectory
    // generate the dynamics evaluation waypoints
    std::vector<std::vector<int>> keyPoints = generateKeyPoints(X_old, U_old);

    // Calculate derivatives via finite differnecing / analytically for cost functions if available
    if(activeDerivativeInterpolator.keypoint_method != "iterative_error"){
        auto start_fd_time = high_resolution_clock::now();
        getDerivativesAtSpecifiedIndices(keyPoints);
        auto stop_fd_time = high_resolution_clock::now();
        auto duration_fd_time = duration_cast<microseconds>(stop_fd_time - start_fd_time);
//        std::cout << "Time taken to get derivatives: " << duration_fd_time.count() / 1000.0f << " ms" << std::endl;
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

    interpolateDerivatives(keyPoints, activeYamlReader->costDerivsFD);

    int totalNumColumnsDerivs = 0;
    for(int i = 0; i < keyPoints.size(); i++){
        totalNumColumnsDerivs += keyPoints[i].size();

    }

    double percentDerivsCalculated = ((double) totalNumColumnsDerivs / (double)numberOfTotalDerivs) * 100.0f;
    if(verboseOutput){
        cout << "percentage of derivs calculated: " << percentDerivsCalculated << endl;
    }

//        A.resize(initControls.size());
//        activeYamlReader->saveTrajecInfomation(A, B, X_old, U_old, activeModelTranslator->modelName, 1);

    if(filteringMatrices){
        filterMatrices();
    }

//        A.resize(initControls.size());
//        activeYamlReader->saveTrajecInfomation(A, B, X_old, U_old, activeModelTranslator->modelName, 2);

//    cout << "l_xx[horizonLength - 1]: " << l_xx[horizonLength - 1] << endl;
//    cout << "l_xx[horizonLength]: " << l_xx[horizonLength] << endl;
//    cout << "l_x[horizonLength]: " << l_x[horizonLength] << endl;
//    cout << "l_u[horizonLength - 1]: " << l_u[horizonLength - 1] << endl;
//    cout << "l_uu[horizonLength - 1]: " << l_uu[horizonLength - 1] << endl;

//    cout << "A[0]: " << endl << A[0] << endl;
//    cout << "A[1]: " << endl << A[1] << endl;
//    cout << "A[horizon - 2]" << endl << A[horizonLength - 2] << endl;
//
//    cout << "B[horizonLength - 1]: " << B[horizonLength - 1] << endl;

    auto stop = high_resolution_clock::now();
    auto linDuration = duration_cast<microseconds>(stop - start);
    time_getDerivs_ms.push_back(linDuration.count() / 1000.0f);

    percentDerivsPerIter.push_back(percentDerivsCalculated);
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

    if(activeDerivativeInterpolator.keypoint_method == "setInterval"){
        for(int i = 1; i < horizonLength - 1; i++){

            if(i % activeDerivativeInterpolator.minN == 0){
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
    }
    else if(activeDerivativeInterpolator.keypoint_method == "adaptive_jerk"){
        std::vector<MatrixXd> jerkProfile = generateJerkProfile();
        evaluationWaypoints = generateKeyPointsAdaptive(jerkProfile);
    }
    else if(activeDerivativeInterpolator.keypoint_method == "adaptive_accel"){
        std::vector<MatrixXd> accelProfile = generateAccelProfile();
        evaluationWaypoints = generateKeyPointsAdaptive(accelProfile);
    }
    else if(activeDerivativeInterpolator.keypoint_method == "iterative_error"){
        computedKeyPoints.clear();
        activePhysicsSimulator->initModelForFiniteDifferencing();
        evaluationWaypoints = generateKeyPointsIteratively();
        activePhysicsSimulator->resetModelAfterFiniteDifferencing();

    }
    else if(activeDerivativeInterpolator.keypoint_method == "magvel_change"){
        std::vector<MatrixXd> velProfile = generateVelProfile();
        evaluationWaypoints = generateKeyPointsMagVelChange(velProfile);
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
    std::vector<std::vector<int>> keypoints;

    for(int t = 0; t < horizonLength; t++){
        keypoints.push_back(std::vector<int>());
    }

    for(int i = 0; i < dof; i++){
        keypoints[0].push_back(i);
    }

    int lastIndices[dof];
    for(int i = 0; i < dof; i++){
        lastIndices[i] = 0;
    }

    // Loop over the trajectory
    for(int j = 0; j < dof; j++){
        for(int i = 0; i < trajecProfile.size(); i++){

            if((i - lastIndices[j]) >= activeDerivativeInterpolator.minN) {
                // Check if the jerk is above the threshold
                if (trajecProfile[i](j, 0) > activeDerivativeInterpolator.jerkThresholds[j]) {
                    keypoints[i].push_back(j);
                    lastIndices[j] = i;
                }
            }

            if((i - lastIndices[j]) >= activeDerivativeInterpolator.maxN){
                keypoints[i].push_back(j);
                lastIndices[j] = i;
            }
        }
    }

    return keypoints;
}

std::vector<std::vector<int>> optimiser::generateKeyPointsIteratively(){
    std::vector<std::vector<int>> keyPoints;
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
        keyPoints.push_back(std::vector<int>());
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
                    keyPoints[i].push_back(j);
                }
            }
        }
    }

    // Sort list into order
    for(int i = 0; i < dof; i++){
        std::sort(keyPoints[i].begin(), keyPoints[i].end());
    }

    // Remove duplicates
    for(int i = 0; i < dof; i++){
        keyPoints[i].erase(std::unique(keyPoints[i].begin(), keyPoints[i].end()), keyPoints[i].end());
    }

    return keyPoints;
}

std::vector<std::vector<int>> optimiser::generateKeyPointsMagVelChange(std::vector<MatrixXd> velProfile){
    std::vector<std::vector<int>> keyPoints;

    for(int t = 0; t < horizonLength; t++){
        keyPoints.push_back(std::vector<int>());
    }

    for(int i = 0; i < dof; i++){
        keyPoints[0].push_back(i);
    }

    // Keeps track of interval from last keypoint for this dof
    std::vector<int> lastKeypointCounter = std::vector<int>(dof, 0);
    std::vector<double> lastVelValue = std::vector<double>(dof, 0);
    std::vector<double> lastVelDirection = std::vector<double>(dof, 0);

    for(int i = 0; i < dof; i++){
        lastVelValue[i] = velProfile[i](0, 0);
    }

    // Loop over the velocity dofs
    for(int i = 0; i < dof; i++){
        // Loop over the horizon
        for(int t = 1; t < horizonLength; t++){

            lastKeypointCounter[i]++;
            double currentVelDirection = velProfile[t](i, 0) - velProfile[t - 1](i, 0);
            double currentVelChangeSinceKeypoint = velProfile[t](i, 0) - lastVelValue[i];

            // If the vel change is above the required threshold
            if(lastKeypointCounter[i] >= activeDerivativeInterpolator.minN){
                if(abs(currentVelChangeSinceKeypoint) > activeDerivativeInterpolator.magVelChangeThresholds[i]){
                    keyPoints[t].push_back(i);
                    lastVelValue[i] = velProfile[t](i, 0);
                    lastKeypointCounter[i] = 0;
                    continue;
                }
            }

//            cout << " after check mag change" << endl;

            // If the interval is greater than minN
            if(lastKeypointCounter[i] >= activeDerivativeInterpolator.minN){
                // If the direction of the velocity has changed
                if(currentVelDirection * lastVelDirection[i] < 0){
                    keyPoints[t].push_back(i);
                    lastVelValue[i] = velProfile[t](i, 0);
                    lastKeypointCounter[i] = 0;
                    continue;
                }
            }
            else{
                lastVelDirection[i] = currentVelDirection;
            }

//            cout << " after check direction" << endl;

            // If interval is greater than maxN
            if(lastKeypointCounter[i] >= activeDerivativeInterpolator.maxN){
                keyPoints[t].push_back(i);
                lastVelValue[i] = velProfile[t](i, 0);
                lastKeypointCounter[i] = 0;
                continue;
            }
        }
    }

    // Enforce last keypoint for all sofs at horizonLength - 1
    for(int i = 0; i < dof; i++){
        keyPoints[horizonLength-1].push_back(i);
    }

    return keyPoints;
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
    if((indices.endIndex - indices.startIndex) <=  activeDerivativeInterpolator.minN){
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
//        cout << "matrixMidApprox: \n" << midColumnsApprox[0] << "\n";
//
//        cout << "matrixMidTrue: \n" << A[midIndex].block(0, dofIndex + dof, dof*2, 1) << "\n";
//        cout << "matrixMidApprox: \n" << midColumnsApprox[1] << "\n";
//    }


    bool approximationGood = false;
    int dof = activeModelTranslator->dof;
    double errorSum = 0.0f;
    int counter = 0;

    for(int i = 0; i < 2; i++){
        int A_col_indices[2] = {dofIndex, dofIndex + dof};
        for(int j = dof; j < activeModelTranslator->stateVectorSize; j++){
            double sqDiff = pow((A[midIndex](j, A_col_indices[i]) - midColumnsApprox[i](j, 0)),2);

//            double absdiff = abs(A[midIndex](j, A_col_indices[i]) - midColumnsApprox[i](j, 0));
//            if(sqDiff > 0.1){
//                sqDiff = 0.0f;
//                counterTooLarge++;
//            }
////            else if(sqDiff < 0.00001){
////                sqDiff = 0.0f;
////                counterTooSmall++;
////            }
//            else{
//
//                counter++;
//            }
            counter++;
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

    // 0.00005
    if(averageError <  activeDerivativeInterpolator.iterativeErrorThreshold){ //0.00001
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

void optimiser::getCostDerivs(){
    #pragma omp parallel for
    for(int i = 0; i < horizonLength; i++){
        if(i == 0){
            activeModelTranslator->costDerivatives(i, l_x[i], l_xx[i], l_u[i], l_uu[i], false);
        }
        else{
            activeModelTranslator->costDerivatives(i, l_x[i], l_xx[i], l_u[i], l_uu[i], false);
        }

    }

    activeModelTranslator->costDerivatives(horizonLength-1,
                                           l_x[horizonLength], l_xx[horizonLength], l_u[horizonLength], l_uu[horizonLength], true);
}

void optimiser::getDerivativesAtSpecifiedIndices(std::vector<std::vector<int>> keyPoints){

    activePhysicsSimulator->initModelForFiniteDifferencing();

    // Loop through keypoints and delete any entries that have no keypoints
    for(int i = 0; i < keyPoints.size(); i++){
        if(keyPoints[i].size() == 0){
            keyPoints.erase(keyPoints.begin() + i);
            i--;
        }
    }

//    for(int i = 0; i < keyPoints.size(); i++){
//        for(int j = 0; j < keyPoints[i].size(); j++){
//            cout << keyPoints[i][j] << " ";
//        }
//        cout << "\n";
//    }

    #pragma omp parallel for
    for(int i = 0; i < keyPoints.size(); i++){

        int timeIndex = i;
        std::vector<int> columns = keyPoints[i];
//        cout << "columns.size(): index " << i << ": " << columns.size() << "\n";
        // If there are no keypoints at a certain data index, dont compute derivatives
//        if(columns.size() == 0){
//            continue;
//        }
        bool terminal = false;
        if(timeIndex == horizonLength - 1){
            terminal = true;
        }
        activeDifferentiator->getDerivatives(A[timeIndex], B[timeIndex], columns, l_x[timeIndex], l_u[timeIndex], l_xx[timeIndex], l_uu[timeIndex], activeYamlReader->costDerivsFD, timeIndex, terminal);

    }

//    cout << "A[0]: \n" << A[0] << "\n";
//    cout << "B[0]: \n" << B[0] << "\n";

//    activeYamlReader->generalSaveMatrices(l_x, "l_x_fd");
//    activeYamlReader->generalSaveMatrices(l_xx, "l_xx_fd");

    activePhysicsSimulator->resetModelAfterFiniteDifferencing();

    if(!activeYamlReader->costDerivsFD){
        #pragma omp parallel for
        for(int i = 0; i < horizonLength; i++){
            if(i == 0){
                activeModelTranslator->costDerivatives(i, l_x[i], l_xx[i], l_u[i], l_uu[i], false);
            }
            else{
                activeModelTranslator->costDerivatives(i, l_x[i], l_xx[i], l_u[i], l_uu[i], false);
            }
        }
        //    #TODO - check if this should be horizon length
        activeModelTranslator->costDerivatives(horizonLength - 1,
                                               l_x[horizonLength], l_xx[horizonLength], l_u[horizonLength], l_uu[horizonLength], true);

    }
}

void optimiser::interpolateDerivatives(std::vector<std::vector<int>> keyPoints, bool costDerivs){
    MatrixXd startB;
    MatrixXd endB;
    MatrixXd addB;

    double start_l_x_col1;
    double end_l_x_col1;
    double add_l_x_col1;
    double start_l_x_col2;
    double end_l_x_col2;
    double add_l_x_col2;

    MatrixXd start_l_xx_col1;
    MatrixXd end_l_xx_col1;
    MatrixXd add_l_xx_col1;
    MatrixXd start_l_xx_col2;
    MatrixXd end_l_xx_col2;
    MatrixXd add_l_xx_col2;

    // Create an array to track startIndices of next interpolation for each dof
    int startIndices[dof];
    for(int i = 0; i < dof; i++){
        startIndices[i] = 0;
    }

    // Loop through all the time indices - can skip the first
    // index as we preload the first index as the start index for all dofs.
    for(int t = 1; t <= horizonLength; t++){
        // Loop through all the dofs
        for(int i = 0; i < dof; i++){
            // Check the current vector at that time segment for the current dof
            std::vector<int> columns = keyPoints[t];

            // If there are no keypoints, continue onto second run of the loop
            if(columns.size() == 0){
                continue;
            }

            for(int j = 0; j < columns.size(); j++){

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

                    if(costDerivs){
                        start_l_x_col1 = l_x[startIndices[i]](i, 0);
                        end_l_x_col1 = l_x[t](i, 0);
                        add_l_x_col1 = (end_l_x_col1 - start_l_x_col1) / (t - startIndices[i]);

                        start_l_x_col2 = l_x[startIndices[i]](i + dof, 0);
                        end_l_x_col2 = l_x[t](i + dof, 0);
                        add_l_x_col2 = (end_l_x_col2 - start_l_x_col2) / (t - startIndices[i]);

                        start_l_xx_col1 = l_xx[startIndices[i]].block(i, 0, 1, dof);
                        end_l_xx_col1 = l_xx[t].block(i, 0, 1, dof);
                        add_l_xx_col1 = (end_l_xx_col1 - start_l_xx_col1) / (t - startIndices[i]);

                        start_l_xx_col2 = l_xx[startIndices[i]].block(i + dof, 0, 1, dof);
                        end_l_xx_col2 = l_xx[t].block(i + dof, 0, 1, dof);
                        add_l_xx_col2 = (end_l_xx_col2 - start_l_xx_col2) / (t - startIndices[i]);
                    }

                    if(i < num_ctrl){
                        startB = B[startIndices[i]].block(dof, i, dof, 1);
                        endB = B[t].block(dof, i, dof, 1);
                        addB = (endB - startB) / (t - startIndices[i]);
                    }

                    for(int k = startIndices[i]; k < t; k++){
                        A[k].block(dof, i, dof, 1) = startACol1 + ((k - startIndices[i]) * addACol1);

                        A[k].block(dof, i + dof, dof, 1) = startACol2 + ((k - startIndices[i]) * addACol2);

                        if(costDerivs){
                            l_x[k](i) = start_l_x_col1 + ((k - startIndices[i]) * add_l_x_col1);
                            l_x[k](i + dof) = start_l_x_col2 + ((k - startIndices[i]) * add_l_x_col2);

                            l_xx[k].block(i, 0, 1, dof) = start_l_xx_col1 + ((k - startIndices[i]) * add_l_xx_col1);
                            l_xx[k].block(i + dof, 0, 1, dof) = start_l_xx_col2 + ((k - startIndices[i]) * add_l_xx_col2);
                        }

                        if(i < num_ctrl){
                            B[k].block(dof, i, dof, 1) = startB + ((k - startIndices[i]) * addB);
                        }
                    }
                    startIndices[i] = t;
                }
            }
        }
    }
    // TODO - remove this????
//    l_xx[horizonLength] = l_xx[horizonLength - 1].replicate(1, 1);
//    l_x[horizonLength] = l_x[horizonLength - 1].replicate(1, 1);
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

std::vector<MatrixXd> optimiser::generateVelProfile(){
    std::vector<MatrixXd> velProfile;
    MatrixXd velocities(activeModelTranslator->dof, 1);

    for(int t = 0; t < horizonLength; t++){

        for(int i = 0; i < activeModelTranslator->dof; i++){
            velocities(i, 0) = X_old[t](i+dof, 0);
        }

        velProfile.push_back(velocities);
    }

    return velProfile;
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


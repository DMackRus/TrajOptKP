#include "KeyPointGenerator.h"

KeyPointGenerator::KeyPointGenerator(std::shared_ptr<Differentiator> _differentiator, std::shared_ptr<PhysicsSimulator> _physics_simulator) {
    differentiator = _differentiator;
    physics_simulator = _physics_simulator;

}

std::vector<std::vector<int>> KeyPointGenerator::GenerateKeyPoints(int horizon, std::vector<MatrixXd> trajectory_states, std::vector<MatrixXd> trajec_controls,
                                                                   keypoint_method keypoint_method, std::vector<MatrixXd> &A, std::vector<MatrixXd> &B){
    int dof = trajectory_states[0].rows() / 2;

    std::cout << "Generating key-points using method: " << keypoint_method.name << "\n";
    // Loop through the trajectory and decide what indices should be evaluated via finite differencing
    std::vector<std::vector<int>> keypoints;
    std::vector<int> oneRow;
    for(int i = 0; i < dof; i++){
        oneRow.push_back(i);
    }
    keypoints.push_back(oneRow);

    // For the trivial case of horizon == 2, we only need to evaluate the start and end points
    if(horizon == 2){
        keypoints.push_back(oneRow);
        return keypoints;
    }

    if(keypoint_method.name == "setInterval"){
        for(int i = 1; i < horizon; i++){

            if(i % keypoint_method.min_N == 0){
                std::vector<int> oneRow;
                for(int j = 0; j < dof; j++){
                    oneRow.push_back(j);
                }
                keypoints.push_back(oneRow);
            }
            else{
                std::vector<int> oneRow;
                keypoints.push_back(oneRow);
            }
        }
    }
    else if(keypoint_method.name == "adaptive_jerk"){
        std::vector<MatrixXd> jerkProfile = GenerateJerkProfile(horizon, trajectory_states);
        keypoints = GenerateKeyPointsAdaptive(horizon, jerkProfile, keypoint_method);
    }
    else if(keypoint_method.name == "adaptive_accel"){
        std::vector<MatrixXd> accelProfile = GenerateAccellerationProfile(horizon, trajectory_states);
        keypoints = GenerateKeyPointsAdaptive(horizon, accelProfile, keypoint_method);
    }
    else if(keypoint_method.name == "iterative_error"){
        computedKeyPoints.clear();
        physics_simulator->initModelForFiniteDifferencing();
        keypoints = GenerateKeyPointsIteratively(horizon, keypoint_method, trajectory_states, A, B);
        physics_simulator->resetModelAfterFiniteDifferencing();

    }
    else if(keypoint_method.name == "magvel_change"){
        std::vector<MatrixXd> velProfile = GenerateVelocityProfile(horizon, trajectory_states);
        keypoints = GenerateKeyPointsVelocityChange(horizon, velProfile, keypoint_method);
    }
    else{
        std::cout << "ERROR: keyPointsMethod not recognised \n";
    }

    // Enforce that last time step is evaluated for all dofs
    keypoints.back().clear();

    for(int i = 0; i < dof; i++){
        keypoints.back().push_back(i);
    }

//     Print out the key points
//    for(int i = 0; i < keypoints.size(); i++){
//        cout << "timestep " << i << ": ";
//        for(int j = 0; j < keypoints[i].size(); j++){
//            cout << keypoints[i][j] << " ";
//        }
//        cout << "\n";
//    }

    return keypoints;
}

std::vector<std::vector<int>> KeyPointGenerator::GenerateKeyPointsAdaptive(int horizon, std::vector<MatrixXd> trajecProfile,
                                                                           keypoint_method keypoint_method) {
    int dof = trajecProfile[0].rows() / 2;

    std::vector<std::vector<int>> keypoints;

    for(int t = 0; t < horizon; t++){
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

            if((i - lastIndices[j]) >= keypoint_method.min_N) {
                // Check if the jerk is above the threshold
                if (trajecProfile[i](j, 0) > keypoint_method.jerk_thresholds[j]) {
                    keypoints[i].push_back(j);
                    lastIndices[j] = i;
                }
            }

            if((i - lastIndices[j]) >= keypoint_method.max_N){
                keypoints[i].push_back(j);
                lastIndices[j] = i;
            }
        }
    }
    return keypoints;
}

std::vector<std::vector<int>> KeyPointGenerator::GenerateKeyPointsIteratively(int horizon, keypoint_method keypoint_method, std::vector<MatrixXd> trajectory_states,
                                                                              std::vector<MatrixXd> &A, std::vector<MatrixXd> &B) {
    int dof = trajectory_states[0].rows() / 2;

    std::vector<std::vector<int>> keyPoints;
    bool binsComplete[dof];
    std::vector<indexTuple> indexTuples;
    int startIndex = 0;
    int endIndex = horizon - 1;

    // Initialise variables
    for(int i = 0; i < dof; i++){
        binsComplete[i] = false;
        computedKeyPoints.push_back(std::vector<int>());
    }

    for(int i = 0; i < horizon; i++){
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
//                cout <<"dof: " << i <<  ": index tuple: " << listOfIndicesCheck[j].startIndex << " " << listOfIndicesCheck[j].endIndex << endl;
                bool approximationGood = CheckDOFColumnError(listOfIndicesCheck[j], i, keypoint_method, dof, A, B);

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
                }
                else{
                    subListWithMidpoints.push_back(listOfIndicesCheck[j].startIndex);
                    subListWithMidpoints.push_back(midIndex);
                    subListWithMidpoints.push_back(listOfIndicesCheck[j].endIndex);
                }
            }

            if(allChecksComplete){
                binsComplete[i] = true;
                subListWithMidpoints.clear();
            }

            listOfIndicesCheck = subListIndices;
            subListIndices.clear();
        }
    }

    // Loop over the horizon
    for(int i = 0; i < horizon; i++){
        // Loop over the dofs
        for(int j = 0; j < dof; j++){
            // Loop over the computed key points per dof
            for(int k = 0; k < computedKeyPoints[j].size(); k++){
                // If the current index is a computed key point
                if(i == computedKeyPoints[j][k]){
                    keyPoints[i].push_back(j);
                }
            }
        }
    }

    // Sort list into order
    for(int i = 0; i < horizon; i++){
        std::sort(keyPoints[i].begin(), keyPoints[i].end());
    }

    // Remove duplicates
    for(int i = 0; i < horizon; i++){
        keyPoints[i].erase(std::unique(keyPoints[i].begin(), keyPoints[i].end()), keyPoints[i].end());
    }

    return keyPoints;
}

bool KeyPointGenerator::CheckDOFColumnError(indexTuple indices, int dof_index, keypoint_method keypoint_method, int num_dofs,
                                            std::vector<MatrixXd> &A, std::vector<MatrixXd> &B) {
    int state_vector_size = num_dofs * 2;

    MatrixXd midColumnsApprox[2];
    for(int i = 0; i < 2; i++){
        midColumnsApprox[i] = MatrixXd::Zero(state_vector_size, 1);
    }

    int midIndex = (indices.startIndex + indices.endIndex) / 2;
    if((indices.endIndex - indices.startIndex) <=  keypoint_method.min_N){
        return true;
    }

    MatrixXd blank1, blank2, blank3, blank4;

    bool startIndexExists = false;
    bool midIndexExists = false;
    bool endIndexExists = false;

    for(int i = 0; i < computedKeyPoints[dof_index].size(); i++){
        if(computedKeyPoints[dof_index][i] == indices.startIndex){
            startIndexExists = true;
        }

        if(computedKeyPoints[dof_index][i] == midIndex){
            midIndexExists = true;
        }

        if(computedKeyPoints[dof_index][i] == indices.endIndex){
            endIndexExists = true;
        }
    }

    std::vector<int> cols;
    cols.push_back(dof_index);

    int tid = omp_get_thread_num();

    if(!startIndexExists){
        differentiator->getDerivatives(A[indices.startIndex], B[indices.startIndex], cols, blank1, blank2, blank3, blank4, false, indices.startIndex, false, tid);
        computedKeyPoints[dof_index].push_back(indices.startIndex);
    }

    if(!midIndexExists){
        differentiator->getDerivatives(A[midIndex], B[midIndex], cols, blank1, blank2, blank3, blank4, false, midIndex, false, tid);
        computedKeyPoints[dof_index].push_back(midIndex);
    }

    if(!endIndexExists){
        differentiator->getDerivatives(A[indices.endIndex], B[indices.endIndex], cols, blank1, blank2, blank3, blank4, false, indices.endIndex, false, tid);
        computedKeyPoints[dof_index].push_back(indices.endIndex);
    }

    midColumnsApprox[0] = (A[indices.startIndex].block(0, dof_index, num_dofs*2, 1) + A[indices.endIndex].block(0, dof_index, num_dofs*2, 1)) / 2;
    midColumnsApprox[1] = (A[indices.startIndex].block(0, dof_index + num_dofs, num_dofs*2, 1) + A[indices.endIndex].block(0, dof_index + num_dofs, num_dofs*2, 1)) / 2;


    bool approximationGood = false;
    int dof = num_dofs;
    double errorSum = 0.0f;
    int counter = 0;

    for(int i = 0; i < 2; i++){
        int A_col_indices[2] = {dof_index, dof_index + dof};
        for(int j = dof; j < num_dofs*2; j++){
            double sqDiff = pow((A[midIndex](j, A_col_indices[i]) - midColumnsApprox[i](j, 0)),2);

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

    if(averageError <  keypoint_method.iterative_error_threshold){
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

std::vector<std::vector<int>> KeyPointGenerator::GenerateKeyPointsVelocityChange(int horizon, std::vector<MatrixXd> velProfile,
                                                                                 keypoint_method keypoint_method) {
    int dof = velProfile[0].rows();

    std::vector<std::vector<int>> keyPoints;

    for(int t = 0; t < horizon; t++){
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
        lastVelValue[i] = velProfile[0](i, 0);
    }

    // Loop over the velocity dofs
    for(int i = 0; i < dof; i++){
        // Loop over the horizon
        for(int t = 1; t < horizon; t++){

            lastKeypointCounter[i]++;
            double currentVelDirection = velProfile[t](i, 0) - velProfile[t - 1](i, 0);
            double currentVelChangeSinceKeypoint = velProfile[t](i, 0) - lastVelValue[i];

            // If the vel change is above the required threshold
            if(lastKeypointCounter[i] >= keypoint_method.min_N){
                if(abs(currentVelChangeSinceKeypoint) > keypoint_method.velocity_change_threshold[i]){
                    keyPoints[t].push_back(i);
                    lastVelValue[i] = velProfile[t](i, 0);
                    lastKeypointCounter[i] = 0;
                    continue;
                }
            }

//            cout << " after check mag change" << endl;

            // If the interval is greater than min_N
            if(lastKeypointCounter[i] >= keypoint_method.min_N){
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

            // If interval is greater than max_N
            if(lastKeypointCounter[i] >= keypoint_method.max_N){
                keyPoints[t].push_back(i);
                lastVelValue[i] = velProfile[t](i, 0);
                lastKeypointCounter[i] = 0;
                continue;
            }
        }
    }

    // Enforce last keypoint for all dofs at horizonLength - 1
    for(int i = 0; i < dof; i++){
        keyPoints[horizon-1].push_back(i);
    }

    return keyPoints;
}

std::vector<MatrixXd> KeyPointGenerator::GenerateJerkProfile(int horizon, std::vector<MatrixXd> trajectory_states) {

    int dof = trajectory_states[0].rows() / 2;
    MatrixXd jerk(dof, 1);

    MatrixXd state1(trajectory_states[0].rows(), 1);
    MatrixXd state2(trajectory_states[0].rows(), 1);
    MatrixXd state3(trajectory_states[0].rows(), 1);

    std::vector<MatrixXd> jerkProfile;

    for(int i = 0; i < horizon - 2; i++){
        state1 = trajectory_states[i];
        state2 = trajectory_states[i + 1];
        state3 = trajectory_states[i + 2];

        MatrixXd accell1 = state2 - state1;
        MatrixXd accell2 = state3 - state2;

        for(int j = 0; j < dof; j++){
            jerk(j, 0) = abs(accell2(j+dof, 0) - accell1(j+dof, 0));
        }

        jerkProfile.push_back(jerk);
    }

    return jerkProfile;
}

std::vector<MatrixXd> KeyPointGenerator::GenerateAccellerationProfile(int horizon, std::vector<MatrixXd> trajectory_states) {
    int dof = trajectory_states[0].rows() / 2;
    MatrixXd accel(dof, 1);

    MatrixXd state1(trajectory_states[0].rows(), 1);
    MatrixXd state2(trajectory_states[0].rows(), 1);

    std::vector<MatrixXd> accelProfile;

    for(int i = 0; i < horizon - 1; i++){
        state1 = trajectory_states[i];
        state2 = trajectory_states[i + 1];

        MatrixXd accelState = state2 - state1;

        for(int j = 0; j < dof; j++){
            accel(j, 0) = accelState(j+dof, 0);
        }

        accelProfile.push_back(accel);
    }

    return accelProfile;
}

std::vector<MatrixXd> KeyPointGenerator::GenerateVelocityProfile(int horizon, std::vector<MatrixXd> trajectory_states) {
    int dof = trajectory_states[0].rows() / 2;

    std::vector<MatrixXd> velProfile;
    MatrixXd velocities(dof, 1);

    for(int t = 0; t < horizon; t++){

        for(int i = 0; i < dof; i++){
            velocities(i, 0) = trajectory_states[t](i+dof, 0);
        }

        velProfile.push_back(velocities);
    }

    return velProfile;
}
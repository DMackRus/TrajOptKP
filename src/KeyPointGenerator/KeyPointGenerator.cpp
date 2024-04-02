#include "KeyPointGenerator.h"

KeypointGenerator::KeypointGenerator(std::shared_ptr<Differentiator> _differentiator,
                                     std::shared_ptr<MuJoCoHelper> MuJoCo_helper,
                                     int _dof, int _horizon) {
    differentiator = _differentiator;
    physics_simulator = MuJoCo_helper;

    dof = _dof;
    horizon = _horizon;

    // Allocate static arrays
    last_percentages.resize(dof);
    last_num_keypoints.resize(dof);

    max_last_jerk.resize(dof);
    min_last_jerk.resize(dof);

    max_last_velocity.resize(dof);
    min_last_velocity.resize(dof);

    for(int t = 0; t < horizon; t++){
        jerk_profile.push_back(MatrixXd(dof, 1));
        velocity_profile.push_back(MatrixXd(dof, 1));
    }
}

void KeypointGenerator::ResizeStateVector(int new_num_dofs, int _horizon){
    horizon = _horizon;
    dof = new_num_dofs;

    last_percentages.resize(dof);
    last_num_keypoints.resize(dof);

    max_last_jerk.resize(dof);
    min_last_jerk.resize(dof);

    max_last_velocity.resize(dof);
    min_last_velocity.resize(dof);

    // Setup the size of the jerk and velocity profiles
    jerk_profile.clear();
    velocity_profile.clear();

    for(int t = 0; t < horizon; t++){
        jerk_profile.push_back(MatrixXd(dof, 1));
        velocity_profile.push_back(MatrixXd(dof, 1));
    }
}

keypoint_method KeypointGenerator::ReturnCurrentKeypointMethod() {
    return current_keypoint_method;
}

void KeypointGenerator::SetKeypointMethod(keypoint_method method){
    current_keypoint_method = method;
}

void KeypointGenerator::PrintKeypointMethod(){
    std::cout << "Method: " << current_keypoint_method.name << std::endl;
    std::cout << "min_N: " << current_keypoint_method.min_N << " max_N: " << current_keypoint_method.max_N << std::endl;
    std::cout << "jerk thresholds: ";
    for(int i = 0; i < dof; i++){
        std::cout << " " << current_keypoint_method.jerk_thresholds[i];
    }
    std::cout << "\n ";
    std::cout << "velocity change thresholds: ";
    for(int i = 0; i < dof; i++){
        std::cout << " " << current_keypoint_method.velocity_change_thresholds[i];
    }
    std::cout << "\n ";
}

void KeypointGenerator::GenerateKeyPoints(const std::vector<MatrixXd> &trajectory_states,
                                               std::vector<MatrixXd> &A, std::vector<MatrixXd> &B){

    if(keypoints_computed){
        return;
    }

    keypoints.clear();
//    std::cout << "Clearing keypoint vectors: " << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - clear_start).count() / 1000.0f << "ms\n";

    if(current_keypoint_method.name == "set_interval"){
//        auto start_interval = std::chrono::high_resolution_clock::now();
        GenerateKeyPointsSetInterval();
//        std::cout << "Interval keypoint generation time: " << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_interval).count() / 1000.0f << "ms\n";

    }
    else if(current_keypoint_method.name == "adaptive_jerk"){
        auto start_jerk = std::chrono::high_resolution_clock::now();
        GenerateJerkProfile(trajectory_states);
//        std::cout << "Jerk profile generation time: " << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_jerk).count() / 1000.0f << "ms\n";
        GenerateKeyPointsAdaptive(jerk_profile);
    }
    else if(current_keypoint_method.name == "adaptive_accel"){
        std::vector<MatrixXd> acceleration_profile = GenerateAccellerationProfile(horizon, trajectory_states);
        GenerateKeyPointsAdaptive(acceleration_profile);
    }
    else if(current_keypoint_method.name == "iterative_error"){
        computed_keypoints.clear();
        physics_simulator->initModelForFiniteDifferencing();
        keypoints = GenerateKeyPointsIteratively(horizon, trajectory_states, A, B);
        physics_simulator->resetModelAfterFiniteDifferencing();

    }
    else if(current_keypoint_method.name == "velocity_change"){
        GenerateVelocityProfile(trajectory_states);
        GenerateKeyPointsVelocityChange(velocity_profile);
    }
    else{
        std::cerr << "ERROR: keyPointsMethod not recognised \n";
        exit(1);
    }

    //Print out the key points
//

    UpdateLastPercentageDerivatives(keypoints);
}

void KeypointGenerator::AdjustKeyPointMethod(double expected, double actual,
                                             std::vector<MatrixXd> &trajectory_states,
                                             std::vector<double> &dof_importances){


    // If we are not in auto-adjust mode, then return
    if(current_keypoint_method.auto_adjust == false){
        return;
    }

    // Compute lower limit for number of key-points, based on the max_N
    int lower_lim_num_derivs = ceil((double)horizon / (double)current_keypoint_method.max_N) + 1;

    std::vector<int> desired_num_keypoints = std::vector<int>(dof);
    std::vector<double> desired_derivative_percentages = std::vector<double>(dof);

    // New desired percentages

//    std::vector<int> desired_derivative_nums = std::vector<int>(dof, 0);

    // Print last num keypoints
//    cout << "Last num keypoints: ";
//    for(int i = 0; i < dof; i++){
//        cout << last_num_keypoints[i] << " ";
//    }
//    cout << "\n";
//
//    cout << "last percentages: ";
//    for(int i = 0; i < dof; i++){
//        cout << last_percentages[i] << " ";
//    }
//    cout << "\n";

    // If the last optimisation decreased the cost
    desired_derivative_percentages = DesiredPercentageDerivs(expected, actual, dof_importances);

//    std::cout << "desired derivative percentages: ";
//    for(int i = 0; i < dof; i++){
//        std::cout << desired_derivative_percentages[i] << " ";
//    }
//    std::cout << std::endl;

    // Convert percentages to number of key-points
    desired_num_keypoints = ConvertPercentagesToNumKeypoints(desired_derivative_percentages);

    // Enforce minimum and maximum number of key-points
    for(int i = 0; i < dof; i++){
        if(desired_num_keypoints[i] < lower_lim_num_derivs){
            desired_num_keypoints[i] = lower_lim_num_derivs;
        }

        if(desired_num_keypoints[i] > horizon){
            desired_num_keypoints[i] = horizon;
        }
    }

//    std::cout << "last percentages: ";
//    for(int i = 0; i < dof; i++){
//        std::cout << last_percentages[i] << " ";
//
//    }
//    std::cout << std::endl;
//
//    std::cout << "desired percentages: ";
//    for(int i = 0; i < dof; i++){
//        std::cout << desired_derivative_percentages[i] << " ";
//    }
//    std::cout << std::endl;

    AutoAdjustKeypointParameters(trajectory_states, desired_num_keypoints, 3);
}

std::vector<double> KeypointGenerator::DesiredPercentageDerivs(double expected, double actual,
                                            std::vector<double> &dof_importances){

    std::vector<double> desired_derivative_percentages = std::vector<double>(dof);

    double surprise = actual / expected;
//    std:: cout << "actual was: " << actual << " expected was: " << expected << "surprise was: " << surprise << std::endl;

    // If we has some cost reduction
    if(actual > 0){
        // Make the key-points greedier

        // When surprise is low, dont update
        double raw_adjust_factor;
        if(surprise < surprise_lower){
//            std::cout << "surprise was low" << std::endl;
            raw_adjust_factor = -2 - pow(expected, 2);

            if(raw_adjust_factor < -5){
                raw_adjust_factor = -5;
            }
        }
            // Lets scale our greediness depending on how much surprise we received
        else{
            // This might need caps on it.
            raw_adjust_factor = 3 * pow(surprise, 2) + 2;
        }

        // Cap the adjust factor
        if(raw_adjust_factor > 5){
            raw_adjust_factor = 5;
        }

//        std::cout << "raw adjust factor  " << raw_adjust_factor << std::endl;

        for(int i = 0; i < dof; i++){

            // Take into account the dof importances, if a dof is very important, we want to be less greedy
            // If a dof is not important, we want to be more greedy
            double adjust_factor;

            if(dof_importances[i] == 0.0){
                adjust_factor = raw_adjust_factor;
            }
            else{
                adjust_factor = raw_adjust_factor * (1.0 / dof_importances[i]);
            }
            desired_derivative_percentages[i] = last_percentages[i] - adjust_factor;
        }
    }
    // If we had no cost reduction
    else{
        // Make the key-points less greedy
        for(int i = 0; i < dof; i++) {

            // TODO(DMackRus) we might need to take into acount the old cost also.
            double raw_adjust_factor = pow(expected, 2);

            if(raw_adjust_factor > 5){
                raw_adjust_factor = 5;
            }

            double adjust_factor = raw_adjust_factor * dof_importances[i];

            desired_derivative_percentages[i] = last_percentages[i] + adjust_factor;
        }
    }

    return desired_derivative_percentages;
}

void KeypointGenerator::AutoAdjustKeypointParameters(const std::vector<MatrixXd> &trajectory_states,
                                  const std::vector<int> &desired_num_keypoints, int num_iterations){

    std::vector<double> dof_percentages;
    std::vector<MatrixXd> empty;

//    std::cout << "desired derivs: ";
//    for(int i = 0; i < dof; i++){
//        std::cout << desired_num_keypoints[i] << " ";
//    }
//    std::cout << std::endl;

    GenerateKeypointsOrderOfImportance(trajectory_states, desired_num_keypoints);
    UpdateLastPercentageDerivatives(keypoints);

//    std::cout << "actual derivs: ";
//    for(int i = 0; i < dof; i++){
//        std::cout << last_num_keypoints[i] << " ";
//    }
//    std::cout << std::endl;
//
//    std::cout << "new percentages: ";
//    for(int i = 0; i < dof; i++){
//        std::cout << last_percentages[i] << " ";
//    }
//    std::cout << std::endl;

//    for(int i = 0; i < keypoints.size(); i++){
//        cout << "timestep " << i << ": ";
//        for(int j = 0; j < keypoints[i].size(); j++){
//            cout << keypoints[i][j] << " ";
//        }
//        cout << "\n";
//    }

    // Prevents recomputation for next iteration as we have already adjusted.
    keypoints_computed = true;
}

void KeypointGenerator::GenerateKeyPointsSetInterval(){

    std::vector<int> full_row(dof, 0);
    std::vector<int> empty_row;

    for(int i = 0; i < dof; i++){
        full_row[i] = i;
    }

    for(int t = 0; t < horizon - 1; t++){
        if(t % current_keypoint_method.min_N == 0){
            keypoints.push_back(full_row);
        }
        else{
            keypoints.push_back(empty_row);
        }
    }

    // Always push the last row
    keypoints.push_back(full_row);
}

void KeypointGenerator::GenerateKeyPointsAdaptive(const std::vector<MatrixXd> &trajec_profile) {
    std::vector<int> full_row(dof, 0);

    for(int i = 0; i < dof; i++){
        full_row[i] = i;
    }
    keypoints.push_back(full_row);

    int last_indices[dof];
    for(int i = 0; i < dof; i++){
        last_indices[i] = 0;
        max_last_jerk[i] = trajec_profile[0](i, 0);
        min_last_jerk[i] = trajec_profile[0](i, 0);
    }

    for(int t = 1; t < horizon - 1; t++){
        std::vector<int> row;
        for(int j = 0; j < dof; j++){
            if((t - last_indices[j]) >= current_keypoint_method.min_N){
                if(trajec_profile[t](j, 0) > current_keypoint_method.jerk_thresholds[j]){
                    row.push_back(j);
                    last_indices[j] = t;
                }
            }
            if((t - last_indices[j]) >= current_keypoint_method.max_N){
                row.push_back(j);
                last_indices[j] = t;
            }

            // Update min and max values
            if(trajec_profile[t](j, 0) > max_last_jerk[j]){
                max_last_jerk[j] = trajec_profile[t](j, 0);
            }

            if(trajec_profile[t](j, 0) < min_last_jerk[j]){
                min_last_jerk[j] = trajec_profile[t](j, 0);
            }
        }
        keypoints.push_back(row);
    }
    keypoints.push_back(full_row);
}

void KeypointGenerator::GenerateKeypointsOrderOfImportance(const std::vector<MatrixXd> &trajectory_states,
                                                           const std::vector<int> &num_keypoints){
    // Generate jerk profile.
    GenerateJerkProfile(trajectory_states);

    std::vector<std::vector<int>> keypoints_per_dof(dof);

    for(int i = 0; i < dof; i++){
        std::vector<double> jerk_vals;
        for(int t = 1; t < horizon - 2; t++) {
            jerk_vals.push_back(jerk_profile[t](i, 0));
        }

        // Sort jerks in order of magnitude
        std::vector<int> sorted_indices = sortIndices(jerk_vals);

        // Have to push the first and last time indices
        keypoints_per_dof[i].push_back(0);
        keypoints_per_dof[i].push_back(horizon - 2);

        for (int k = 0; k < num_keypoints[i] - 2; k++) {
            keypoints_per_dof[i].push_back(sorted_indices[k]);
        }
    }

    //Print the keypoints per dof
//    for(int i = 0; i < dof; i++){
//        cout << "DOF " << i << ": ";
//        for(int j = 0; j < keypoints_per_dof[i].size(); j++){
//            cout << keypoints_per_dof[i][j] << " ";
//        }
//        cout << "\n";
//    }

    // clear the previous keypoints
    keypoints.clear();

    // Construct keypoints per timestep
    for(int i = 0; i < horizon - 1; i++){
        std::vector<int> row;
        for(int j = 0; j < dof; j++){
            // loop throguh keypoints_per_dof
            for(int k = 0; k < keypoints_per_dof[j].size(); k++){
                if(keypoints_per_dof[j][k] == i){
                    row.push_back(j);
//                    keypoints_per_dof.erase(keypoints_per_dof.begin() + i);
                    break;
                }
            }
        }
        keypoints.push_back(row);
    }

    // print keypoints
//    for(int i = 0; i < keypoints.size(); i++){
//        cout << "timestep " << i << ": ";
//        for(int j = 0; j < keypoints[i].size(); j++){
//            cout << keypoints[i][j] << " ";
//        }
//        cout << "\n";
//    }

}

std::vector<std::vector<int>> KeypointGenerator::GenerateKeyPointsIteratively(int horizon, std::vector<MatrixXd> trajectory_states,
                                                                              std::vector<MatrixXd> &A, std::vector<MatrixXd> &B) {
    int dof = trajectory_states[0].rows() / 2;

    std::vector<std::vector<int>> keypoints;
    bool bins_complete[dof];
    std::vector<index_tuple> index_tuples;
    int start_index = 0;
    int end_index = horizon - 1;

    // Initialise variables
    for(int i = 0; i < dof; i++){
        bins_complete[i] = false;
        computed_keypoints.push_back(std::vector<int>());
    }

    for(int i = 0; i < horizon; i++){
        keypoints.push_back(std::vector<int>());
    }

    // Loop through all dofs in the system
    #pragma omp parallel for
    for(int i = 0; i < dof; i++){
//        std::cout << "---------------------  Generating key points for dof --------------------------------- " << i << std::endl;
        std::vector<index_tuple> list_of_indices_check;
        index_tuple initial_tuple;
        initial_tuple.start_index = start_index;
        initial_tuple.end_index = end_index;
        list_of_indices_check.push_back(initial_tuple);

        std::vector<index_tuple> sub_list_indices;
        std::vector<int> sub_list_with_midpoints;

        while(!bins_complete[i]){
            bool allChecksComplete = true;

            for(int j = 0; j < list_of_indices_check.size(); j++) {

                int midIndex = (list_of_indices_check[j].start_index + list_of_indices_check[j].end_index) / 2;
//                cout <<"dof: " << i <<  ": index tuple: " << list_of_indices_check[j].start_index << " " << list_of_indices_check[j].end_index << endl;
                bool approximationGood = CheckDOFColumnError(list_of_indices_check[j], i, dof, A, B);

                if (!approximationGood) {
                    allChecksComplete = false;
                    index_tuple tuple1;
                    tuple1.start_index = list_of_indices_check[j].start_index;
                    tuple1.end_index = midIndex;
                    index_tuple tuple2;
                    tuple2.start_index = midIndex;
                    tuple2.end_index = list_of_indices_check[j].end_index;
                    sub_list_indices.push_back(tuple1);
                    sub_list_indices.push_back(tuple2);
                }
                else{
                    sub_list_with_midpoints.push_back(list_of_indices_check[j].start_index);
                    sub_list_with_midpoints.push_back(midIndex);
                    sub_list_with_midpoints.push_back(list_of_indices_check[j].end_index);
                }
            }

            if(allChecksComplete){
                bins_complete[i] = true;
                sub_list_with_midpoints.clear();
            }

            list_of_indices_check = sub_list_indices;
            sub_list_indices.clear();
        }
    }

    // Loop over the horizon
    for(int i = 0; i < horizon; i++){
        // Loop over the dofs
        for(int j = 0; j < dof; j++){
            // Loop over the computed key points per dof
            for(int k = 0; k < computed_keypoints[j].size(); k++){
                // If the current index is a computed key point
                if(i == computed_keypoints[j][k]){
                    keypoints[i].push_back(j);
                }
            }
        }
    }

    // Sort list into order
    for(int i = 0; i < horizon; i++){
        std::sort(keypoints[i].begin(), keypoints[i].end());
    }

    // Remove duplicates
    for(int i = 0; i < horizon; i++){
        keypoints[i].erase(std::unique(keypoints[i].begin(), keypoints[i].end()), keypoints[i].end());
    }

    return keypoints;
}

bool KeypointGenerator::CheckDOFColumnError(index_tuple indices, int dof_index, int num_dofs,
                                            std::vector<MatrixXd> &A, std::vector<MatrixXd> &B) {
    int state_vector_size = num_dofs * 2;

    // The two columns of the "A" matrix we will compare (position, velocity) for that dof to evaluate our approximation
    MatrixXd mid_columns_approximated[2];
    for(int i = 0; i < 2; i++){
        mid_columns_approximated[i] = MatrixXd::Zero(state_vector_size, 1);
    }

    // Middle index in trajectory between start and end index passed from "indices" struct
    int mid_index = (indices.start_index + indices.end_index) / 2;
    if((indices.end_index - indices.start_index) <= current_keypoint_method.min_N){
        return true;
    }

    MatrixXd blank1, blank2, blank3, blank4;

    bool start_index_computed = false;
    bool mid_index_computed = false;
    bool end_index_computed = false;

    for(int i = 0; i < computed_keypoints[dof_index].size(); i++){
        if(computed_keypoints[dof_index][i] == indices.start_index){
            start_index_computed = true;
        }

        if(computed_keypoints[dof_index][i] == mid_index){
            mid_index_computed = true;
        }

        if(computed_keypoints[dof_index][i] == indices.end_index){
            end_index_computed = true;
        }
    }

    std::vector<int> cols;
    cols.push_back(dof_index);

    // Gets thread id so we can make sure we use different data structure for F.D computations
    int tid = omp_get_thread_num();

    if(!start_index_computed){
        differentiator->ComputeDerivatives(A[indices.start_index], B[indices.start_index], cols, blank1, blank2, blank3, blank4, false, indices.start_index, false, tid, true, 1e-6);
        computed_keypoints[dof_index].push_back(indices.start_index);
    }

    if(!mid_index_computed){
        differentiator->ComputeDerivatives(A[mid_index], B[mid_index], cols, blank1, blank2, blank3, blank4, false, mid_index, false, tid, true, 1e-6);
        computed_keypoints[dof_index].push_back(mid_index);
    }

    if(!end_index_computed){
        differentiator->ComputeDerivatives(A[indices.end_index], B[indices.end_index], cols, blank1, blank2, blank3, blank4, false, indices.end_index, false, tid, true, 1e-6);
        computed_keypoints[dof_index].push_back(indices.end_index);
    }

    mid_columns_approximated[0] = (A[indices.start_index].block(0, dof_index, num_dofs * 2, 1) + A[indices.end_index].block(0, dof_index, num_dofs * 2, 1)) / 2;
    mid_columns_approximated[1] = (A[indices.start_index].block(0, dof_index + num_dofs, num_dofs * 2, 1) + A[indices.end_index].block(0, dof_index + num_dofs, num_dofs * 2, 1)) / 2;


    bool approximation_good = false;
    double error_sum = 0.0f;
    int counter = 0;

    for(int i = 0; i < 2; i++){
        int A_col_indices[2] = {dof_index, dof_index + num_dofs};
        for(int j = num_dofs; j < num_dofs*2; j++){
            double square_difference = pow((A[mid_index](j, A_col_indices[i]) - mid_columns_approximated[i](j, 0)), 2);

            counter++;
            error_sum += square_difference;
        }
//        cout << "error_sum: " << error_sum << "\n";
    }

    double average_error;
    if(counter > 0){
        average_error = error_sum / counter;
    }
    else{
        average_error = 0.0f;
    }

//    if(dofIndex == 0){
//        cout << "average error: " << average_error << "\n";
//    }

//    cout << "average error: " << average_error << "\n";
//    cout << "num valid: " << counter << "\n";
//    cout << "num too small: " << counterTooSmall << "\n";
//    cout << "num too large: " << counterTooLarge << "\n";

    if(average_error < current_keypoint_method.iterative_error_threshold){
        approximation_good = true;
    }
    else{
//        cout << "matrix mid approx" << matrixMidApprox << "\n";
//        cout << "matrix mid true" << A[mid_index] << "\n";
    }

//    if(counter == 0){
//        cout << "start index: " << indices.start_index << " mid index: " << mid_index << " end index: " << indices.end_index << "\n";
//        cout << "matrix mid approx" << matrixMidApprox << "\n";
//        cout << "matrix mid true" << A[mid_index] << "\n";
//    }

    return approximation_good;
}

void KeypointGenerator::GenerateKeyPointsVelocityChange(const std::vector<MatrixXd> &velocity_profile) {

    std::vector<int> full_row(dof, 0);

    for(int i = 0; i < dof; i++){
        full_row[i] = i;
    }
    keypoints.push_back(full_row);

    // Keeps track of interval from last keypoint for this dof
    std::vector<int> last_keypoint_counter = std::vector<int>(dof, 0);
    std::vector<double> last_vel_value = std::vector<double>(dof, 0);
    std::vector<double> last_vel_direction = std::vector<double>(dof, 0);

    for(int i = 0; i < dof; i++){
        last_vel_value[i] = velocity_profile[0](i, 0);

        min_last_velocity[i] = velocity_profile[0](i, 0);
        max_last_velocity[i] = velocity_profile[0](i, 0);
    }

    // Loop over the horizon
    for(int t = 1; t < horizon; t++){
        std::vector<int> row;

        // Loop over the velocity dofs
        for(int i = 0; i < dof; i++){

            last_keypoint_counter[i]++;
            double current_vel_direction = velocity_profile[t](i, 0) - velocity_profile[t - 1](i, 0);
            double current_vel_change_since_last_keypoint = velocity_profile[t](i, 0) - last_vel_value[i];

            // If the vel change is above the required threshold
            if(last_keypoint_counter[i] >= current_keypoint_method.min_N){
                if(abs(current_vel_change_since_last_keypoint) > current_keypoint_method.velocity_change_thresholds[i]){
                    row.push_back(i);
                    last_vel_value[i] = velocity_profile[t](i, 0);
                    last_keypoint_counter[i] = 0;
                    continue;
                }
            }

            // If the interval is greater than min_N
            if(last_keypoint_counter[i] >= current_keypoint_method.min_N){
                // If the direction of the velocity has changed
                if(current_vel_direction * last_vel_direction[i] < 0){
                    row.push_back(i);
                    last_vel_value[i] = velocity_profile[t](i, 0);
                    last_keypoint_counter[i] = 0;
                    continue;
                }
            }
            else{
                last_vel_direction[i] = current_vel_direction;
            }

            // If interval is greater than max_N
            if(last_keypoint_counter[i] >= current_keypoint_method.max_N){
                row.push_back(i);
                last_vel_value[i] = velocity_profile[t](i, 0);
                last_keypoint_counter[i] = 0;
                continue;
            }

            // Update min and max velocities
            if(velocity_profile[t](i, 0) < min_last_velocity[i]){
                min_last_velocity[i] = velocity_profile[t](i, 0);
            }

            if(velocity_profile[t](i, 0) > max_last_velocity[i]){
                max_last_velocity[i] = velocity_profile[t](i, 0);
            }
        }

        keypoints.push_back(row);
    }

    // Enforce last keypoint for all dofs at horizonLength - 1
    for(int i = 0; i < dof; i++){
        keypoints[horizon - 1].push_back(i);
    }
}

void KeypointGenerator::GenerateJerkProfile(const std::vector<MatrixXd> &trajectory_states){

    MatrixXd jerk(dof, 1);

    MatrixXd state1(trajectory_states[0].rows(), 1);
    MatrixXd state2(trajectory_states[0].rows(), 1);
    MatrixXd state3(trajectory_states[0].rows(), 1);

    for(int t = 0; t < horizon - 2; t++){
        state1 = trajectory_states[t];
        state2 = trajectory_states[t + 1];
        state3 = trajectory_states[t + 2];

        MatrixXd accell1 = state2 - state1;
        MatrixXd accell2 = state3 - state2;

        for(int j = 0; j < dof; j++){
            jerk(j, 0) = abs(accell2(j+dof, 0) - accell1(j+dof, 0));
        }

        jerk_profile[t] = jerk;
    }
}

std::vector<MatrixXd> KeypointGenerator::GenerateAccellerationProfile(int horizon, std::vector<MatrixXd> trajectory_states) {
    int dof = trajectory_states[0].rows() / 2;
    MatrixXd accel(dof, 1);

    MatrixXd state1(trajectory_states[0].rows(), 1);
    MatrixXd state2(trajectory_states[0].rows(), 1);

    std::vector<MatrixXd> accelleration_profile;

    for(int i = 0; i < horizon - 1; i++){
        state1 = trajectory_states[i];
        state2 = trajectory_states[i + 1];

        MatrixXd accell_states = state2 - state1;

        for(int j = 0; j < dof; j++){
            accel(j, 0) = accell_states(j + dof, 0);
        }

        accelleration_profile.push_back(accel);
    }

    return accelleration_profile;
}

void KeypointGenerator::GenerateVelocityProfile(const std::vector<MatrixXd> &trajectory_states) {
    int dof = trajectory_states[0].rows() / 2;

    MatrixXd velocities(dof, 1);

    for(int t = 0; t < horizon; t++){
        for(int i = 0; i < dof; i++){
            velocities(i, 0) = trajectory_states[t](i+dof, 0);
        }
        velocity_profile[t] = velocities;
    }
}

void KeypointGenerator::UpdateLastPercentageDerivatives(std::vector<std::vector<int>> &keypoints){
    last_percentages = ComputePercentageDerivatives(keypoints);
}

std::vector<double> KeypointGenerator::ComputePercentageDerivatives(std::vector<std::vector<int>> &keypoints){
    std::vector<int> dof_count = std::vector<int>(dof, 0);
    std::vector<double> percentages = std::vector<double>(dof, 0);
    for(int t = 0; t < horizon - 1; t++){
        for(int i = 0; i < keypoints[t].size(); i++){
            for(int j = 0; j < dof; j++){
                // if match between keypoint and dof
                if(j == keypoints[t][i]){
                    dof_count[j]++;
                    break;
                }
            }
        }
    }

//    std::cout << "dof count: ";
    for(int i = 0; i < dof; i++){
        last_num_keypoints[i] = dof_count[i];
        percentages[i] = ((double)dof_count[i] / (double)(horizon)) * 100;
//        std::cout << dof_count[i] << " ";
    }
//    std::cout << std::endl;

    return percentages;
}

std::vector<int> KeypointGenerator::ConvertPercentagesToNumKeypoints(const std::vector<double> &percentages){
    std::vector<int> num_keypoints = std::vector<int>(dof, 0);
    for(int i = 0; i < dof; i++){
        num_keypoints[i] = (int)round((percentages[i] / 100) * horizon);
    }
    return num_keypoints;
}

std::vector<double> KeypointGenerator::ConvertNumKeypointsToPercentages(const std::vector<int> &num_keypoints){
    std::vector<double> percentages = std::vector<double>(dof, 0);
    for(int i = 0; i < dof; i++){
        percentages[i] = ((double)num_keypoints[i] / (double)horizon) * 100;
    }
    return percentages;
}

void KeypointGenerator::ResetCache(){
    keypoints_computed = false;
}
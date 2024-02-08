#include "KeyPointGenerator.h"

KeypointGenerator::KeypointGenerator(std::shared_ptr<Differentiator> _differentiator,
                                     std::shared_ptr<PhysicsSimulator> _physics_simulator,
                                     int _dof) {
    differentiator = _differentiator;
    physics_simulator = _physics_simulator;

    dof = _dof;

    // Allocate static arrays
    last_percentages.reserve(dof);

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

std::vector<std::vector<int>> KeypointGenerator::GenerateKeyPoints(int horizon, std::vector<MatrixXd> trajectory_states, std::vector<MatrixXd> trajec_controls,
                                                                   std::vector<MatrixXd> &A, std::vector<MatrixXd> &B){

    if(current_keypoint_method.auto_adjust && auto_adjust_initialisation_occured){
        // Perform some initialisation here by looping over states trajectory.

    }

    // Loop through the trajectory and decide what indices should be evaluated via finite differencing
    std::vector<std::vector<int>> keypoints;

    std::vector<int> one_row;
    for(int i = 0; i < dof; i++){
        one_row.push_back(i);
    }
    keypoints.push_back(one_row);

    // For the trivial case of horizon == 2, we only need to evaluate the start and end points
    if(horizon == 2){
        keypoints.push_back(one_row);
        return keypoints;
    }

    if(current_keypoint_method.name == "setInterval"){
        keypoints = GenerateKeyPointsSetInterval(horizon);

    }
    else if(current_keypoint_method.name == "adaptive_jerk"){
        std::vector<MatrixXd> jerk_profile = GenerateJerkProfile(horizon, trajectory_states);
        keypoints = GenerateKeyPointsAdaptive(horizon, jerk_profile);
    }
    else if(current_keypoint_method.name == "adaptive_accel"){
        std::vector<MatrixXd> acceleration_profile = GenerateAccellerationProfile(horizon, trajectory_states);
        keypoints = GenerateKeyPointsAdaptive(horizon, acceleration_profile);
    }
    else if(current_keypoint_method.name == "iterative_error"){
        computed_keypoints.clear();
        physics_simulator->initModelForFiniteDifferencing();
        keypoints = GenerateKeyPointsIteratively(horizon, trajectory_states, A, B);
        physics_simulator->resetModelAfterFiniteDifferencing();

    }
    else if(current_keypoint_method.name == "magvel_change"){
        std::vector<MatrixXd> velocity_profile = GenerateVelocityProfile(horizon, trajectory_states);
        keypoints = GenerateKeyPointsVelocityChange(horizon, velocity_profile);
    }
    else{
        std::cout << "ERROR: keyPointsMethod not recognised \n";
    }

    // Enforce that last time step is evaluated for all dofs, otherwise nothing to interpolate to
    keypoints.back().clear();
    for(int i = 0; i < dof; i++){
        keypoints.back().push_back(i);
    }

    UpdateLastPercentageDerivatives(keypoints, horizon);

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

void KeypointGenerator::AdjustKeyPointMethod(double old_cost, double new_cost, int horizon,
                                             std::vector<MatrixXd> &trajectory_states){


    // If we are not in auto-adjust mode, then return
    if(current_keypoint_method.auto_adjust == false){
        return;
    }

    double min_percentage = (2.0 / horizon) * 100.0;

    std::vector<double> desired_derivative_percentages = std::vector<double>(dof, 0.0);

    // Calculate new desired derivative percentages based on previous ones

    // If the cost decreased, lets make the key-points more sparse
    if(new_cost < old_cost){
        // Do something basic to begin
        for(int i = 0; i < dof; i++){
            current_keypoint_method.velocity_change_thresholds[i] =
                    current_keypoint_method.velocity_change_thresholds[i] * 1.1;
        }

    }
    else{

        for(int i = 0; i < dof; i++) {
            current_keypoint_method.velocity_change_thresholds[i] =
                    current_keypoint_method.velocity_change_thresholds[i] / 1.1;
        }

//        current_keypoint_method.min_N = std::min(100, current_keypoint_method.min_N - 1);
//        if(current_keypoint_method.min_N == 0){
//            current_keypoint_method.min_N = 1;
//        }
    }

}

std::vector<std::vector<int>> KeypointGenerator::GenerateKeyPointsSetInterval(int horizon){
    std::vector<std::vector<int>> keypoints;

    for(int i = 0; i < horizon; i++){

        if(i % current_keypoint_method.min_N == 0){
            std::vector<int> one_row;
            for(int j = 0; j < dof; j++){
                one_row.push_back(j);
            }
            keypoints.push_back(one_row);
        }
        else{
            std::vector<int> one_row;
            keypoints.push_back(one_row);
        }
    }

    return keypoints;
}

std::vector<std::vector<int>> KeypointGenerator::GenerateKeyPointsAdaptive(int horizon, std::vector<MatrixXd> trajec_profile) {
    int dof = trajec_profile[0].rows() / 2;

    std::vector<std::vector<int>> keypoints;

    for(int t = 0; t < horizon; t++){
        keypoints.push_back(std::vector<int>());
    }

    for(int i = 0; i < dof; i++){
        keypoints[0].push_back(i);
    }

    int last_indices[dof];
    for(int i = 0; i < dof; i++){
        last_indices[i] = 0;
    }

    // Loop over the trajectory
    for(int j = 0; j < dof; j++){
        for(int i = 0; i < trajec_profile.size(); i++){

            if((i - last_indices[j]) >= current_keypoint_method.min_N) {
                // Check if the jerk is above the threshold
                if (trajec_profile[i](j, 0) > current_keypoint_method.jerk_thresholds[j]) {
                    keypoints[i].push_back(j);
                    last_indices[j] = i;
                }
            }

            if((i - last_indices[j]) >= current_keypoint_method.max_N){
                keypoints[i].push_back(j);
                last_indices[j] = i;
            }
        }
    }
    return keypoints;
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
        differentiator->getDerivatives(A[indices.start_index], B[indices.start_index], cols, blank1, blank2, blank3, blank4, false, indices.start_index, false, tid);
        computed_keypoints[dof_index].push_back(indices.start_index);
    }

    if(!mid_index_computed){
        differentiator->getDerivatives(A[mid_index], B[mid_index], cols, blank1, blank2, blank3, blank4, false, mid_index, false, tid);
        computed_keypoints[dof_index].push_back(mid_index);
    }

    if(!end_index_computed){
        differentiator->getDerivatives(A[indices.end_index], B[indices.end_index], cols, blank1, blank2, blank3, blank4, false, indices.end_index, false, tid);
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

std::vector<std::vector<int>> KeypointGenerator::GenerateKeyPointsVelocityChange(int horizon, std::vector<MatrixXd> velocity_profile) {
    int dof = velocity_profile[0].rows();

    std::vector<std::vector<int>> keypoints;

    for(int t = 0; t < horizon; t++){
        keypoints.push_back(std::vector<int>());
    }

    for(int i = 0; i < dof; i++){
        keypoints[0].push_back(i);
    }

    // Keeps track of interval from last keypoint for this dof
    std::vector<int> last_keypoint_counter = std::vector<int>(dof, 0);
    std::vector<double> last_vel_value = std::vector<double>(dof, 0);
    std::vector<double> last_vel_direction = std::vector<double>(dof, 0);

    for(int i = 0; i < dof; i++){
        last_vel_value[i] = velocity_profile[0](i, 0);
    }

    // Loop over the velocity dofs
    for(int i = 0; i < dof; i++){
        // Loop over the horizon
        for(int t = 1; t < horizon; t++){

            last_keypoint_counter[i]++;
            double current_vel_direction = velocity_profile[t](i, 0) - velocity_profile[t - 1](i, 0);
            double current_vel_change_since_last_keypoint = velocity_profile[t](i, 0) - last_vel_value[i];

            // If the vel change is above the required threshold
            if(last_keypoint_counter[i] >= current_keypoint_method.min_N){
                if(abs(current_vel_change_since_last_keypoint) > current_keypoint_method.velocity_change_thresholds[i]){
                    keypoints[t].push_back(i);
                    last_vel_value[i] = velocity_profile[t](i, 0);
                    last_keypoint_counter[i] = 0;
                    continue;
                }
            }

//            cout << " after check mag change" << endl;

            // If the interval is greater than min_N
            if(last_keypoint_counter[i] >= current_keypoint_method.min_N){
                // If the direction of the velocity has changed
                if(current_vel_direction * last_vel_direction[i] < 0){
                    keypoints[t].push_back(i);
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
                keypoints[t].push_back(i);
                last_vel_value[i] = velocity_profile[t](i, 0);
                last_keypoint_counter[i] = 0;
                continue;
            }
        }
    }

    // Enforce last keypoint for all dofs at horizonLength - 1
    for(int i = 0; i < dof; i++){
        keypoints[horizon - 1].push_back(i);
    }

    return keypoints;
}

std::vector<MatrixXd> KeypointGenerator::GenerateJerkProfile(int horizon, std::vector<MatrixXd> trajectory_states) {

    int dof = trajectory_states[0].rows() / 2;
    MatrixXd jerk(dof, 1);

    MatrixXd state1(trajectory_states[0].rows(), 1);
    MatrixXd state2(trajectory_states[0].rows(), 1);
    MatrixXd state3(trajectory_states[0].rows(), 1);

    std::vector<MatrixXd> jerk_profile;

    for(int i = 0; i < horizon - 2; i++){
        state1 = trajectory_states[i];
        state2 = trajectory_states[i + 1];
        state3 = trajectory_states[i + 2];

        MatrixXd accell1 = state2 - state1;
        MatrixXd accell2 = state3 - state2;

        for(int j = 0; j < dof; j++){
            jerk(j, 0) = abs(accell2(j+dof, 0) - accell1(j+dof, 0));
        }

        jerk_profile.push_back(jerk);
    }

    return jerk_profile;
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

std::vector<MatrixXd> KeypointGenerator::GenerateVelocityProfile(int horizon, std::vector<MatrixXd> trajectory_states) {
    int dof = trajectory_states[0].rows() / 2;

    std::vector<MatrixXd> velocity_profile;
    MatrixXd velocities(dof, 1);

    for(int t = 0; t < horizon; t++){

        for(int i = 0; i < dof; i++){
            velocities(i, 0) = trajectory_states[t](i+dof, 0);
        }

        velocity_profile.push_back(velocities);
    }

    return velocity_profile;
}

void KeypointGenerator::UpdateLastPercentageDerivatives(std::vector<std::vector<int>> keypoints, int horizon){
    std::vector<int> dof_count = std::vector<int>(dof, 0);
    for(int t = 0; t < keypoints.size(); t++){
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

    for(int i = 0; i < dof; i++){
        last_percentages[i] = ((double)dof_count[i] / (double)(horizon)) * 100;
    }
}
#include "StdInclude.h"
#include "FileHandler.h"
#include "Visualiser.h"
#include "MuJoCoHelper.h"

// --------------------- different scenes -----------------------
#include "ModelTranslator/Acrobot.h"
#include "ModelTranslator/Pentabot.h"
#include "ModelTranslator/PistonBlock.h"

#include "ModelTranslator/Reaching.h"

#include "ModelTranslator/TwoDPushing.h"
#include "ModelTranslator/ThreeDPushing.h"
#include "ModelTranslator/BoxFlick.h"
#include "ModelTranslator/BoxSweep.h"

#include "ModelTranslator/Walker.h"
//#include "Hopper.h"
//#include "humanoid.h"

// --------------------- different optimisers -----------------------
#include "Optimiser/iLQR.h"
#include "Optimiser/iLQR_SVR.h"
//#include "Optimiser/PredictiveSampling.h"
//#include "Optimiser/GradDescent.h"

//----------------------- Testing methods ---------------------------
#include "GenTestingData.h"

// --------------------- other -----------------------
#include <mutex>

// --------------------- Global class instances --------------------------------
std::shared_ptr<ModelTranslator> activeModelTranslator;
std::shared_ptr<Differentiator> activeDifferentiator;
std::shared_ptr<Optimiser> activeOptimiser;
std::shared_ptr<iLQR> iLQROptimiser;
std::shared_ptr<iLQR_SVR> iLQR_SVR_Optimiser;
//std::shared_ptr<PredictiveSampling> stompOptimiser;
//std::shared_ptr<GradDescent> gradDescentOptimiser;
std::shared_ptr<Visualiser> activeVisualiser;
std::shared_ptr<FileHandler> yamlReader;

bool async_mpc;
std::string task;
bool mpc_visualise = true;
bool playback = true;

std::mutex mtx;
std::condition_variable cv;

bool apply_next_control = false;

int assign_task();

void InitControls();
void OpenLoopOptimisation(int opt_horizon);
void MPCUntilComplete(int OPT_HORIZON);

void AsyncMPC();
void worker();

double avg_opt_time, avg_percent_derivs, avg_time_derivs, avg_time_bp, avg_time_fp;

bool stop_mpc = false;

int main(int argc, char **argv) {

    // Expected arguments
    // 1. Program name
    // 2. Task name
    if(argc < 2){
        std::cout << "No task name provided, exiting" << endl;
        return -1;
    }

    std::string configFileName = argv[1];
//    std::cout << "config file name: " << configFileName << endl;

    // Optional arguments
    // 3. key-point method (only used for generating testing data)
    // Only used for generating testing data for different key-point methods
//    if(argc > 2){
//        for (int i = 1; i < argc; i++) {
//            testingMethods.push_back(argv[i]);
//        }
//    }

    std::string optimiser;
    std::string runMode;

    std::string taskInitMode;

    yamlReader = std::make_shared<FileHandler>();
    yamlReader->readSettingsFile("/generalConfigs/" + configFileName + ".yaml");
    optimiser = yamlReader->optimiser;
    runMode = yamlReader->project_run_mode;
    task = yamlReader->taskName;
    taskInitMode = yamlReader->taskInitMode;
    async_mpc = yamlReader->async_mpc;

    // Instantiate model translator as specified by the config file.
    if(assign_task() == EXIT_FAILURE){
        return EXIT_FAILURE;
    }

    activeDifferentiator = std::make_shared<Differentiator>(activeModelTranslator, activeModelTranslator->MuJoCo_helper);

    for(int j = 0; j < 5; j++){
        mj_step(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->master_reset_data);
    }
    activeModelTranslator->MuJoCo_helper->AppendSystemStateToEnd(activeModelTranslator->MuJoCo_helper->master_reset_data);

    //Instantiate my visualiser
    activeVisualiser = std::make_shared<Visualiser>(activeModelTranslator);

    // Setup the initial horizon, based on open loop or mpc method
    int opt_horizon = 0;
    if(runMode == "MPC_Until_Complete"){
        opt_horizon = activeModelTranslator->MPC_horizon;
    }
    else{
        opt_horizon = activeModelTranslator->openloop_horizon;
    }

    // Choose an Optimiser
    if(optimiser == "iLQR"){
        iLQROptimiser = std::make_shared<iLQR>(activeModelTranslator,
                                               activeModelTranslator->MuJoCo_helper,
                                               activeDifferentiator,
                                               opt_horizon, activeVisualiser, yamlReader);
        activeOptimiser = iLQROptimiser;
    }
    else if(optimiser == "iLQR_SVR"){
        iLQR_SVR_Optimiser = std::make_shared<iLQR_SVR>(activeModelTranslator,
                                                        activeModelTranslator->MuJoCo_helper,
                                                        activeDifferentiator,
                                                        opt_horizon, activeVisualiser, yamlReader);
        activeOptimiser = iLQR_SVR_Optimiser;
    }
//    else if(optimiser == "PredictiveSampling"){
//        stompOptimiser = std::make_shared<PredictiveSampling>(activeModelTranslator,
//                                                              activeModelTranslator->MuJoCo_helper,
//                                                              yamlReader, activeDifferentiator,
//                                                              opt_horizon, 8);
//        activeOptimiser = stompOptimiser;
//    }
//    else if(optimiser == "GradDescent"){
//        gradDescentOptimiser = std::make_shared<GradDescent>(activeModelTranslator,
//                                                             activeModelTranslator->MuJoCo_helper,
//                                                             activeDifferentiator, activeVisualiser,
//                                                             opt_horizon, yamlReader);
//        activeOptimiser = gradDescentOptimiser;
//    }
    else{
        std::cerr << "invalid Optimiser selected, exiting" << endl;
        return -1;
    }

    if(runMode == "Generate_dynamics_data"){
        GenTestingData myTestingObject(iLQROptimiser, activeModelTranslator,
                                       activeDifferentiator, activeVisualiser, yamlReader);

        return myTestingObject.GenerateDynamicsDerivsData(100, 4);
    }

    if(runMode == "Generate_test_scenes"){
        GenTestingData myTestingObject(iLQROptimiser, activeModelTranslator,
                                       activeDifferentiator, activeVisualiser, yamlReader);

        return myTestingObject.GenerateTestScenes(100);
    }

    if(runMode == "Generate_openloop_data"){
        GenTestingData myTestingObject(activeOptimiser, activeModelTranslator,
                                       activeDifferentiator, activeVisualiser, yamlReader);

        int task_horizon = activeModelTranslator->openloop_horizon;
        int re_add_dofs;
        double K_threshold;

        if(argc > 2){
            task_horizon = std::atoi(argv[2]);
        }

        if(argc > 3){
            re_add_dofs = std::atoi(argv[3]);
            K_threshold = std::atof(argv[4]);

            myTestingObject.SetParamsiLQR_SVR(re_add_dofs, K_threshold);
            std::cout << "set optimiser parameters \n";
        }

        return myTestingObject.GenDataOpenloopOptimisation(task_horizon);

    }
    if(runMode == "Generate_asynchronus_mpc_data"){
        GenTestingData myTestingObject(activeOptimiser, activeModelTranslator,
                                       activeDifferentiator, activeVisualiser, yamlReader);

        int task_horizon = 60;
        int task_timeout = 2000;
        int re_add_dofs;
        double K_threshold;

        if(argc > 2){
            task_horizon = std::atoi(argv[2]);
        }

        if(argc > 3){
            task_timeout = std::atoi(argv[3]);
        }

        if(argc > 4){
            re_add_dofs = std::atoi(argv[4]);
            K_threshold = std::atof(argv[5]);

            myTestingObject.SetParamsiLQR_SVR(re_add_dofs, K_threshold);
            std::cout << "set optimiser parameters \n";
        }

        return myTestingObject.GenDataAsyncMPC(task_horizon, task_timeout);
    }

    // random start and goal state
    std::string taskPrefix = activeModelTranslator->model_name;
    if(taskInitMode == "random"){
        activeModelTranslator->GenerateRandomGoalAndStartState();
    }
    else if(taskInitMode == "fromCSV"){
        yamlReader->loadTaskFromFile(taskPrefix, yamlReader->csvRow, activeModelTranslator->full_state_vector);
        activeModelTranslator->full_state_vector.Update();
        activeModelTranslator->current_state_vector = activeModelTranslator->full_state_vector;
        activeModelTranslator->UpdateSceneVisualisation();
    }

    // Initialise the system state from desired mechanism
    // Set goal pose here maybe???
    activeModelTranslator->InitialiseSystemToStartState(activeModelTranslator->MuJoCo_helper->master_reset_data);

    // Methods of control / visualisation
    if(runMode == "Init_controls"){
        cout << "SHOWING INIT CONTROLS MODE \n";
        InitControls();
    }
    else if(runMode == "Optimise_once"){
        cout << "OPTIMISE TRAJECTORY ONCE AND DISPLAY MODE \n";
        activeOptimiser->verbose_output = true;
        OpenLoopOptimisation(activeModelTranslator->openloop_horizon);
    }
    else if(runMode == "MPC_until_completion"){
        cout << "MPC UNTIL TASK COMPLETE MODE \n";
        AsyncMPC();
    }
    else{
        cout << "INVALID MODE OF OPERATION OF PROGRAM \n";

        // Testing code for rotating object around in mid air and testing ost function etc.
//        pose_6 object;
//        object.position[0] = 0.8;
//        object.position[1] = 0.0;
//        object.position[2] = 0.5;
//
//        object.orientation[0] = 0.0;
//        object.orientation[1] = 0.0;
//        object.orientation[2] = 0.0;
//
//        m_point axis_test = {0, 0.5, 0.1};
//        m_quat test = axis2Quat(axis_test);
//        axis_test = quat2Axis(test);
//        std::cout << "quat: " << test << std::endl;
//        std::cout << "axis test: " << axis_test << "\n";
//
//        activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
//        activeModelTranslator->MuJoCo_helper->SetBodyPoseAngle("goal", object, activeModelTranslator->MuJoCo_helper->vis_data);
//
//        while(activeVisualiser->windowOpen()){
//            activeModelTranslator->MuJoCo_helper->ForwardSimulator(activeModelTranslator->MuJoCo_helper->vis_data);
//            activeVisualiser->render("Test");
//        }
        // --------------------------------------------------------------------------------------------------------------

        std::vector<double> time_derivs_full;
        std::vector<double> time_bp_full;
        std::vector<double> time_derivs_reduced;
        std::vector<double> time_bp_reduced;

        int dofs_full, dofs_reduced;
        int N = 10;

        dofs_full = activeModelTranslator->full_state_vector.dof;

        iLQROptimiser = std::make_shared<iLQR>(activeModelTranslator, activeModelTranslator->MuJoCo_helper, activeDifferentiator, opt_horizon, activeVisualiser, yamlReader);

//        MatrixXd state_vector = activeModelTranslator->ReturnStateVectorQuaternions(activeModelTranslator->MuJoCo_helper->master_reset_data);
//        std::cout << "size of state vector quaternion: " << state_vector.rows() << std::endl;
//        activeModelTranslator->current_state_vector.Update();
//        std::cout << "num dofs: " << activeModelTranslator->current_state_vector.dof << " num dofs quat: " << activeModelTranslator->current_state_vector.dof_quat << std::endl;

        std::cout << "state vector: ";
        for(const auto & state_name : activeModelTranslator->current_state_vector.state_names){
            std::cout << state_name << " ";
        }
        std::cout << "\n";
        std::cout << "size of state vector before: " << activeModelTranslator->current_state_vector.dof << " \n";

        // remove elements
        std::vector<std::string> remove = {"panda0_joint1", "goal_x", "obstacle_1_x"};
        activeModelTranslator->UpdateCurrentStateVector(remove, false);

        std::cout << "state vector: ";
        for(const auto & state_name : activeModelTranslator->current_state_vector.state_names){
            std::cout << state_name << " ";
        }
        std::cout << "\n";
        std::cout << "size of state vector after: " << activeModelTranslator->current_state_vector.dof << " \n";


//        std::vector<MatrixXd> initSetupControls = activeModelTranslator->CreateInitSetupControls(1000);
//        activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->master_reset_data, activeModelTranslator->MuJoCo_helper->main_data);
//
//        std::vector<MatrixXd> U_init;
//        U_init = activeModelTranslator->CreateInitOptimisationControls(opt_horizon);
//        activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], activeModelTranslator->MuJoCo_helper->master_reset_data);
//
//
//        iLQROptimiser->RolloutTrajectory(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], true,  U_init);
//        std::cout << "rollout done \n";
//
//        // Compute derivatives
//        auto timer_start = std::chrono::high_resolution_clock::now();
//
//        for(int i = 0; i < N; i++){
//            timer_start = std::chrono::high_resolution_clock::now();
//            iLQROptimiser->GenerateDerivatives();
//            time_derivs_full.push_back(std::chrono::duration_cast<std::chrono::milliseconds>(
//                    std::chrono::high_resolution_clock::now() - timer_start).count());
//        }
//
//        for(int i = 0; i < N; i++){
//            timer_start = std::chrono::high_resolution_clock::now();
//            iLQROptimiser->BackwardsPassQuuRegularisation();
//            time_bp_full.push_back(std::chrono::duration_cast<std::chrono::milliseconds>(
//                    std::chrono::high_resolution_clock::now() - timer_start).count());
//        }
//
//
//        // Remove elements from task
//        std::vector<std::string> remove_elements = {"goal_y", "goal_pitch", "goal_roll", "goal_yaw",
//                                                    "mediumCylinder_y", "mediumCylinder_pitch", "mediumCylinder_roll", "mediumCylinder_yaw",
//                                                    "bigBox_y", "bigBox_pitch", "bigBox_roll", "bigBox_yaw",
//                                                    "obstacle1_y", "obstacle1_pitch", "obstacle1_roll", "obstacle1_yaw",
//                                                    "obstacle2_y", "obstacle2_pitch", "obstacle2_roll", "obstacle2_yaw",
//                                                    "obstacle3_y", "obstacle3_pitch", "obstacle3_roll", "obstacle3_yaw",
//                                                    "obstacle4_y", "obstacle4_pitch", "obstacle4_roll", "obstacle4_yaw",
//                                                    "obstacle5_y", "obstacle5_pitch", "obstacle5_roll", "obstacle5_yaw",};
//        activeModelTranslator->UpdateCurrentStateVector(remove_elements, false);
//        dofs_reduced = activeModelTranslator->dof;
//
//        // resize optimiser state
//        iLQROptimiser->Resize(activeModelTranslator->dof, activeModelTranslator->num_ctrl, iLQROptimiser->horizon_length);
//
//        iLQROptimiser->RolloutTrajectory(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], true,  U_init);
//        std::cout << "2nd rollout done \n";
//
//        // Compute derivatives
//        for(int i = 0; i < N; i++){
//            timer_start = std::chrono::high_resolution_clock::now();
//            iLQROptimiser->GenerateDerivatives();
//            time_derivs_reduced.push_back(std::chrono::duration_cast<std::chrono::milliseconds>(
//                    std::chrono::high_resolution_clock::now() - timer_start).count());
//        }
//
//        // Compute backwards pass
//        for(int i = 0; i < N; i++){
//            timer_start = std::chrono::high_resolution_clock::now();
//            iLQROptimiser->BackwardsPassQuuRegularisation();
//            time_bp_reduced.push_back(std::chrono::duration_cast<std::chrono::milliseconds>(
//                    std::chrono::high_resolution_clock::now() - timer_start).count());
//        }
//
//        double avg_time_derivs_full, avg_time_derivs_reduced, avg_time_bp_full, avg_time_bp_reduced;
//
//        for(int i = 0; i < N; i++){
//            avg_time_derivs_full += time_derivs_full[i];
//            avg_time_derivs_reduced += time_derivs_reduced[i];
//            avg_time_bp_full += time_bp_full[i];
//            avg_time_bp_reduced += time_bp_reduced[i];
//        }
//
//        avg_time_derivs_full /= N;
//        avg_time_derivs_reduced /= N;
//        avg_time_bp_full /= N;
//        avg_time_bp_reduced /= N;
//
//        std::cout << "-------- Results of state vector dimensionality reduction ----------- \n";
//        std::cout << "num dofs in full state vector: " << dofs_full << "\n";
//        std::cout << "time of derivs for full state vector: " << avg_time_derivs_full << " ms \n";
//        std::cout << "time of bp for full state vector: " << avg_time_bp_full << " ms \n";
//        std::cout << "num dofs in reduced state vector: " << dofs_reduced << "\n";
//        std::cout << "time of derivs for reduced state vector: " << avg_time_derivs_reduced << " ms\n";
//        std::cout << "time of bp for reduced state vector: " << avg_time_bp_reduced << " ms\n";
//
//        double percentage_decrease_derivs = (1 - (avg_time_derivs_reduced / avg_time_derivs_full)) * 100.0;
//        double percentage_decrease_bp = (1 - (avg_time_bp_reduced / avg_time_bp_full)) * 100.0;
//        double dof_percentage_decrease = (1 - ((double)dofs_reduced / (double)dofs_full)) * 100.0;
//        std::cout << "percent decrease of derivatives: " << percentage_decrease_derivs << "\n";
//        std::cout << "percent decrease of backwards pass: " << percentage_decrease_bp << "\n";
//        std::cout << "expected percentage decrease: " << dof_percentage_decrease << "\n";

        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

void InitControls(){
    int setupHorizon = 1000;
    int optHorizon = 2500;
    int controlCounter = 0;
    int visualCounter = 0;

    std::vector<MatrixXd> initControls;

    std::vector<MatrixXd> initSetupControls = activeModelTranslator->CreateInitSetupControls(setupHorizon);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->master_reset_data, activeModelTranslator->MuJoCo_helper->main_data);
    std::vector<MatrixXd> initOptimisationControls = activeModelTranslator->CreateInitOptimisationControls(optHorizon);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);

    //Stitch setup and optimisation controls together
    initControls.insert(initControls.end(), initOptimisationControls.begin(), initOptimisationControls.end());

    while(activeVisualiser->windowOpen()){

        activeModelTranslator->SetControlVector(initControls[controlCounter], activeModelTranslator->MuJoCo_helper->main_data,
                                                activeModelTranslator->current_state_vector);
        mj_step(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->main_data);


        controlCounter++;
        visualCounter++;

        if(controlCounter >= initControls.size()){
            controlCounter = 0;
            activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
        }

        if(visualCounter > 5){
            visualCounter = 0;
            activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->main_data);
            activeModelTranslator->MuJoCo_helper->ForwardSimulator(activeModelTranslator->MuJoCo_helper->vis_data);
            activeVisualiser->render("show init controls");
        }
    }
}

void OpenLoopOptimisation(int opt_horizon){
    int controlCounter = 0;
    int visualCounter = 0;
    bool showFinalControls = true;
    const char* label = "Final trajectory after optimisation";

    std::cout << "opt horizon is" << opt_horizon << std::endl;

    std::vector<MatrixXd> initControls;
    std::vector<MatrixXd> finalControls;

    std::vector<MatrixXd> initSetupControls = activeModelTranslator->CreateInitSetupControls(1000);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->master_reset_data, activeModelTranslator->MuJoCo_helper->main_data);

    std::vector<MatrixXd> initOptimisationControls = activeModelTranslator->CreateInitOptimisationControls(opt_horizon);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], activeModelTranslator->MuJoCo_helper->master_reset_data);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->master_reset_data);

    auto start = high_resolution_clock::now();
    std::vector<MatrixXd> optimisedControls = activeOptimiser->Optimise(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], initOptimisationControls, yamlReader->maxIter, yamlReader->minIter, opt_horizon);
    auto stop = high_resolution_clock::now();
    auto linDuration = duration_cast<microseconds>(stop - start);

    // Stitch together setup controls with init control + optimised controls
    initControls.insert(initControls.end(), initOptimisationControls.begin(), initOptimisationControls.end());
    finalControls.insert(finalControls.end(), optimisedControls.begin(), optimisedControls.end());

    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);

    while(activeVisualiser->windowOpen()){

        if(showFinalControls){
            activeModelTranslator->SetControlVector(finalControls[controlCounter], activeModelTranslator->MuJoCo_helper->main_data,
                                                    activeModelTranslator->current_state_vector);
        }
        else{
            activeModelTranslator->SetControlVector(initControls[controlCounter], activeModelTranslator->MuJoCo_helper->main_data,
                    activeModelTranslator->current_state_vector);
        }

        mj_step(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->main_data);

        controlCounter++;
        visualCounter++;

        if(controlCounter >= finalControls.size()){
            controlCounter = 0;
            activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
            showFinalControls = !showFinalControls;
            if(showFinalControls){
                label = "Final trajectory after optimisation";
            }
            else{
                label = "Initial trajectory before optimisation";
            }
        }

        if(visualCounter >= 5){
            visualCounter = 0;
            activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->main_data);
            activeModelTranslator->MuJoCo_helper->ForwardSimulator(activeModelTranslator->MuJoCo_helper->vis_data);
            activeVisualiser->render(label);
        }
    }
}

void worker(){
    MPCUntilComplete(activeModelTranslator->MPC_horizon);
}

void AsyncMPC(){

    // Setup the task
    std::vector<MatrixXd> initSetupControls = activeModelTranslator->CreateInitSetupControls(1000);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->master_reset_data, activeModelTranslator->MuJoCo_helper->main_data);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->master_reset_data);


    // Whether Optimiser will output useful information
    activeOptimiser->verbose_output = true;
    // Visualise MPC trajectory live
    mpc_visualise = true;

    std::thread MPC_controls_thread;
    MPC_controls_thread = std::thread(&worker);

    int vis_counter = 0;
    MatrixXd next_control;

    // timer variables
    std::chrono::steady_clock::time_point begin;
    std::chrono::steady_clock::time_point end;

    int MAX_TASK_TIME = 2500;
    int task_time = 0;

    while(task_time < MAX_TASK_TIME){
        begin = std::chrono::steady_clock::now();

        if(async_mpc || (!async_mpc && apply_next_control)){

            if(!async_mpc){
                apply_next_control = false;
            }

            if(activeVisualiser->current_control_index < activeVisualiser->controlBuffer.size()){

                next_control = activeVisualiser->controlBuffer[activeVisualiser->current_control_index];
                // Increment the current control index
                activeVisualiser->current_control_index++;
            }
            else{
                // TODO make this grav compensation controls?
                std::vector<double> grav_compensation;
                std::string robot_name = activeModelTranslator->current_state_vector.robots[0].name;
                activeModelTranslator->MuJoCo_helper->GetRobotJointsGravityCompensationControls(robot_name, grav_compensation,
                                                                                                activeModelTranslator->MuJoCo_helper->vis_data);
                MatrixXd empty_control(activeModelTranslator->current_state_vector.num_ctrl, 1);
//                empty_control.setZero();
                for(int i = 0; i < activeModelTranslator->current_state_vector.num_ctrl; i++){
                    empty_control(i) = grav_compensation[i];
                }
                next_control = empty_control;
            }

//            std::cout << "next control is: " << next_control.transpose() << "\n";

            // Store latest control and state in a replay buffer
            activeVisualiser->trajectory_controls.push_back(next_control);
            MatrixXd next_state = activeModelTranslator->ReturnStateVector(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->full_state_vector);
            activeVisualiser->trajectory_states.push_back(next_state);

            // Set the latest control
            activeModelTranslator->SetControlVector(next_control, activeModelTranslator->MuJoCo_helper->vis_data,
                                                    activeModelTranslator->current_state_vector);

            // Update the simulation
            mj_step(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->vis_data);
            task_time++;

            // Check if task complete
            double dist;
            if(activeModelTranslator->TaskComplete(activeModelTranslator->MuJoCo_helper->vis_data, dist)){
                cout << "task complete - dist: " << dist  << endl;
                break;
            }
        }

        // Update the visualisation
        // Unsure why rendering every time causes it to lag so much more???
        vis_counter++;
        if(vis_counter > 5){
            activeVisualiser->render("live-MPC");
            vis_counter = 0;
        }

        end = std::chrono::steady_clock::now();
        // time taken
        auto time_taken = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();

        // compare how long we took versus the timestep of the model
        int difference_ms = (activeModelTranslator->MuJoCo_helper->ReturnModelTimeStep() * 1000) - (time_taken / 1000.0f) + 1;

        if(difference_ms > 0) {
//                difference_ms = 20;
            std::this_thread::sleep_for(std::chrono::milliseconds(difference_ms));
        }
    }


    // Stop MPC thread
    {
        std::unique_lock<std::mutex> lock(mtx);
        stop_mpc = true;
    }

    MPC_controls_thread.join();

    double cost = 0.0f;
        activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
        for(int i = 0; i < activeVisualiser->trajectory_states.size(); i++){
            activeModelTranslator->SetControlVector(activeVisualiser->trajectory_controls[i], activeModelTranslator->MuJoCo_helper->vis_data,
                                                    activeModelTranslator->current_state_vector);
            activeModelTranslator->SetStateVector(activeVisualiser->trajectory_states[i], activeModelTranslator->MuJoCo_helper->vis_data,
                                                  activeModelTranslator->full_state_vector);
            activeModelTranslator->MuJoCo_helper->ForwardSimulator(activeModelTranslator->MuJoCo_helper->vis_data);
            cost += activeModelTranslator->CostFunction(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->full_state_vector, false);

        }

    std::cout << "final cost of entire MPC trajectory was: " << cost << "\n";
    std::cout << "avg opt time: " << avg_opt_time << " ms \n";
    std::cout << "avg percent derivs: " << avg_percent_derivs << " % \n";
    std::cout << "avg time derivs: " << avg_time_derivs << " ms \n";
    std::cout << "avg time BP: " << avg_time_bp << " ms \n";
    std::cout << "avg time FP: " << avg_time_fp << " ms \n";

}

// Before calling this function, we should setup the activeModelTranslator with the correct initial state and the
// Optimiser settings. This function can then return necessary testing data for us to store
void MPCUntilComplete(int OPT_HORIZON){

    std::vector<double> time_get_derivs;
    std::vector<double> time_bp;
    std::vector<double> time_fp;
    std::vector<double> percent_derivs_computed;

    std::vector<MatrixXd> optimised_controls;

    // Instantiate init controls
    std::vector<MatrixXd> init_opt_controls;

    // Create init optimisation controls and reset system state
    init_opt_controls = activeModelTranslator->CreateInitOptimisationControls(OPT_HORIZON);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], activeModelTranslator->MuJoCo_helper->master_reset_data);

    optimised_controls = activeOptimiser->Optimise(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], init_opt_controls, 1, 1, OPT_HORIZON);

    MatrixXd current_state;

    while(!stop_mpc){

        // Copy current state of system (vis data) to starting data object for optimisation
        activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], activeModelTranslator->MuJoCo_helper->vis_data);

        // Get the current control index
        int current_control_index = activeVisualiser->current_control_index;

        // Delete all controls before control index
        optimised_controls.erase(optimised_controls.begin(), optimised_controls.begin() + current_control_index);

        // Copy last control to keep control trajectory the same size
        MatrixXd last_control = optimised_controls.back();
        for (int i = 0; i < current_control_index; ++i) {
            optimised_controls.push_back(last_control);
        }

        optimised_controls = activeOptimiser->Optimise(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], optimised_controls, 1, 1, OPT_HORIZON);

        // Store last iteration timing results
        time_get_derivs.push_back(activeOptimiser->avg_time_get_derivs_ms);
        time_bp.push_back(activeOptimiser->avg_time_backwards_pass_ms);
        time_fp.push_back(activeOptimiser->avg_time_forwards_pass_ms);
        percent_derivs_computed.push_back(activeOptimiser->avg_percent_derivs);

        int optTimeToTimeSteps = activeOptimiser->opt_time_ms / (activeModelTranslator->MuJoCo_helper->ReturnModelTimeStep() * 1000);

        // By the time we have computed optimal controls, main visualisation will be some number
        // of time-steps ahead. We need to find the correct control to apply.
        current_state = activeModelTranslator->ReturnStateVector(activeModelTranslator->MuJoCo_helper->vis_data,
                                                                 activeModelTranslator->current_state_vector);

        // Compute the best starting state
        double smallestError = 1000.00;
        int bestMatchingStateIndex = optTimeToTimeSteps;

        if(bestMatchingStateIndex >= OPT_HORIZON){
            bestMatchingStateIndex = OPT_HORIZON - 1;
        }
        for(int i = 0; i < OPT_HORIZON; i++){
//                std::cout << "i: " << i << " state: " << activeOptimiser->X_old[i].transpose() << std::endl;
//                std::cout << "correct state: " << current_vis_state.transpose() << std::endl;
            double currError = 0.0f;
            for(int j = 0; j < activeModelTranslator->current_state_vector.dof*2; j++){
                // TODO - im not sure about this, should we use full state?
                currError += abs(activeOptimiser->X_old[i](j) - current_state(j));
            }
            if(currError < smallestError){
                smallestError = currError;
                bestMatchingStateIndex = i;
            }
        }

        // Mutex lock
        {
            std::unique_lock<std::mutex> lock(mtx);

            activeVisualiser->controlBuffer = optimised_controls;

            // Set the current control index to the best matching state index
            activeVisualiser->current_control_index = bestMatchingStateIndex;

            apply_next_control = true;
        }

        std::cout << "best matching state index: " << bestMatchingStateIndex << std::endl;
    }

    avg_time_derivs = 0.0;
    avg_time_bp = 0.0;
    avg_time_fp = 0.0;
    avg_percent_derivs = 0.0;

    for(int i = 0; i < time_get_derivs.size(); i++){
        avg_time_derivs += time_get_derivs[i];
        avg_time_bp += time_bp[i];
        avg_time_fp += time_fp[i];
        avg_percent_derivs += percent_derivs_computed[i];
    }

    avg_time_derivs /= time_get_derivs.size();
    avg_time_bp /= time_bp.size();
    avg_time_fp /= time_fp.size();
    avg_percent_derivs /= percent_derivs_computed.size();

    avg_opt_time = avg_time_derivs + avg_time_bp + avg_time_fp;
}

int assign_task(){
    if(task == "acrobot"){
        std::shared_ptr<Acrobot> myAcrobot = std::make_shared<Acrobot>();
        activeModelTranslator = myAcrobot;

    }
    else if(task == "pentabot"){
        std::shared_ptr<Pentabot> my_pentabot = std::make_shared<Pentabot>();
        activeModelTranslator = my_pentabot;
    }
    else if(task == "reaching"){
        std::shared_ptr<pandaReaching> myReaching = std::make_shared<pandaReaching>();
        activeModelTranslator = myReaching;
    }
    else if(task == "pushing_no_clutter"){
        std::shared_ptr<TwoDPushing> myTwoDPushing = std::make_shared<TwoDPushing>(noClutter);
        activeModelTranslator = myTwoDPushing;

    }
    else if(task == "pushing_low_clutter"){
        std::shared_ptr<TwoDPushing> myTwoDPushing = std::make_shared<TwoDPushing>(lowClutter);
        activeModelTranslator = myTwoDPushing;

    }
    else if(task == "pushing_moderate_clutter"){
        std::shared_ptr<TwoDPushing> myTwoDPushing = std::make_shared<TwoDPushing>(heavyClutter);
        activeModelTranslator = myTwoDPushing;

    }
    else if(task == "pushing_moderate_clutter_constrained"){
        std::shared_ptr<TwoDPushing> myTwoDPushing = std::make_shared<TwoDPushing>(constrainedClutter);
        activeModelTranslator = myTwoDPushing;
    }
    else if(task == "3D_pushing"){
        std::shared_ptr<ThreeDPushing> myThreeDPushing = std::make_shared<ThreeDPushing>();
        activeModelTranslator = myThreeDPushing;
    }
    else if(task == "box_push_toppling"){
        cout << "not implemented task yet " << endl;
        return EXIT_FAILURE;
    }
    else if(task == "box_flick_no_clutter"){
        std::shared_ptr<BoxFlick> myBoxFlick = std::make_shared<BoxFlick>(noClutter);
        activeModelTranslator = myBoxFlick;
    }
    else if(task == "box_flick_low_clutter"){
        std::shared_ptr<BoxFlick> myBoxFlick = std::make_shared<BoxFlick>(lowClutter);
        activeModelTranslator = myBoxFlick;
    }
    else if(task == "box_flick_moderate_clutter"){
        std::shared_ptr<BoxFlick> myBoxFlick = std::make_shared<BoxFlick>(heavyClutter);
        activeModelTranslator = myBoxFlick;
    }
    else if(task == "walker_walk"){
        std::shared_ptr<walker> myLocomotion = std::make_shared<walker>(PLANE, WALK);
        activeModelTranslator = myLocomotion;
    }
    else if(task == "walker_run"){
        std::shared_ptr<walker> myLocomotion = std::make_shared<walker>(UNEVEN, RUN);
        activeModelTranslator = myLocomotion;
    }
    else if(task == "walker_uneven"){
        std::shared_ptr<walker> myLocomotion = std::make_shared<walker>(UNEVEN, WALK);
        activeModelTranslator = myLocomotion;
    }
    else if(task == "Hopper"){
        cout << "not implemented task yet " << endl;
        return EXIT_FAILURE;
    }
    else if(task == "box_sweep"){
        std::shared_ptr<BoxSweep> myBoxSweep = std::make_shared<BoxSweep>();
        activeModelTranslator = myBoxSweep;
    }
    else if(task == "piston_block"){
        std::shared_ptr<PistonBlock> my_piston_block = std::make_shared<PistonBlock>();
        activeModelTranslator = my_piston_block;
    }
    else{
        std::cout << "invalid scene selected, exiting" << std::endl;
    }
    return EXIT_SUCCESS;
}


//int generateTestingData_MPCHorizons(){
//    playback = false;
//    mpcVisualise = false;
//
//    // start timer here
//    auto startTime = std::chrono::high_resolution_clock::now();
//
//    activeDifferentiator = std::make_shared<Differentiator>(activeModelTranslator, activeModelTranslator->MuJoCo_helper);
//
//    activeVisualiser = std::make_shared<Visualiser>(activeModelTranslator);
//    iLQROptimiser = std::make_shared<iLQR>(activeModelTranslator, activeModelTranslator->MuJoCo_helper,
//                                           activeDifferentiator, yamlReader->maxHorizon, activeVisualiser,
//                                           yamlReader);
//    activeOptimiser = iLQROptimiser;
//
//    // ------------------------- data storage -------------------------------------
//    std::vector<std::vector<double>> finalCosts;
//    std::vector<double> finalCostsRow;
//
//    std::vector<std::vector<double>> avgHzs;
//    std::vector<double> avgHZRow;
//
//    std::vector<std::vector<double>> avgTimeForDerivs;
//    std::vector<double> avgTimeForDerivsRow;
//
//    std::vector<std::vector<double>> avgTimeBP;
//    std::vector<double> avgTimeBPRow;
//
//    std::vector<std::vector<double>> avgTimeFP;
//    std::vector<double> avgTimeFPRow;
//
//    std::vector<std::vector<double>> avgPercentDerivs;
//    std::vector<double> avgPercentDerivsRow;
//
//    std::vector<int> horizons = {20, 30, 40, 50, 60, 70, 80};
//    std::vector<std::string> horizonNames;
//    int numHorizons = horizons.size();
//
//    for(int i = 0; i < horizons.size(); i++){
//        horizonNames.push_back(std::to_string(horizons[i]));
//    }
//
//    if(testingMethods.empty()){
//        return 0;
//    }
//
//    std::vector<std::string> methodNames = {"baseline", "SI5", "SI10", "SI20", "adaptive_jerk2", "iterative_error", "magvel_change2"};
//    std::vector<int> testIndices;
//    bool anyMatch = false;
//    for(const auto & testingMethod : testingMethods){
//        for(int j = 0; j < methodNames.size(); j++){
//            if(testingMethod == methodNames[j]){
//                anyMatch = true;
//                testIndices.push_back(j);
//            }
//        }
//
//    }
//
//    if(!anyMatch){
//        cout << "passed testing arguments didnt match any allowed methods \n";
//        return 0;
//    }
//
//    std::vector<int> minN = {1, 5, 10, 20, 2, 2, 2};
//    std::vector<int> maxN = {1, 5, 10, 20, 20, 5, 10};
//    std::vector<std::string> keypoint_method = {"setInterval", "setInterval", "setInterval", "setInterval", "adaptive_jerk", "iterative_error", "magvel_change"};
//
//    std::vector<double> targetVelocities;
//    double minTarget = 0.1;
//    double maxTarget = 0.3;
//    int numTests = 100;
//    double currentVel = minTarget;
//
//    for(int i = 0; i < numTests; i++){
//        targetVelocities.push_back(currentVel);
//        currentVel += (maxTarget - minTarget) / (numTests - 1);
//    }
//
//    auto startTimer = std::chrono::high_resolution_clock::now();
//    activeOptimiser->verbose_output = false;
//
//    for(int testIndex : testIndices) {
//        cout << "---------- current method " << methodNames[testIndex] << " ----------------" << endl;
//
//        struct keypoint_method currentInterpolator = activeOptimiser->ReturnCurrentKeypointMethod();
//        currentInterpolator.min_N = minN[testIndex];
//        currentInterpolator.max_N = maxN[testIndex];
//        currentInterpolator.name = keypoint_method[testIndex];
//        activeOptimiser->SetCurrentKeypointMethod(currentInterpolator);
//
//        finalCosts.clear();
//        avgHzs.clear();
//        avgTimeForDerivs.clear();
//        avgTimeBP.clear();
//        avgTimeFP.clear();
//        avgPercentDerivs.clear();
//
//        for (int i = 0; i < targetVelocities.size(); i++) {
//            cout << "------------------------------------ Trajec " << i << " ------------------------------------\n";
//
//            // Loop through our interpolating derivatives methods
//            finalCostsRow.clear();
//            avgHZRow.clear();
//            avgTimeForDerivsRow.clear();
//            avgTimeBPRow.clear();
//            avgTimeFPRow.clear();
//            avgPercentDerivsRow.clear();
//
//            MatrixXd startStateVector;
//            startStateVector.resize(activeModelTranslator->state_vector_size, 1);
//
//            yamlReader->loadTaskFromFile(activeModelTranslator->model_name, i, startStateVector,
//                                         activeModelTranslator->X_desired);
//
//            // Walker model where were trying to match a velocity
//            if(task == "walker"){
//                activeModelTranslator->X_desired(10) = targetVelocities[i];
//            }
//
//            activeModelTranslator->X_start = startStateVector;
//            activeModelTranslator->SetStateVector(startStateVector, activeModelTranslator->MuJoCo_helper->master_reset_data);
//            for(int j = 0; j < 5; j++){
//                mj_step(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->master_reset_data);
//            }
//
//            if(!activeModelTranslator->MuJoCo_helper->checkIfDataIndexExists(0)){
//                activeModelTranslator->MuJoCo_helper->appendSystemStateToEnd(activeModelTranslator->MuJoCo_helper->master_reset_data);
//            }
//
//            // Move the end-effector to a decent starting position
//            activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
//            std::vector<MatrixXd> setupControls = activeModelTranslator->CreateInitSetupControls(1000);
//            activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->master_reset_data, activeModelTranslator->MuJoCo_helper->main_data);
//
//            for (int j = 0; j < numHorizons; j++) {
//                double avgHz = 0.0f;
//                double finalCost = 0.0f;
//                double avgPercentageDerivs = 0.0f;
//                double avgTimeForDerivs = 0.0f;
//                double avgTimeBP = 0.0f;
//                double avgTimeFP = 0.0f;
//
//                cout << "--------------------------------------------------------------------------------\n";
//                cout << "current horizon: " << horizonNames[j] << "\n";
//
//                activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
//                MPCUntilComplete(finalCost, avgHz, avgTimeForDerivs, avgPercentageDerivs, avgTimeBP, avgTimeFP, 1500, 1, horizons[j]);
//
//                finalCostsRow.push_back(finalCost);
//                avgHZRow.push_back(avgHz);
//                avgTimeForDerivsRow.push_back(avgTimeForDerivs);
//                avgTimeBPRow.push_back(avgTimeBP);
//                avgTimeFPRow.push_back(avgTimeFP);
//                avgPercentDerivsRow.push_back(avgPercentageDerivs);
//            }
//
//            // New row of data added
//            finalCosts.push_back(finalCostsRow);
//            avgHzs.push_back(avgHZRow);
//            avgTimeForDerivs.push_back(avgTimeForDerivsRow);
//            avgTimeBP.push_back(avgTimeBPRow);
//            avgTimeFP.push_back(avgTimeFPRow);
//            avgPercentDerivs.push_back(avgPercentDerivsRow);
//
//            auto currentTime = std::chrono::high_resolution_clock::now();
//            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTimer).count();
//
//            cout << "Time taken so far: " << duration/ 1000.0f << " s" << endl;
//        }
//        // Save data to csv
//        cout << "save data to file for " << methodNames[testIndex] << endl;
//        std::string taskPrefix = activeModelTranslator->model_name + "_" + methodNames[testIndex];
//        yamlReader->saveResultsData_MPC(taskPrefix, horizonNames, finalCosts, avgHzs,
//                                        avgTimeForDerivs, avgTimeBP, avgTimeFP, avgPercentDerivs);
//    }
//
//    cout << "tests exited correctly \n";
//    return 1;
//}

//void generateTestingData_MPC(){
//    playback = false;
//    mpcVisualise = false;
//
//    // start timer here
//    auto startTime = std::chrono::high_resolution_clock::now();
//
//    for(int k = 0; k < 1; k ++) {
//        activeDifferentiator = std::make_shared<Differentiator>(activeModelTranslator, activeModelTranslator->MuJoCo_helper);
//
//        activeVisualiser = std::make_shared<Visualiser>(activeModelTranslator);
//        iLQROptimiser = std::make_shared<iLQR>(activeModelTranslator, activeModelTranslator->MuJoCo_helper,
//                                               activeDifferentiator, yamlReader->maxHorizon, activeVisualiser,
//                                               yamlReader);
//        activeOptimiser = iLQROptimiser;
//
//        // ------------------------- data storage -------------------------------------
//        std::vector<std::vector<double>> finalCosts;
//        std::vector<double> finalCostsRow;
//
//        std::vector<std::vector<double>> avgHzs;
//        std::vector<double> avgHZRow;
//
//        std::vector<std::vector<double>> avgTimeForDerivs;
//        std::vector<double> avgTimeForDerivsRow;
//
//        std::vector<std::vector<double>> avgTimeBP;
//        std::vector<double> avgTimeBPRow;
//
//        std::vector<std::vector<double>> avgTimeFP;
//        std::vector<double> avgTimeFPRow;
//
//        std::vector<std::vector<double>> avgPercentDerivs;
//        std::vector<double> avgPercentDerivsRow;
//
////        std::vector<std::string> methodNames = {"baseline", "SI5", "SI20", "adapJerk", "iter_error", "magvel"};
////        int numMethods = methodNames.size();
////        std::vector<string> keypointMethods = {"setInterval", "setInterval", "setInterval", "adaptive_jerk", "iterative_error", "magvel_change"};
////        std::vector<int> min_N = {1, 5, 20, 1, 1, 1};
////        std::vector<int> max_N = {1, 2, 5, 5, 5, 5};
//
//        std::vector<std::string> methodNames = {"magvel", "iter_error"};
//        int numMethods = methodNames.size();
//        std::vector<string> keypointMethods = {"magvel_change", "iterative_error"};
//        std::vector<int> minN = {1, 1};
//        std::vector<int> maxN = {5, 5};
//
//        std::vector<double> targetVelocities;
//        double minTarget = 0.1;
//        double maxTarget = 0.3;
//        int numTests = 100;
//        double currentVel = minTarget;
//
//        for(int i = 0; i < numTests; i++){
//            targetVelocities.push_back(currentVel);
//            currentVel += (maxTarget - minTarget) / (numTests - 1);
//        }
//
//        auto startTimer = std::chrono::high_resolution_clock::now();
//        activeOptimiser->verbose_output = false;
//
//        for (int i = 0; i < targetVelocities.size(); i++) {
//            cout << "------------------------------------ Trajec " << i << " ------------------------------------\n";
//
//            // Loop through our interpolating derivatives methods
//            finalCostsRow.clear();
//            avgHZRow.clear();
//            avgTimeForDerivsRow.clear();
//            avgTimeBPRow.clear();
//            avgTimeFPRow.clear();
//            avgPercentDerivsRow.clear();
//
//            MatrixXd startStateVector;
//            startStateVector.resize(activeModelTranslator->state_vector_size, 1);
//
//            yamlReader->loadTaskFromFile(activeModelTranslator->model_name, i, startStateVector,
//                                         activeModelTranslator->X_desired);
//
//            // Walker model where were trying to match a velocity
//            if(task == "walker"){
//                activeModelTranslator->X_desired(10) = targetVelocities[i];
//            }
//
//
//            activeModelTranslator->X_start = startStateVector;
//            activeModelTranslator->SetStateVector(startStateVector, activeModelTranslator->MuJoCo_helper->master_reset_data);
//            for(int j = 0; j < 5; j++){
//                mj_step(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->master_reset_data);
//            }
//
//            if(!activeModelTranslator->MuJoCo_helper->checkIfDataIndexExists(0)){
//                activeModelTranslator->MuJoCo_helper->appendSystemStateToEnd(activeModelTranslator->MuJoCo_helper->master_reset_data);
//            }
//
//            // Move the end-effector to a decent starting position
//            activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
//            std::vector<MatrixXd> setupControls = activeModelTranslator->CreateInitSetupControls(1000);
//            activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->master_reset_data, activeModelTranslator->MuJoCo_helper->main_data);
//
//
////            activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
////            activeModelTranslator->MuJoCo_helper->copySystemState(0, activeModelTranslator->MuJoCo_helper->master_reset_data);
//
//            for (int j = 0; j < numMethods; j++) {
//                double avgHz = 0.0f;
//                double finalCost = 0.0f;
//                double avgPercentageDerivs = 0.0f;
//                double avgTimeForDerivs = 0.0f;
//                double avgTimeBP = 0.0f;
//                double avgTimeFP = 0.0f;
//
//                cout << "--------------------------------------------------------------------------------\n";
//                cout << "current method: " << methodNames[j] << "\n";
//
//                // Setup the keypoint method
//                keypoint_method currentInterpolator = activeOptimiser->ReturnCurrentKeypointMethod();
//                currentInterpolator.min_N = minN[j];
//                currentInterpolator.max_N = maxN[j];
//                currentInterpolator.name = keypointMethods[j];
//                activeOptimiser->SetCurrentKeypointMethod(currentInterpolator);
//
//                activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
//                MPCUntilComplete(finalCost, avgHz, avgTimeForDerivs, avgPercentageDerivs, avgTimeBP, avgTimeFP, 1000, 1, 50);
//
//                finalCostsRow.push_back(finalCost);
//                avgHZRow.push_back(avgHz);
//                avgTimeForDerivsRow.push_back(avgTimeForDerivs);
//                avgTimeBPRow.push_back(avgTimeBP);
//                avgTimeFPRow.push_back(avgTimeFP);
//                avgPercentDerivsRow.push_back(avgPercentageDerivs);
//            }
//
//            // New row of data added
//            finalCosts.push_back(finalCostsRow);
//            avgHzs.push_back(avgHZRow);
//            avgTimeForDerivs.push_back(avgTimeForDerivsRow);
//            avgTimeBP.push_back(avgTimeBPRow);
//            avgTimeFP.push_back(avgTimeFPRow);
//            avgPercentDerivs.push_back(avgPercentDerivsRow);
//
//            auto currentTime = std::chrono::high_resolution_clock::now();
//            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTimer).count();
//
//
//            cout << "Time taken so far: " << duration/ 1000.0f << " s" << endl;
//        }
//        // Save data to csv
//        cout << "save data to file\n";
//        yamlReader->saveResultsData_MPC(activeModelTranslator->model_name, methodNames, finalCosts, avgHzs,
//                                        avgTimeForDerivs, avgTimeBP, avgTimeFP, avgPercentDerivs);
//    }
//}

//void generateFilteringData(){
//    int setupHorizon = 1000;
//    int optHorizon = 2200;
//    int numTests = 100;
//
//    MatrixXd startStateVector;
//    startStateVector.resize(activeModelTranslator->state_vector_size, 1);
//
//    std::vector<double> lowPassTests = {0.05, 0.1, 0.15, 0.2, 0.25, 0.3};
//    std::vector<std::vector<double>> FIRTests;
//    FIRTests.push_back({0.25, 0.5, 0.25});
//    FIRTests.push_back({0.1, 0.15, 0.5, 0.15, 0.1});
//    FIRTests.push_back({0.05, 0.1, 0.15, 0.3, 0.15, 0.1, 0.05});
//    FIRTests.push_back({0.05, 0.05, 0.15, 0.2, 0.2, 0.2, 0.15, 0.05, 0.05});
//    FIRTests.push_back({0.05, 0.1, 0.15, 0.2, 0.2, 0.15, 0.1, 0.05});
//
//    keypoint_method currentInterpolator = activeOptimiser->ReturnCurrentKeypointMethod();
//    currentInterpolator.name = "setInterval";
//    currentInterpolator.min_N = 1;
//    activeOptimiser->SetCurrentKeypointMethod(currentInterpolator);
//
//    for (int i = 0; i < numTests; i++) {
//        yamlReader->loadTaskFromFile(activeModelTranslator->model_name, i, startStateVector,
//                                     activeModelTranslator->X_desired);
//        activeModelTranslator->X_start = startStateVector;
//        activeModelTranslator->SetStateVector(startStateVector, activeModelTranslator->MuJoCo_helper->master_reset_data);
//        mj_step(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->master_reset_data);
//
//        std::vector<MatrixXd> initSetupControls = activeModelTranslator->CreateInitSetupControls(setupHorizon);
//        activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->master_reset_data, activeModelTranslator->MuJoCo_helper->main_data);
//
//        std::vector<MatrixXd> initOptimisationControls = activeModelTranslator->CreateInitOptimisationControls(optHorizon);
//
//        // Initialise task for optimisation by here
//
//        // ---------------- unfiltered tests ------------------------------
//        activeOptimiser->filteringMethod = "none";
//        // Load a task from saved tasks
//
//        activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
//        activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->savedSystemStatesList[0], activeModelTranslator->MuJoCo_helper->master_reset_data);
//
//        std::vector<MatrixXd> optimisedControls = activeOptimiser->Optimise(activeModelTranslator->MuJoCo_helper->savedSystemStatesList[0],
//                                                                            initOptimisationControls,
//                                                                            yamlReader->maxIter,
//                                                                            yamlReader->minIter,
//                                                                            optHorizon);
//        // Save cost history to file
//        std::string filePrefix;
//        filePrefix = activeModelTranslator->model_name + "/none/";
//
//        yamlReader->saveCostHistory(activeOptimiser->costHistory, filePrefix, i);
//
//        // ---------------------- Low pass filter tests ----------------------
//
//        for(double lowPassTest : lowPassTests){
//            activeOptimiser->filteringMethod = "low_pass";
//            activeOptimiser->lowPassACoefficient = lowPassTest;
//            // Load a task from saved tasks
//
//            activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
//            activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->savedSystemStatesList[0], activeModelTranslator->MuJoCo_helper->master_reset_data);
//
//            std::vector<MatrixXd> optimisedControls = activeOptimiser->Optimise(activeModelTranslator->MuJoCo_helper->savedSystemStatesList[0],
//                                                                                initOptimisationControls,
//                                                                                yamlReader->maxIter,
//                                                                                yamlReader->minIter,
//                                                                                optHorizon);
//            // Save cost history to file
//            std::string filePrefix;
//
//            filePrefix = activeModelTranslator->model_name + "/lowPass" + std::to_string(lowPassTest) + "/";
//
//            yamlReader->saveCostHistory(activeOptimiser->costHistory, filePrefix, i);
//        }
//
//        // ---------------------- FIR filter tests ----------------------
//        for(int j = 0; j < FIRTests.size(); j++){
//            activeOptimiser->filteringMethod = "FIR";
//            activeOptimiser->setFIRFilter(FIRTests[j]);
//            // Load a task from saved tasks
//
//            activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
//            activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->savedSystemStatesList[0], activeModelTranslator->MuJoCo_helper->master_reset_data);
//
//            std::vector<MatrixXd> optimisedControls = activeOptimiser->Optimise(activeModelTranslator->MuJoCo_helper->savedSystemStatesList[0],
//                                                                                initOptimisationControls,
//                                                                                yamlReader->maxIter,
//                                                                                yamlReader->minIter,
//                                                                                optHorizon);
//            // Save cost history to file
//            std::string filePrefix;
//
//            filePrefix = activeModelTranslator->model_name + "/FIR_" + std::to_string(j) + "/";
//
//            yamlReader->saveCostHistory(activeOptimiser->costHistory, filePrefix, i);
//        }
//
//
//    }
//}

//void onetaskGenerateTestingData(){
//    int setupHorizon = 1000;
//    int optHorizon = 2800;
//
//    MatrixXd startStateVector;
//    startStateVector.resize(activeModelTranslator->state_vector_size, 1);
//
//    std::vector<std::vector<double>> optTimes;
//    std::vector<double> optTimesRow;
//
//    std::vector<std::vector<double>> costReductions;
//    std::vector<double> costReductionsRow;
//
//    std::vector<std::vector<double>> avgPercentageDerivs;
//    std::vector<double> avgPercentageDerivsRow;
//
//    std::vector<std::vector<double>> avgTimeForDerivs;
//    std::vector<double> avgTimeForDerivsRow;
//
//    std::vector<std::vector<int>> numIterations;
//    std::vector<int> numIterationsRow;
//
//    // Object manipulation?
////    std::vector<std::string> methodNames = {"baseline", "SI5", "SI1000", "adaptive_jerk_5", "iterative_error_5", "magvel_change_5"};
////    int numMethods = methodNames.size();
////    std::vector<string> keyPointMethods = {"setInterval", "setInterval", "setInterval", "adaptive_jerk", "iterative_error", "magvel_change"};
////    std::vector<int> min_N = {1, 5, 1000, 5, 5, 5};
////    std::vector<int> max_N = {1, 5, 1000, 100, 100, 100};
//
//    std::vector<std::string> methodNames = {"baseline", "SI5", "SI1000", "adaptive_jerk_5", "iterative_error_5", "magvel_change_5"};
//    int numMethods = methodNames.size();
//    std::vector<string> keyPointMethods = {"set_interval", "set_interval", "set_interval", "adaptive_jerk", "iterative_error", "magvel_change"};
//    std::vector<int> minN = {1, 5, 1000, 2, 2, 2};
//    std::vector<int> maxN = {1, 5, 1000, 10, 10, 10};
//
////    std::vector<std::string> methodNames = {"iterative_error"};
////    int numMethods = methodNames.size();
////    std::vector<std::string> keyPointMethods = {"iterative_error"};
////    std::vector<int> min_N = {5};
////    std::vector<int> max_N = {20};
//
//    // Loop through saved trajectories
//    for(int i = 0; i < 100; i++){
//        cout << "------------------------------------ Trajec " << i << " ------------------------------------\n";
//        cout << "dof: " << activeModelTranslator->dof << endl;
//
//        // Loop through our interpolating derivatives methods
//        optTimesRow.clear();
//        costReductionsRow.clear();
//        avgPercentageDerivsRow.clear();
//        avgTimeForDerivsRow.clear();
//        numIterationsRow.clear();
//
//        yamlReader->loadTaskFromFile(activeModelTranslator->model_name, i, startStateVector, activeModelTranslator->X_desired);
//        activeModelTranslator->X_start = startStateVector;
//        activeModelTranslator->SetStateVector(startStateVector, activeModelTranslator->MuJoCo_helper->master_reset_data);
//        for(int j = 0; j < 5; j++){
//            mj_step(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->master_reset_data);
//        }
//
//
//        if(!activeModelTranslator->MuJoCo_helper->checkIfDataIndexExists(0)){
//            activeModelTranslator->MuJoCo_helper->appendSystemStateToEnd(activeModelTranslator->MuJoCo_helper->master_reset_data);
//        }
//
//        // Move the end-effector to a decent starting position
//        activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
//        std::vector<MatrixXd> setupControls = activeModelTranslator->CreateInitSetupControls(1000);
//        activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->master_reset_data, activeModelTranslator->MuJoCo_helper->main_data);
//
//        std::vector<MatrixXd> initOptimisationControls = activeModelTranslator->CreateInitOptimisationControls(optHorizon);
//
//        for(int j = 0; j < numMethods; j++){
//            double optTime;
//            double costReduction;
//            double avgPercentageDerivs;
//            double avgTimeForDerivs;
//            int numIterationsForConvergence;
//
//            activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
//            activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->savedSystemStatesList[0], activeModelTranslator->MuJoCo_helper->master_reset_data);
//
//            // Setup interpolation method
//            keypoint_method currentInterpolator = activeOptimiser->ReturnCurrentKeypointMethod();
//            currentInterpolator.name = keyPointMethods[j];
//            currentInterpolator.min_N = minN[j];
//            currentInterpolator.max_N = maxN[j];
//            activeOptimiser->SetCurrentKeypointMethod(currentInterpolator);
//
//            std::vector<MatrixXd> optimisedControls = activeOptimiser->Optimise(activeModelTranslator->MuJoCo_helper->savedSystemStatesList[0], initOptimisationControls, 8, 2, optHorizon);
//
////            yamlReader->save_trajec_information(activeOptimiser->A, activeOptimiser->B, activeOptimiser->X_old, activeOptimiser->U_old, activeModelTranslator->model_name, i, optHorizon);
//
//            // Return testing data and append appropriately
//            activeOptimiser->ReturnOptimisationData(optTime, costReduction, avgPercentageDerivs, avgTimeForDerivs, numIterationsForConvergence);
//            optTimesRow.push_back(optTime);
//            costReductionsRow.push_back(costReduction);
//            avgPercentageDerivsRow.push_back(avgPercentageDerivs);
//            avgTimeForDerivsRow.push_back(avgTimeForDerivs);
//            numIterationsRow.push_back(numIterationsForConvergence);
//
////            activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->main_data, 0);
//
////            int controlCounter = 0;
////            int visualCounter = 0;
////            cout << "final controls size: " << optimisedControls.size() << endl;
////
////            while(controlCounter < initOptimisationControls.size()){
////
////                activeModelTranslator->SetControlVector(initOptimisationControls[controlCounter], activeModelTranslator->MuJoCo_helper->main_data);
////
////                activeModelTranslator->MuJoCo_helper->stepSimulator(1, activeModelTranslator->MuJoCo_helper->main_data);
////
////                controlCounter++;
////                visualCounter++;
////
////                if(visualCounter > 5){
////
////                    activeVisualiser->render("show init controls");
////                    visualCounter = 0;
////                }
////            }
//
//        }
//        optTimes.push_back(optTimesRow);
//        costReductions.push_back(costReductionsRow);
//        avgPercentageDerivs.push_back(avgPercentageDerivsRow);
//        avgTimeForDerivs.push_back(avgTimeForDerivsRow);
//        numIterations.push_back(numIterationsRow);
//
//        cout << "average percent derivs: " << avgPercentageDerivsRow[0] << endl;
//        cout << "row " << i << " done \n";
//    }
//
//    // Save data to file
//    cout << "save data to file \n";
//    yamlReader->saveResultsDataForMethods(activeModelTranslator->model_name, methodNames, optTimes, costReductions, avgPercentageDerivs, avgTimeForDerivs, numIterations);
//}

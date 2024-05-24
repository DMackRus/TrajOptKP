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
#include "ModelTranslator/SweepMultiple.h"

#include "ModelTranslator/Walker.h"

#include "ModelTranslator/SquishSoft.h"
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

bool record_trajectory = false;
bool async_mpc = true;
std::string task;
bool mpc_visualise = true;
bool playback = true;

std::mutex mtx;
std::condition_variable cv;

std::vector<stateVectorList> tracking_state_vector;
stateVectorList current_mpc_state_vector;

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
    record_trajectory = yamlReader->record_trajectory;

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


        activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
//
//        std::cout << "nq: " << activeModelTranslator->MuJoCo_helper->model->nq << "\n";
//        std::cout << "nv: " << activeModelTranslator->MuJoCo_helper->model->nv << "\n";
//        std::cout << "nbody: " << activeModelTranslator->MuJoCo_helper->model->nbody << "\n";
//
//        // Print state vector name and its associated q pos index
//        std::cout << "dof in current state vector: " << activeModelTranslator->current_state_vector.dof << "\n";
//        std::cout << "state vector names size: " << activeModelTranslator->current_state_vector.state_names.size() << "\n";
//        for(int i = 0; i < activeModelTranslator->current_state_vector.dof; i++){
//            std::cout << "i: " << i << "\n";
//            int q_index = activeModelTranslator->StateIndexToQposIndex(i, activeModelTranslator->current_state_vector);
//            std::cout << "state name: " << activeModelTranslator->current_state_vector.state_names[i] << ": " << q_index << "\n";
//        }

        //
//        activeModelTranslator->current_state_vector.PrintFormattedStateVector();
//
//        // remove some vertices
//        std::vector<std::string> remove = {"jelly_V1_x", "jelly_V2_y", "jelly_V11_z"};
//        activeModelTranslator->UpdateCurrentStateVector(remove, false);
//
//        activeModelTranslator->current_state_vector.PrintFormattedStateVector();


//        std::string flex_name = "Jelly2";
//        int flexId = mj_name2id(activeModelTranslator->MuJoCo_helper->model, mjOBJ_FLEX, flex_name.c_str());
//        int firstVertadr = activeModelTranslator->MuJoCo_helper->model->flex_vertadr[flexId];
//        int numVertices = activeModelTranslator->MuJoCo_helper->model->flex_vertnum[flexId];
//
//        int body_id = activeModelTranslator->MuJoCo_helper->model->flex_vertbodyid[firstVertadr];
//        int geom_id = activeModelTranslator->MuJoCo_helper->model->body_geomadr[body_id];
//
//        std::cout << "flex id: " << flexId << " first vert adr: " << firstVertadr << "body is: " << body_id << "geom id: " << geom_id << "\n";
//
//        activeModelTranslator->MuJoCo_helper->model->geom_rgba[geom_id * 4]     = 0; // Red
//        activeModelTranslator->MuJoCo_helper->model->geom_rgba[geom_id * 4 + 1] = 0; // Green
//        activeModelTranslator->MuJoCo_helper->model->geom_rgba[geom_id * 4 + 2] = 0; // Blue
//        activeModelTranslator->MuJoCo_helper->model->geom_rgba[geom_id * 4 + 3] = 0.5; // Alpha
//
        while(activeVisualiser->windowOpen()){
//            activeModelTranslator->MuJoCo_helper->ForwardSimulator(activeModelTranslator->MuJoCo_helper->vis_data);
            mj_step(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->vis_data);
            activeVisualiser->render("Test");
        }
        // --------------------------------------------------------------------------------------------------------------

//        std::vector<double> time_derivs_full;
//        std::vector<double> time_bp_full;
//        std::vector<double> time_derivs_reduced;
//        std::vector<double> time_bp_reduced;

//        int dofs_full, dofs_reduced;
//        int N = 10;
//
//        dofs_full = activeModelTranslator->full_state_vector.dof;
//
//        iLQROptimiser = std::make_shared<iLQR>(activeModelTranslator, activeModelTranslator->MuJoCo_helper, activeDifferentiator, opt_horizon, activeVisualiser, yamlReader);

//        MatrixXd state_vector = activeModelTranslator->ReturnStateVectorQuaternions(activeModelTranslator->MuJoCo_helper->master_reset_data);
//        std::cout << "size of state vector quaternion: " << state_vector.rows() << std::endl;
//        activeModelTranslator->current_state_vector.Update();
//        std::cout << "num dofs: " << activeModelTranslator->current_state_vector.dof << " num dofs quat: " << activeModelTranslator->current_state_vector.dof_quat << std::endl;

//        std::cout << "state vector: ";
//        for(const auto & state_name : activeModelTranslator->current_state_vector.state_names){
//            std::cout << state_name << " ";
//        }
//        std::cout << "\n";
//        std::cout << "size of state vector before: " << activeModelTranslator->current_state_vector.dof << " \n";
//
//        // remove elements
//        std::vector<std::string> remove = {"panda0_joint1", "goal_x", "obstacle_1_x"};
//        activeModelTranslator->UpdateCurrentStateVector(remove, false);
//
//        std::cout << "state vector: ";
//        for(const auto & state_name : activeModelTranslator->current_state_vector.state_names){
//            std::cout << state_name << " ";
//        }
//        std::cout << "\n";
//        std::cout << "size of state vector after: " << activeModelTranslator->current_state_vector.dof << " \n";


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


    if(record_trajectory){
        activeVisualiser->StartRecording(task + "_init_controls");
    }

    while(activeVisualiser->windowOpen()){

        activeModelTranslator->SetControlVector(initControls[controlCounter], activeModelTranslator->MuJoCo_helper->main_data,
                                                activeModelTranslator->current_state_vector);
        mj_step(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->main_data);


        controlCounter++;
        visualCounter++;

        if(controlCounter >= initControls.size()){
            controlCounter = 0;
            activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
            activeVisualiser->StopRecording();
        }

        if(visualCounter > 5){
            visualCounter = 0;
            activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->main_data);
            activeModelTranslator->MuJoCo_helper->ForwardSimulator(activeModelTranslator->MuJoCo_helper->vis_data);
            if(record_trajectory){
                activeVisualiser->render("");
            }
            else{
                activeVisualiser->render("show init controls");
            }

        }
    }
}

void OpenLoopOptimisation(int opt_horizon){
    int control_counter = 0;
    int visual_counter = 0;
    bool show_opt_controls = true;
    const char* label = "Final trajectory after optimisation";

    std::vector<MatrixXd> init_controls;
    std::vector<MatrixXd> optimised_controls;

    std::vector<MatrixXd> init_setup_controls = activeModelTranslator->CreateInitSetupControls(1000);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->master_reset_data, activeModelTranslator->MuJoCo_helper->main_data);

    std::vector<MatrixXd> init_opt_controls = activeModelTranslator->CreateInitOptimisationControls(opt_horizon);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], activeModelTranslator->MuJoCo_helper->master_reset_data);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->master_reset_data);

    std::vector<MatrixXd> optimisedControls = activeOptimiser->Optimise(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0],
                                                                        init_opt_controls, yamlReader->maxIter,
                                                                        yamlReader->minIter, opt_horizon);

    // Stitch together setup controls with init control + optimised controls
    init_controls.insert(init_controls.end(), init_opt_controls.begin(), init_opt_controls.end());
    optimised_controls.insert(optimised_controls.end(), optimisedControls.begin(), optimisedControls.end());

    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);

    if(record_trajectory){
        activeVisualiser->StartRecording(task + "_optimised_controls");
        label = "";
    }
    while(activeVisualiser->windowOpen()){

        if(show_opt_controls){
            activeModelTranslator->SetControlVector(optimised_controls[control_counter], activeModelTranslator->MuJoCo_helper->main_data,
                                                    activeModelTranslator->current_state_vector);
        }
        else{
            activeModelTranslator->SetControlVector(init_controls[control_counter], activeModelTranslator->MuJoCo_helper->main_data,
                                                    activeModelTranslator->current_state_vector);
        }

        mj_step(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->main_data);

        control_counter++;
        visual_counter++;

        if(control_counter >= optimised_controls.size()){
            control_counter = 0;
            activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
            show_opt_controls = !show_opt_controls;
            if(show_opt_controls){
                label = "Final trajectory after optimisation";
            }
            else{
                label = "Initial trajectory before optimisation";
            }

            activeVisualiser->StopRecording();
        }

        if(visual_counter >= 5){
            visual_counter = 0;
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

    // Setup initial state vector for visualisation
    current_mpc_state_vector = activeModelTranslator->current_state_vector;

    std::vector<MatrixXd> replay_states;
    int nq = activeModelTranslator->MuJoCo_helper->model->nq;
    int nv = activeModelTranslator->MuJoCo_helper->model->nv;
    MatrixXd full_state(nq + nv, 1);
    auto *_full_state = new double[nq+nv];

    while(task_time < MAX_TASK_TIME){
        begin = std::chrono::steady_clock::now();

        if(async_mpc || (!async_mpc && apply_next_control)){

            tracking_state_vector.push_back(current_mpc_state_vector);


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

            // We store the full state for visualisation relay purposes
            mj_getState(activeModelTranslator->MuJoCo_helper->model,
                        activeModelTranslator->MuJoCo_helper->vis_data, _full_state, mjSTATE_PHYSICS);
            for(int i = 0; i < nq+nv; i++){
                full_state(i, 0) = _full_state[i];
            }
            replay_states.push_back(full_state);
            // ----------------------------------------

            MatrixXd next_state = activeModelTranslator->ReturnStateVectorQuaternions(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->full_state_vector);
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

    if(record_trajectory){
        activeVisualiser->StartRecording(task + "_MPC");
    }
    for(int i = 0; i < activeVisualiser->trajectory_states.size(); i++){

        // Update current state vector for visualising what was being consider at this point
        activeModelTranslator->current_state_vector = tracking_state_vector[i];
        activeModelTranslator->UpdateSceneVisualisation();

        activeModelTranslator->SetControlVector(activeVisualiser->trajectory_controls[i], activeModelTranslator->MuJoCo_helper->vis_data,
                                                activeModelTranslator->full_state_vector);

        // - complicated but works well, we use full mj_state for replay as we want to visualise all dofs and how they change
        // - not just the ones in the state vector
        for(int j = 0; j < nq+nv; j++){
            _full_state[j] = replay_states[i](j, 0);
        }
        mj_setState(activeModelTranslator->MuJoCo_helper->model,
                    activeModelTranslator->MuJoCo_helper->vis_data, _full_state, mjSTATE_PHYSICS);
        activeModelTranslator->MuJoCo_helper->ForwardSimulator(activeModelTranslator->MuJoCo_helper->vis_data);
        cost += activeModelTranslator->CostFunction(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->full_state_vector, false);
        if(i % 5 == 0){
            activeVisualiser->render("");
        }
    }
    activeVisualiser->StopRecording();

    delete [] _full_state;

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
    current_mpc_state_vector = activeModelTranslator->current_state_vector;

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
        current_mpc_state_vector = activeModelTranslator->current_state_vector;

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
    else if(task == "sweep_multiple"){
        std::shared_ptr<SweepMultiple> my_sweep_multiple = std::make_shared<SweepMultiple>();
        activeModelTranslator = my_sweep_multiple;
    }
    else if(task == "piston_block"){
        std::shared_ptr<PistonBlock> my_piston_block = std::make_shared<PistonBlock>();
        activeModelTranslator = my_piston_block;
    }
    else if(task == "squish_soft"){
        std::shared_ptr<SquishSoft> my_squish_soft = std::make_shared<SquishSoft>();
        activeModelTranslator = my_squish_soft;
    }
    else{
        std::cout << "invalid scene selected, " << task << " does not exist" << std::endl;
    }
    return EXIT_SUCCESS;
}
#include "Differentiator.h"

Differentiator::Differentiator(std::shared_ptr<ModelTranslator> _modelTranslator, std::shared_ptr<MuJoCoHelper> _MuJoCo_helper){
    activeModelTranslator = _modelTranslator;
    MuJoCo_helper = _MuJoCo_helper;

    dof = activeModelTranslator->dof;
    num_ctrl = activeModelTranslator->num_ctrl;
}

void Differentiator::ComputeDerivatives(MatrixXd &A, MatrixXd &B, const std::vector<int> &cols,
                                        MatrixXd &l_x, MatrixXd &l_u, MatrixXd &l_xx, MatrixXd &l_uu,
                                        int dataIndex, int threadId, bool terminal, bool costDerivs,
                                        bool central_diff, double eps){

    // Reset some debugging timing variables
    time_mj_forwards = 0.0f;
    count_integrations = 0;

    auto diff_start = std::chrono::high_resolution_clock::now();
    int tid = threadId;

    // Initialise sub matrices of A and B matrix
    // ------------ dof x dof ----------------
    MatrixXd dqposdqpos(dof, dof);
    MatrixXd dqposdqvel(dof, dof);
    MatrixXd dqveldqpos(dof, dof);
    MatrixXd dqveldqvel(dof, dof);

    MatrixXd dqaccdqpos(dof, dof);
    MatrixXd dqaccdqvel(dof, dof);

    // ------------ dof x ctrl --------------
    MatrixXd dqposdctrl(dof, num_ctrl);
    MatrixXd dqveldctrl(dof, num_ctrl);

    MatrixXd dqaccdctrl(dof, num_ctrl);


    // ---------- 1 x dof -------------------
    MatrixXd dcostdctrl(num_ctrl, 1);
    MatrixXd dcostdpos(dof, 1);
    MatrixXd dcostdvel(dof, 1);

    double costInc = 0.0f;
    double costDec = 0.0f;

    // Copy data we wish to finite-difference into finite differencing data (for multi threading)
    MuJoCo_helper->cpMjData(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid], MuJoCo_helper->savedSystemStatesList[dataIndex]);

    if(USE_DQACC){
        auto start = std::chrono::high_resolution_clock::now();
        calc_dqaccdctrl(dqaccdqpos, cols, dataIndex, tid, dcostdctrl, costDerivs, terminal);
//        std::cout << "dqaccdctrl time: "  << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count() / 1000.0f << std::endl;

        start = std::chrono::high_resolution_clock::now();
        calc_dqaccdqvel(dqaccdqvel, cols, dataIndex, tid, dcostdvel, costDerivs, terminal);
//        std::cout << "dqaccdqvel time: "  << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count() / 1000.0f << std::endl;

        start = std::chrono::high_resolution_clock::now();
        calc_dqaccdqpos(dqaccdqpos, cols, dataIndex, tid, dcostdpos, costDerivs, terminal);
//        std::cout << "dqaccdq time: "  << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count() / 1000.0f << std::endl;
    }
    else{
        auto start = std::chrono::high_resolution_clock::now();
        FD_Controls(dqveldctrl, dqposdctrl, cols, dataIndex, tid,
                    central_diff, eps, dcostdctrl, costDerivs, terminal);
//        std::cout << "dqveldctrl time: "  << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count() / 1000.0f << std::endl;

        start = std::chrono::high_resolution_clock::now();
        FD_Velcoities(dqveldqvel, dqposdqvel, cols, dataIndex, tid,
                      central_diff, eps, dcostdpos, costDerivs, terminal);
//        std::cout << "dqveldq time: "  << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count() / 1000.0f << std::endl;

        start = std::chrono::high_resolution_clock::now();
        FD_Positions(dqveldqpos, dqposdqpos, cols, dataIndex, tid,
                     central_diff, eps, dcostdvel, costDerivs, terminal);
//        std::cout << "dqveldqvel time: "  << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count() / 1000.0f << std::endl;
    }

    // ------------ A -----------------
    // dqposdqpos       dqposdqvel
    //
    // dqveldqpos       dqveldqvel
    // --------------------------------

    for(int i = 0; i < cols.size(); i++){
        if(USE_DQACC){
            A.block(dof, cols[i], dof, 1) = dqaccdqpos.block(0, cols[i], dof, 1) * MuJoCo_helper->returnModelTimeStep();
            for(int j = 0; j < dof; j ++){
                if(j == cols[i]){
                    A(dof + j, cols[i] + dof) = 1 + (dqaccdqvel(j, cols[i]) * MuJoCo_helper->returnModelTimeStep());
                }
                else{
                    A(dof + j, cols[i] + dof) = dqaccdqvel(j, cols[i]) * MuJoCo_helper->returnModelTimeStep();
                }

            }
        }
        else{
            A.block(0, cols[i], dof, 1) = dqposdqpos.block(0, cols[i], dof, 1);
            A.block(0, cols[i] + dof, dof, 1) = dqposdqvel.block(0, cols[i], dof, 1);

            A.block(dof, cols[i], dof, 1) = dqveldqpos.block(0, cols[i], dof, 1);
            A.block(dof, cols[i] + dof, dof, 1) = dqveldqvel.block(0, cols[i], dof, 1);
        }

    }

    // ------------- B -------------------
    //          dqposdctrl
    //          dqveldctrl
    // ----------------------------------
    for(int i = 0; i < cols.size(); i++){
        if(cols[i] < num_ctrl){
            if(USE_DQACC){
                B.block(dof, cols[i], dof, 1) = dqaccdctrl.block(0, cols[i], dof, 1) * MuJoCo_helper->returnModelTimeStep();
            }
            else{
                B.block(0, cols[i], dof, 1) = dqposdctrl.block(0, cols[i], dof, 1);

                B.block(dof, cols[i], dof, 1) = dqveldctrl.block(0, cols[i], dof, 1);
            }
        }
    }

//    std::cout << "time of sim integration: " << time_mj_forwards / 1000.0f << "\n";
//    std::cout << "num of sim integration: " << count_integrations << "\n";
//    std::cout << "diff time: "  << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - diff_start).count() / 1000.0 << std::endl;
}

void Differentiator::FD_Controls(MatrixXd &dqveldctrl, MatrixXd &dqposdctrl, const std::vector<int> &cols,
                                  int dataIndex, int tid, bool central_diff, double eps,
                                  MatrixXd &dcostdctrl, bool fd_costDerivs, bool terminal){

    MatrixXd velocityInc(dof, 1);
    MatrixXd velocityDec(dof, 1);
    MatrixXd positionInc(dof, 1);
    MatrixXd positionDec(dof, 1);

    double costInc;
    double costDec;

    MatrixXd unperturbedControls = activeModelTranslator->ReturnControlVector(MuJoCo_helper->fd_data[tid]);
    for(int i = 0; i < num_ctrl; i++){

        bool computeColumn = false;
        for(int j = 0; j < cols.size(); j++){
            if(i == cols[j]){
                computeColumn = true;
            }
        }

        if(computeColumn){
            count_integrations++;
            // perturb control vector positively
            MatrixXd perturbedControls = unperturbedControls.replicate(1,1);
            perturbedControls(i) += eps;
            activeModelTranslator->SetControlVector(perturbedControls, MuJoCo_helper->fd_data[tid]);

            // Integrate the simulator
            auto start = std::chrono::high_resolution_clock::now();
//            MuJoCo_helper->stepSimulator(1, tid);
//            mj_step(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid]);
            mj_stepSkip(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid], mjSTAGE_VEL, 1);
            time_mj_forwards += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count();
//            cout << "after first step simulator - " << i << endl;

            // return the  new velcoity vector
            velocityInc = activeModelTranslator->returnVelocityVector(MuJoCo_helper->fd_data[tid]);
            positionInc = activeModelTranslator->returnPositionVector(MuJoCo_helper->fd_data[tid]);

            // If calculating cost derivatives
            if(fd_costDerivs){
                // Calculate cost
                costInc = activeModelTranslator->CostFunction(MuJoCo_helper->fd_data[tid], terminal);
            }

            // return data state back to initial data state
            MuJoCo_helper->copySystemState(MuJoCo_helper->fd_data[tid], MuJoCo_helper->savedSystemStatesList[dataIndex]);

            // perturb control vector in opposite direction
            perturbedControls = unperturbedControls.replicate(1, 1);
            perturbedControls(i) -= eps;

            // integrate simulator
            start = std::chrono::high_resolution_clock::now();
//            MuJoCo_helper->stepSimulator(1, tid);
//            mj_step(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid]);
            mj_stepSkip(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid], mjSTAGE_VEL, 1);
            time_mj_forwards += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count();
//            cout << "after second step simulator - " << i << endl;

            // return the new velocity vector
            velocityDec = activeModelTranslator->returnVelocityVector(MuJoCo_helper->fd_data[tid]);
            positionDec = activeModelTranslator->returnPositionVector(MuJoCo_helper->fd_data[tid]);

            // If calculating cost derivatives via finite-differencing
            if(fd_costDerivs){
                // Calculate cost
                costDec = activeModelTranslator->CostFunction(MuJoCo_helper->fd_data[tid], terminal);
            }

            // Calculate one column of the dqveldctrl matrix
            for(int j = 0; j < dof; j++){
                dqveldctrl(j, i) = (velocityInc(j) - velocityDec(j))/(2*eps);
                dqposdctrl(j, i) = (positionInc(j) - positionDec(j))/(2*eps);

            }

            if(fd_costDerivs){
                dcostdctrl(i, 0) = (costInc - costDec)/(2*eps);
            }

            // Undo perturbation
            MuJoCo_helper->copySystemState(MuJoCo_helper->fd_data[tid], MuJoCo_helper->savedSystemStatesList[dataIndex]);
        }
    }
}

void Differentiator::FD_Velcoities(MatrixXd &dqveldqvel, MatrixXd &dqposdqvel, const std::vector<int> &cols,
                                   int dataIndex, int tid, bool central_diff, double eps,
                                   MatrixXd &dcostdvel, bool fd_costDerivs, bool terminal){

    // Increment and decrement vectors
    MatrixXd velocityInc(dof, 1);
    MatrixXd velocityDec(dof, 1);
    MatrixXd positionInc(dof, 1);
    MatrixXd positionDec(dof, 1);

    double costInc;
    double costDec;

    MatrixXd unperturbedVelocities = activeModelTranslator->returnVelocityVector(MuJoCo_helper->fd_data[tid]);

    for(int i = 0; i < dof; i++){
        bool computeColumn = false;

        for(int j = 0; j < cols.size(); j++){
            if(i == cols[j]){
                computeColumn = true;
            }
        }

        if(computeColumn){
            count_integrations++;
            // Perturb velocity vector positively
            MatrixXd perturbedVelocities = unperturbedVelocities.replicate(1, 1);
            perturbedVelocities(i) += eps;
            activeModelTranslator->setVelocityVector(perturbedVelocities, MuJoCo_helper->fd_data[tid]);

            // Integrate the simulator
            auto start = std::chrono::high_resolution_clock::now();
//            mj_step(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid]);
            mj_stepSkip(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid], mjSTAGE_POS, 1);
            time_mj_forwards += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count();

            // return the new velocity vector
            velocityInc = activeModelTranslator->returnVelocityVector(MuJoCo_helper->fd_data[tid]);
            positionInc = activeModelTranslator->returnPositionVector(MuJoCo_helper->fd_data[tid]);

            // If calculating cost derivs via finite-differencing
            if(fd_costDerivs){
                costInc = activeModelTranslator->CostFunction(MuJoCo_helper->fd_data[tid], terminal);
            }

            // reset the data state back to initial data state
            MuJoCo_helper->copySystemState(MuJoCo_helper->fd_data[tid], MuJoCo_helper->savedSystemStatesList[dataIndex]);

            // perturb velocity vector negatively
            perturbedVelocities = unperturbedVelocities.replicate(1, 1);
            perturbedVelocities(i) -= eps;
            activeModelTranslator->setVelocityVector(perturbedVelocities, MuJoCo_helper->fd_data[tid]);

            // Integrate the simulatormodel
            start = std::chrono::high_resolution_clock::now();
//            mj_step(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid]);
            mj_stepSkip(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid], mjSTAGE_POS, 1);
            time_mj_forwards += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count();

            // Return the new velocity vector
            velocityDec = activeModelTranslator->returnVelocityVector(MuJoCo_helper->fd_data[tid]);
            positionDec = activeModelTranslator->returnPositionVector(MuJoCo_helper->fd_data[tid]);

            // If calculating cost derivs via finite-differencing
            if(fd_costDerivs){
                costDec = activeModelTranslator->CostFunction(MuJoCo_helper->fd_data[tid], terminal);
            }

            // Calculate one column of the dqveldqvel matrix
            for(int j = 0; j < dof; j++){
                dqveldqvel(j, i) = (velocityInc(j) - velocityDec(j))/(2*eps);
                dqposdqvel(j, i) = (positionInc(j) - positionDec(j))/(2*eps);
            }

            if(fd_costDerivs){
                dcostdvel(i, 0) = (costInc - costDec)/(2*eps);
            }

            // Undo perturbation
            MuJoCo_helper->copySystemState(MuJoCo_helper->fd_data[tid], MuJoCo_helper->savedSystemStatesList[dataIndex]);
        }
    }
}

void Differentiator::FD_Positions(MatrixXd &dqveldqpos, MatrixXd &dqposdqpos, const std::vector<int> &cols,
                                  int dataIndex, int tid, bool central_diff, double eps,
                                  MatrixXd &dcostdpos, bool fd_costDerivs, bool terminal){

    // Increment and decrement vectors
    MatrixXd velocityInc(dof, 1);
    MatrixXd velocityDec(dof, 1);
    MatrixXd positionInc(dof, 1);
    MatrixXd positionDec(dof, 1);

    double costInc;
    double costDec;

    MatrixXd unperturbedPositions = activeModelTranslator->returnPositionVector(MuJoCo_helper->fd_data[tid]);

    int num_integrations = 0;
    double time_integrations_here = 0.0f;

    for(int i = 0; i < dof; i++){
        bool computeColumn = false;
        for(int j = 0; j < cols.size(); j++) {
            if (i == cols[j]) {
                computeColumn = true;
            }
        }

        if(computeColumn){
            count_integrations++;
            num_integrations++;
            // Perturb position vector positively
            MatrixXd perturbedPositions = unperturbedPositions.replicate(1, 1);
            perturbedPositions(i) += eps;
            activeModelTranslator->setPositionVector(perturbedPositions, MuJoCo_helper->fd_data[tid]);

            // Integrate the simulator
            auto start = std::chrono::high_resolution_clock::now();
//            mj_step(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid]);
            mj_stepSkip(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid], mjSTAGE_NONE, 1);
            time_mj_forwards += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count();
            time_integrations_here += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count();
//                cout << "after step simulator positon - " << i << endl;

            // return the new velocity vector
            velocityInc = activeModelTranslator->returnVelocityVector(MuJoCo_helper->fd_data[tid]);
            positionInc = activeModelTranslator->returnPositionVector(MuJoCo_helper->fd_data[tid]);
//                cout << "Velocity positive " << velocityInc << endl;

            if(fd_costDerivs){
                costInc = activeModelTranslator->CostFunction(MuJoCo_helper->fd_data[tid], terminal);
            }

            // reset the data state back to initial data statedataIndex
            MuJoCo_helper->copySystemState(MuJoCo_helper->fd_data[tid], MuJoCo_helper->savedSystemStatesList[dataIndex]);

            // perturb position vector negatively
            perturbedPositions = unperturbedPositions.replicate(1, 1);
            perturbedPositions(i) -= eps;
            activeModelTranslator->setPositionVector(perturbedPositions, MuJoCo_helper->fd_data[tid]);

            // Integrate the simulator
            start = std::chrono::high_resolution_clock::now();
//            mj_step(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid]);
            mj_stepSkip(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid], mjSTAGE_NONE, 1);
            time_mj_forwards += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count();
            time_integrations_here += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count();

            // Return the decremented vectors
            velocityDec = activeModelTranslator->returnVelocityVector(MuJoCo_helper->fd_data[tid]);
            positionDec = activeModelTranslator->returnPositionVector(MuJoCo_helper->fd_data[tid]);

            if(fd_costDerivs){
                costDec = activeModelTranslator->CostFunction(MuJoCo_helper->fd_data[tid], terminal);
            }

            // Calculate one column of the dqaccdq matrix
            for(int j = 0; j < dof; j++){
                dqveldqpos(j, i) = (velocityInc(j) - velocityDec(j))/(2*eps);
                dqposdqpos(j, i) = (positionInc(j) - positionDec(j))/(2*eps);
            }

            if(fd_costDerivs){
                dcostdpos(i, 0) = (costInc - costDec)/(2*eps);
            }

            // Undo perturbation
            MuJoCo_helper->copySystemState(MuJoCo_helper->fd_data[tid], MuJoCo_helper->savedSystemStatesList[dataIndex]);
        }
    }
//    std::cout << "num integrations in dqvel/dqpos: " << num_integrations << std::endl;
//    std::cout << "time integrations in dqvel/dqpos: " << time_integrations_here << std::endl;
}

//void Differentiator::calc_dqaccdctrl(MatrixXd &dqaccdctrl, const std::vector<int> &cols, int dataIndex, int tid, MatrixXd &dcostdctrl, bool fd_costDerivs, bool terminal){
//
//    MatrixXd acellInc(dof, 1);
//    MatrixXd acellDec(dof, 1);
//    double costInc;
//    double costDec;
//
//    double temp_time_forwards = 0.0f;
//    double temp_time_copy = 0.0f;
//
//    MatrixXd unperturbedControls = activeModelTranslator->ReturnControlVector(MuJoCo_helper->fd_data[tid]);
//    MatrixXd perturbedControls = unperturbedControls.replicate(1,1);
//
//    for(int i = 0; i < num_ctrl; i++){
//        bool computeColumn = false;
//        for(int j = 0; j < cols.size(); j++){
//            if(i == cols[j]){
//                computeColumn = true;
//            }
//        }
//
//        if(computeColumn){
//            // perturb control vector positively
//            perturbedControls = unperturbedControls.replicate(1,1);
//            perturbedControls(i) += epsControls;
//            activeModelTranslator->SetControlVector(perturbedControls, MuJoCo_helper->fd_data[tid]);
//
//            // Integrate the simulator
//            auto start = std::chrono::high_resolution_clock::now();
//            mj_forwardSkip(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid], mjSTAGE_VEL, 1);
//            time_mj_forwards += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count();
//            temp_time_forwards += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - start).count();
//            acellInc = activeModelTranslator->returnAccelerationVector(MuJoCo_helper->fd_data[tid]);
//
//            // If calculating cost derivatives
//            if(fd_costDerivs){
//                // Calculate cost
//                costInc = activeModelTranslator->CostFunction(MuJoCo_helper->fd_data[tid], terminal);
//            }
//
//            // return data state back to initial data state
//            start = std::chrono::high_resolution_clock::now();
//            MuJoCo_helper->cpMjData(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid], MuJoCo_helper->savedSystemStatesList[dataIndex]);
//            temp_time_copy += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - start).count();
//
//            // perturb control vector in opposite direction
//            perturbedControls = unperturbedControls.replicate(1, 1);
//            perturbedControls(i) -= epsControls;
//
//            start = std::chrono::high_resolution_clock::now();
//            mj_forwardSkip(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid], mjSTAGE_VEL, 1);
//            time_mj_forwards += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count();
//            temp_time_forwards += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - start).count();
//
//            acellDec = activeModelTranslator->returnAccelerationVector(MuJoCo_helper->fd_data[tid]);
//
//            // If calculating cost derivatives via finite-differencing
//            if(fd_costDerivs){
//                // Calculate cost
//                costDec = activeModelTranslator->CostFunction(MuJoCo_helper->fd_data[tid], terminal);
//            }
//
//            // Calculate one column of the dqveldctrl matrix
//            for(int j = 0; j < dof; j++){
//                dqaccdctrl(j, i) = (acellInc(j) - acellDec(j))/(2*epsControls);
//            }
//
//            if(fd_costDerivs){
//                dcostdctrl(i, 0) = (costInc - costDec)/(2*epsControls);
//            }
//
//            // Undo pertubation
//            start = std::chrono::high_resolution_clock::now();
//            MuJoCo_helper->cpMjData(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid], MuJoCo_helper->savedSystemStatesList[dataIndex]);
//            temp_time_copy += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - start).count();
//        }
//    }
////
////    std::cout << "time forward in calc_dqaccdctrl - " << temp_time_forwards / 1000000.0f << std::endl;
////    std::cout << "time copy state in calc_dqaccdctrl - " << temp_time_copy / 1000000.0f << std::endl;
//}
//
//void Differentiator::calc_dqaccdqvel(MatrixXd &dqaccdqvel, const std::vector<int> &cols, int dataIndex, int tid, MatrixXd &dcostdvel, bool fd_costDerivs, bool terminal){
//
//    MatrixXd acellInc(dof, 1);
//    MatrixXd acellDec(dof, 1);
//    double costInc;
//    double costDec;
//    MatrixXd unperturbedVelocities = activeModelTranslator->returnVelocityVector(MuJoCo_helper->fd_data[tid]);
//
//    for(int i = 0; i < dof; i++){
//        bool computeColumn = false;
//
//        for(int j = 0; j < cols.size(); j++){
//            if(i == cols[j]){
//                computeColumn = true;
//            }
//        }
//
//        if(computeColumn){
//            // Perturb velocity vector positively
//            MatrixXd perturbedVelocities = unperturbedVelocities.replicate(1, 1);
//            perturbedVelocities(i) += epsVelocities;
//            activeModelTranslator->setVelocityVector(perturbedVelocities, MuJoCo_helper->fd_data[tid]);
//
//            //Integrate the simulator
//            auto start = std::chrono::high_resolution_clock::now();
//            mj_forwardSkip(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid], mjSTAGE_POS, 1);
//            time_mj_forwards += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count();
//
//            acellInc = activeModelTranslator->returnAccelerationVector(MuJoCo_helper->fd_data[tid]);
//
//            // If calculating cost derivs via finite-differencing
//            if(fd_costDerivs){
//                costInc = activeModelTranslator->CostFunction(MuJoCo_helper->fd_data[tid], terminal);
//            }
//
//            // reset the data state back to initial data state
////            MuJoCo_helper->copySystemState(tid, dataIndex);
//            MuJoCo_helper->cpMjData(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid], MuJoCo_helper->savedSystemStatesList[dataIndex]);
//
//            // perturb velocity vector negatively
//            perturbedVelocities = unperturbedVelocities.replicate(1, 1);
//            perturbedVelocities(i) -= epsVelocities;
//            activeModelTranslator->setVelocityVector(perturbedVelocities, MuJoCo_helper->fd_data[tid]);
//
//            // Integrate the simulator
//            start = std::chrono::high_resolution_clock::now();
////            MuJoCo_helper->forwardSimulatorWithSkip(tid, mjSTAGE_POS, 1);
//            mj_forwardSkip(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid], mjSTAGE_POS, 1);
//            time_mj_forwards += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count();
//
//            acellDec = activeModelTranslator->returnAccelerationVector(MuJoCo_helper->fd_data[tid]);
//
//            // If calculating cost derivs via finite-differencing
//            if(fd_costDerivs){
//                costDec = activeModelTranslator->CostFunction(MuJoCo_helper->fd_data[tid], terminal);
//            }
//
//            // Calculate one column of the dqveldqvel matrix
//            for(int j = 0; j < dof; j++){
//                dqaccdqvel(j, i) = (acellInc(j) - acellDec(j))/(2*epsVelocities);
//            }
//
//            if(fd_costDerivs){
//                dcostdvel(i, 0) = (costInc - costDec)/(2*epsVelocities);
//            }
//
//            // Undo perturbation
//            MuJoCo_helper->cpMjData(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid], MuJoCo_helper->savedSystemStatesList[dataIndex]);
//        }
//    }
//}
//
//void Differentiator::calc_dqaccdqpos(MatrixXd &dqaccdqpos, const std::vector<int> &cols,
//                                     int dataIndex, int tid, MatrixXd &dcostdpos, bool fd_costDerivs, bool terminal){
//
//    MatrixXd unperturbedPositions = activeModelTranslator->returnPositionVector(MuJoCo_helper->fd_data[tid]);
//    MatrixXd accellInc(dof, 1);
//    MatrixXd accellDec(dof, 1);
//    double costInc;
//    double costDec;
//
//    for(int i = 0; i < dof; i++){
//
//        bool computeColumn = false;
//
//        for(int j = 0; j < cols.size(); j++){
//            if(i == cols[j]){
//                computeColumn = true;
//            }
//        }
//
//        if(computeColumn){
//            // Perturb position vector positively
//            MatrixXd perturbedPositions = unperturbedPositions.replicate(1, 1);
//            perturbedPositions(i) += epsPositions;
//            activeModelTranslator->setPositionVector(perturbedPositions, MuJoCo_helper->fd_data[tid]);
//
//            // Integrate the simulator
//            auto start = std::chrono::high_resolution_clock::now();
////            MuJoCo_helper->forwardSimulatorWithSkip(tid, mjSTAGE_NONE, 0);
//            mj_forwardSkip(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid], mjSTAGE_NONE, 1);
//            time_mj_forwards += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count();
//
//            accellInc = activeModelTranslator->returnAccelerationVector(MuJoCo_helper->fd_data[tid]);
//
//            if(fd_costDerivs){
//                costInc = activeModelTranslator->CostFunction(MuJoCo_helper->fd_data[tid], terminal);
//            }
//
//            //Reset data state
//            MuJoCo_helper->cpMjData(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid], MuJoCo_helper->savedSystemStatesList[dataIndex]);
//
//            //Perturb position vector negatively
//            perturbedPositions = unperturbedPositions.replicate(1, 1);
//            perturbedPositions(i) -= epsPositions;
//            activeModelTranslator->setPositionVector(perturbedPositions, MuJoCo_helper->fd_data[tid]);
//
//            start = std::chrono::high_resolution_clock::now();
//            mj_forwardSkip(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid], mjSTAGE_NONE, 1);
//            time_mj_forwards += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count();
//
//            accellDec = activeModelTranslator->returnAccelerationVector(MuJoCo_helper->fd_data[tid]);
//
//            if(fd_costDerivs){
//                costDec = activeModelTranslator->CostFunction(MuJoCo_helper->fd_data[tid], terminal);
//            }
//
//            // Calculate one column of the dqaccdq matrix
//            for(int j = 0; j < dof; j++){
//                dqaccdqpos(j, i) = (accellInc(j) - accellDec(j))/(2*epsPositions);
//            }
//
//            if(fd_costDerivs){
//                dcostdpos(i, 0) = (costInc - costDec)/(2*epsPositions);
//            }
//
//            // Undo perturbation
//            MuJoCo_helper->cpMjData(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid], MuJoCo_helper->savedSystemStatesList[dataIndex]);
//        }
//    }
//}

//#pragma GCC pop_options


// finite difference cost derivatives graveyard
//if(costDerivs) {
//l_x.block(0, 0, dof, 1) = dcostdpos;
//l_x.block(dof, 0, dof, 1) = dcostdvel;
//
//// ------------- l_u / l_uu -------------------
////                dqcostdctrl
//// --------------------------------------------
//l_u.block(0, 0, numCtrl, 1) = dcostdctrl;
//}
//
//MatrixXd l_x_inc(dof*2, 1);
//MatrixXd l_x_dec(dof*2, 1);
//MatrixXd l_u_inc(numCtrl, 1);
//MatrixXd l_u_dec(numCtrl, 1);
//
//MatrixXd currentState = activeModelTranslator->ReturnStateVector(physicsHelperId);
//MatrixXd currentControl = activeModelTranslator->ReturnControlVector(physicsHelperId);
//
//if(costDerivs && !HESSIAN_APPROXIMATION) {
//double epsCost = 1e-6;
//double epsTest = 1e-6;
//
//MatrixXd test1;
//MatrixXd test2;
//
//for(int i = 0; i < 2*dof; i++){
//MatrixXd perturbedState = currentState.replicate(1, 1);
//perturbedState(i) += epsCost;
//
//for(int j = 0; j < 2*dof; j++){
////                cout << "perturb state index " << i << " with state index " << j << endl;
//MatrixXd stateInc = perturbedState.replicate(1,1);
//stateInc(j) += epsTest;
////                cout << "state inc pos: " << endl << stateInc << endl;
//
//activeModelTranslator->SetStateVector(stateInc, physicsHelperId);
////                active_physics_simulator->forwardSimulator(physicsHelperId);
////                active_physics_simulator->forwardSimulatorWithSkip(physicsHelperId, mjSTAGE_NONE, 0);
//MuJoCo_helper->stepSimulator(1, physicsHelperId);
//MuJoCo_helper->forwardSimulator(physicsHelperId);
//test1 = activeModelTranslator->ReturnStateVector(physicsHelperId);
//costInc = activeModelTranslator->CostFunction(physicsHelperId, terminal);
//MuJoCo_helper->copySystemState(physicsHelperId, dataIndex);
//
//MatrixXd stateDec = perturbedState.replicate(1,1);
//stateDec(j) -= epsTest;
////                cout << "state dec: " << endl << stateDec << endl;
//
//activeModelTranslator->SetStateVector(stateDec, physicsHelperId);
////                active_physics_simulator->forwardSimulator(physicsHelperId);
////                active_physics_simulator->forwardSimulatorWithSkip(physicsHelperId, mjSTAGE_NONE, 0);
//MuJoCo_helper->stepSimulator(1, physicsHelperId);
//MuJoCo_helper->forwardSimulator(physicsHelperId);
//test2 = activeModelTranslator->ReturnStateVector(physicsHelperId);
//costDec = activeModelTranslator->CostFunction(physicsHelperId, terminal);
//MuJoCo_helper->copySystemState(physicsHelperId, dataIndex);
////                cout << "difference in returned states: " << endl << test1 - test2 << endl;
////                cout << "cost inc: " << costInc << endl;
////                cout << "cost dec: " << costDec << endl;
//
//l_x_inc(j, 0) = (costInc - costDec)/(2*epsTest);
//}
//
////            cout << "perturbed state pos: " << endl << perturbedState << endl;
////            cout << "l_x_inc: " << endl << l_x_inc << endl;
//
//perturbedState = currentState.replicate(1, 1);
//perturbedState(i) -= epsCost;
//
//for(int j = 0; j < 2*dof; j++){
//MatrixXd stateInc = perturbedState.replicate(1,1);
//stateInc(j) += epsTest;
//
//activeModelTranslator->SetStateVector(stateInc, physicsHelperId);
////                active_physics_simulator->forwardSimulator(physicsHelperId);
//MuJoCo_helper->stepSimulator(1, physicsHelperId);
//MuJoCo_helper->forwardSimulator(physicsHelperId);
//costInc = activeModelTranslator->CostFunction(physicsHelperId, terminal);
//MuJoCo_helper->copySystemState(physicsHelperId, dataIndex);
//
//MatrixXd stateDec = perturbedState.replicate(1,1);
//stateDec(j) -= epsTest;
//
//activeModelTranslator->SetStateVector(stateDec, physicsHelperId);
////                active_physics_simulator->forwardSimulator(physicsHelperId);
//MuJoCo_helper->stepSimulator(1, physicsHelperId);
//MuJoCo_helper->forwardSimulator(physicsHelperId);
//costDec = activeModelTranslator->CostFunction(physicsHelperId, terminal);
//MuJoCo_helper->copySystemState(physicsHelperId, dataIndex);
//
//l_x_dec(j, 0) = (costInc - costDec)/(2*epsTest);
//}
//
////            cout << "perturbed state neg: " << endl << perturbedState << endl;
////            cout << "l_x_dec: " << endl << l_x_dec << endl;
//
//// New l_x at perturbed position i
//for(int j = 0; j < 2*dof; j++){
//l_xx(j, i) = (l_x_inc(j, 0) - l_x_dec(j, 0))/(2*epsCost);
//}
//}
//
//for(int i = 0; i < numCtrl; i++){
//MatrixXd perturbedControl = currentControl.replicate(1, 1);
//perturbedControl(i) += epsCost;
//
//for(int j = 0; j < numCtrl; j++){
//MatrixXd controlInc = perturbedControl.replicate(1,1);
//controlInc(j) += epsCost;
//
//activeModelTranslator->SetControlVector(controlInc, physicsHelperId);
//
//MuJoCo_helper->stepSimulator(1, physicsHelperId);
//MuJoCo_helper->forwardSimulator(physicsHelperId);
//costInc = activeModelTranslator->CostFunction(physicsHelperId, terminal);
//MuJoCo_helper->copySystemState(physicsHelperId, dataIndex);
//
//MatrixXd controlDec = perturbedControl.replicate(1,1);
//controlDec(j) -= epsCost;
//
//activeModelTranslator->SetControlVector(controlDec, physicsHelperId);
//
//MuJoCo_helper->stepSimulator(1, physicsHelperId);
//MuJoCo_helper->forwardSimulator(physicsHelperId);
//costDec = activeModelTranslator->CostFunction(physicsHelperId, terminal);
//MuJoCo_helper->copySystemState(physicsHelperId, dataIndex);
//
//l_u_inc(j, 0) = (costInc - costDec)/(2*epsCost);
//}
//
//perturbedControl = currentControl.replicate(1, 1);
//perturbedControl(i) -= epsCost;
//
//for(int j = 0; j < numCtrl; j++){
//MatrixXd controlInc = perturbedControl.replicate(1,1);
//controlInc(j) += epsCost;
//
//activeModelTranslator->SetControlVector(controlInc, physicsHelperId);
//
//MuJoCo_helper->stepSimulator(1, physicsHelperId);
//MuJoCo_helper->forwardSimulator(physicsHelperId);
//costInc = activeModelTranslator->CostFunction(physicsHelperId, terminal);
//MuJoCo_helper->copySystemState(physicsHelperId, dataIndex);
//
//MatrixXd controlDec = perturbedControl.replicate(1,1);
//controlDec(j) -= epsCost;
//
//activeModelTranslator->SetControlVector(controlDec, physicsHelperId);
//
//MuJoCo_helper->stepSimulator(1, physicsHelperId);
//MuJoCo_helper->forwardSimulator(physicsHelperId);
//costDec = activeModelTranslator->CostFunction(physicsHelperId, terminal);
//MuJoCo_helper->copySystemState(physicsHelperId, dataIndex);
//
//l_u_dec(j, 0) = (costInc - costDec)/(2*epsCost);
//}
//
//for(int j = 0; j < numCtrl; j++){
//l_uu(j, i) = (l_u_inc(j, 0) - l_u_dec(j, 0))/(2*epsCost);
//}
//}
//}
////    cout << "l_x: " << endl << l_x << endl;
////    cout << "l_xx: " << endl << l_xx << endl;
//
//if(costDerivs && HESSIAN_APPROXIMATION){
//l_xx = l_x * l_x.transpose();
//l_uu = l_u * l_u.transpose();
//}
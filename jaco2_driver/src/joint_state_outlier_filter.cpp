#include <jaco2_driver/joint_state_outlier_filter.h>
JointStateOutlierFilter::JointStateOutlierFilter(double threshold_torque, double threshold_acc)
    : buffer_size_(3),
      threshold_torque_(threshold_torque),
      threshold_acc_(threshold_acc)
{

}

bool JointStateOutlierFilter::doFiltering()
{
    std::size_t nlinks = jstate_buffer_.front().name.size();

    bool result = false;
    std::vector<bool> tests_acc =  {false,false,false};
    std::vector<bool> tests_js =  {false,false,false};
    for(std::size_t i = 0; i < nlinks; ++i)
    {


        double tau1 = jstate_buffer_[0].effort[i];
        double tau2 = jstate_buffer_[1].effort[i];
        double tau3 = jstate_buffer_[2].effort[i];
        double dt_tau1 = (jstate_buffer_[1].stamp_micro_seconds - jstate_buffer_[0].stamp_micro_seconds) * 1e-6;
        double dt_tau2 = (jstate_buffer_[2].stamp_micro_seconds - jstate_buffer_[1].stamp_micro_seconds) * 1e-6;
        double dt_tau3 = (jstate_buffer_[2].stamp_micro_seconds - jstate_buffer_[0].stamp_micro_seconds) * 1e-6;
        double delta_tau1 = (tau2 - tau1) / dt_tau1;
        double delta_tau2 = (tau3 - tau2) / dt_tau2;
        double delta_tau3 = (tau3 - tau1) / dt_tau3;
        tests_js[0] = tests_js[0] || fabs(delta_tau1) > torque_tresh_;
        tests_js[1] = tests_js[1] || fabs(delta_tau2) > torque_tresh_;
        tests_js[2] = tests_js[2] || fabs(delta_tau3) > torque_tresh_;
        result |= ( tests_js[0] ||  tests_js[1]  || tests_js[2]);

    }

    for(std::size_t i = 0; i < acc_msg_buffer_.front().lin_acc.size(); ++i){
        jaco2_msgs::Vector3Stamped v1 = acc_msg_buffer_[0].lin_acc[i];
        jaco2_msgs::Vector3Stamped v2 = acc_msg_buffer_[1].lin_acc[i];
        jaco2_msgs::Vector3Stamped v3 = acc_msg_buffer_[2].lin_acc[i];

        double dt1 = (v2.stamp_micro_seconds - v1.stamp_micro_seconds) * 1e-6;
        double dt2 = (v3.stamp_micro_seconds - v2.stamp_micro_seconds) * 1e-6;
        double dt3 = (v3.stamp_micro_seconds - v1.stamp_micro_seconds) * 1e-6;


        jaco2_msgs::Vector3Stamped delta1 = (v2 - v1) / dt1;
        jaco2_msgs::Vector3Stamped delta2 = (v3 - v2) / dt2;
        jaco2_msgs::Vector3Stamped delta3 = (v3 - v1) / dt3;


        for(std::size_t j = 0; j < 3; ++ j){
            tests_acc[0] = tests_acc[0] || (fabs(delta1.vector[j]) > acc_thres_);
            tests_acc[1] = tests_acc[1] || (fabs(delta2.vector[j]) > acc_thres_);
            tests_acc[2] = tests_acc[2] || (fabs(delta3.vector[j]) > acc_thres_);

        }


        result |= ( tests_acc[0] ||  tests_acc[1]  || tests_acc[2]);

    }

    if(result){
        if(tests_js[0] && !tests_js[1] && !tests_js[2]){
            removeJsOutlier(1,2,0); // first is outlier
        }
        if(!tests_js[0] && tests_js[1] && !tests_js[2]){
            removeJsOutlier(0,2,1); // mid is outlier
        }
        if(!tests_js[0] && !tests_js[1] && tests_js[2]){
            removeJsOutlier(0,1,2); // last is outlier
        }
        if(!tests_js[0] && !tests_js[1] && tests_js[2]){
            removeJsOutlier(0,1,2); // last is outlier
        }
        if(tests_js[0] && tests_js[1] && !tests_js[2]){
            removeJsOutlier(0,2,1); // mid is outlier
        }
        if(tests_js[0] && !tests_js[1] && tests_js[2]){
            removeJsOutlier(1,2,0); // first is outlier
        }
        if(!tests_js[0] && tests_js[1] && tests_js[2]){
            removeJsOutlier(0,1,2); // last is outlier
        }
        if(!tests_js[0] && !tests_js[1] && !tests_js[2]){
            // not everything should be an outlier wrong threshold?
            return false;
        }

        if(tests_acc[0] && !tests_acc[1] && !tests_acc[2]){
            removeAccOutlier(1,2,0); // first is outlier
        }
        if(!tests_acc[0] && tests_acc[1] && !tests_acc[2]){
            removeAccOutlier(0,2,1); // mid is outlier
        }
        if(!tests_acc[0] && !tests_acc[1] && tests_acc[2]){
            removeAccOutlier(0,1,2); // last is outlier
        }
        if(!tests_acc[0] && !tests_acc[1] && tests_acc[2]){
            removeAccOutlier(0,1,2); // last is outlier
        }
        if(tests_acc[0] && tests_acc[1] && !tests_acc[2]){
            removeAccOutlier(0,2,1); // mid is outlier
        }
        if(tests_acc[0] && !tests_acc[1] && tests_acc[2]){
            removeAccOutlier(1,2,0); // first is outlier
        }
        if(!tests_acc[0] && tests_acc[1] && tests_acc[2]){
            removeAccOutlier(0,1,2); // last is outlier
        }
        if(!tests_acc[0] && !tests_acc[1] && !tests_acc[2]){
            // not everything should be an outlier wrong threshold?
            return false;
        }

    }
    return true;
}




void JointStateOutlierFilter::removeJsOutlier(std::size_t i, std::size_t j, std::size_t outlier)
{
    jaco2_msgs::JointState state;

    state.name = jstate_buffer_[i].name;
    std::size_t nj =jstate_buffer_[i].position.size();
    state.position.resize(nj);
    state.velocity.resize(nj);
    state.effort.resize(nj);
    state.acceleration.resize(nj);
    state.stamp_micro_seconds = jstate_buffer_[outlier].stamp_micro_seconds;
    state.label = jstate_buffer_[outlier].label;

    for(std::size_t k = 0; k < nj; ++k){
        state.position[k] = 0.5 * (jstate_buffer_[i].position[k] + jstate_buffer_[j].position[k]);
        state.velocity[k] = 0.5 * (jstate_buffer_[i].velocity[k] + jstate_buffer_[j].velocity[k]);
        state.acceleration[k] = 0.5 * (jstate_buffer_[i].acceleration[k] + jstate_buffer_[j].acceleration[k]);
        state.effort[k] = 0.5 * (jstate_buffer_[i].effort[k] + jstate_buffer_[j].effort[k]);
    }

    jstate_buffer_[outlier] = state;
}

void JointStateOutlierFilter::removeAccOutlier(std::size_t i, std::size_t j, std::size_t outlier)
{
    std::size_t naccs = acc_msg_buffer_[i].lin_acc.size();
    jaco2_msgs::Accelerometers accs(naccs);

    accs.label = acc_msg_buffer_[outlier].label;
    for(std::size_t k = 0; k < naccs; ++k){
        jaco2_msgs::Vector3Stamped mean = (acc_msg_buffer_[i].lin_acc[k] + acc_msg_buffer_[j].lin_acc[k]) * 0.5;
        acc_msg_buffer_[outlier].lin_acc[k] = mean;
    }
}

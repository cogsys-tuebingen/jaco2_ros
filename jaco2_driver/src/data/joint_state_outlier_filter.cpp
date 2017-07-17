#include <jaco2_driver/data/joint_state_outlier_filter.h>
#include <iostream>
using namespace jaco2_data;

JointStateOutlierFilter::JointStateOutlierFilter(double threshold_torque, double threshold_acc)
    : buffer_size_(3),
      threshold_torque(threshold_torque),
      threshold_acc(threshold_acc)
{

}

bool JointStateOutlierFilter::filter(const ExtendedJointStateData &data_in, ExtendedJointStateData &out)
{
    bool success = true;

    if(jstate_buffer_.empty()){
        jstate_buffer_.push_back(data_in);
    }
    else{
        if(data_in.joint_state.stamp != jstate_buffer_.back().joint_state.stamp){
            jstate_buffer_.push_back(data_in);
        }
    }


    while(jstate_buffer_.size() > buffer_size_){
        jstate_buffer_.pop_front();
    }

    if(jstate_buffer_.size() == buffer_size_){
        success = doFiltering();
    }
    out = jstate_buffer_.back();

    return success;
}

bool JointStateOutlierFilter::doFiltering()
{
    bool result = false;
    std::vector<bool> tests_acc =  {false,false,false};
    std::vector<bool> tests_js =  {false,false,false};

    result |= checkTorques(tests_js);
    result |= checkAccs(tests_acc);



    if(result){
        if(tests_js[0] && !tests_js[1] && !tests_js[2]){
            removeOutlier(1,2,0); // first is outlier
        }
        if(!tests_js[0] && tests_js[1] && !tests_js[2]){
            removeOutlier(0,2,1); // mid is outlier
        }
        if(!tests_js[0] && !tests_js[1] && tests_js[2]){
            removeOutlier(0,1,2); // last is outlier
        }
        if(!tests_js[0] && !tests_js[1] && tests_js[2]){
            removeOutlier(0,1,2); // last is outlier
        }
        if(tests_js[0] && tests_js[1] && !tests_js[2]){
            removeOutlier(0,2,1); // mid is outlier
        }
        if(tests_js[0] && !tests_js[1] && tests_js[2]){
            removeOutlier(1,2,0); // first is outlier
        }
        if(!tests_js[0] && tests_js[1] && tests_js[2]){
            removeOutlier(0,1,2); // last is outlier
        }

        if(tests_acc[0] && !tests_acc[1] && !tests_acc[2]){
            removeOutlier(1,2,0); // first is outlier
        }
        if(!tests_acc[0] && tests_acc[1] && !tests_acc[2]){
            removeOutlier(0,2,1); // mid is outlier
        }
        if(!tests_acc[0] && !tests_acc[1] && tests_acc[2]){
            removeOutlier(0,1,2); // last is outlier
        }
        if(!tests_acc[0] && !tests_acc[1] && tests_acc[2]){
            removeOutlier(0,1,2); // last is outlier
        }
        if(tests_acc[0] && tests_acc[1] && !tests_acc[2]){
            removeOutlier(0,2,1); // mid is outlier
        }
        if(tests_acc[0] && !tests_acc[1] && tests_acc[2]){
            removeOutlier(1,2,0); // first is outlier
        }
        if(!tests_acc[0] && tests_acc[1] && tests_acc[2]){
            removeOutlier(0,1,2); // last is outlier
        }
        if(tests_acc[0] && tests_acc[1] && tests_acc[2]){
            // not everything should be an outlier wrong threshold?
            return false;
        }

    }
    return true;
}

bool JointStateOutlierFilter::checkTorques(std::vector<bool> &test)
{
    bool result = false;
    JointStateData& js_st = jstate_buffer_[0].joint_state;
    JointStateData& js_nd = jstate_buffer_[1].joint_state;
    JointStateData& js_rd = jstate_buffer_[2].joint_state;

    double dt_tau1 = js_nd.stamp.substractionResultInSeconds(js_st.stamp);
    double dt_tau2 = js_rd.stamp.substractionResultInSeconds(js_nd.stamp);
    double dt_tau3 = js_rd.stamp.substractionResultInSeconds(js_st.stamp);;

    auto it1 = js_st.torque.begin();
    auto it2 = js_nd.torque.begin();

    for(auto it3 = js_rd.torque.begin(); it3 < js_rd.torque.end(); ++it3)
    {
        double tau1 = *it1;
        double tau2 = *it2;
        double tau3 = *it3;
        double delta_tau1 = (tau2 - tau1) / dt_tau1;
        double delta_tau2 = (tau3 - tau2) / dt_tau2;
        double delta_tau3 = (tau3 - tau1) / dt_tau3;
        test[0] = test[0] || fabs(delta_tau1) > threshold_torque;
        test[1] = test[1] || fabs(delta_tau2) > threshold_torque;
        test[2] = test[2] || fabs(delta_tau3) > threshold_torque;
        result |= ( test[0] ||  test[1]  || test[2]);
        ++it1;
        ++it2;
    }
    return result;
}

bool JointStateOutlierFilter::checkAccs(std::vector<bool> &test)
{
    bool result = false;
    auto it_acc1 = jstate_buffer_[0].lin_acc.begin();
    auto it_acc2 = jstate_buffer_[1].lin_acc.begin();
    AccelerometerData& linacc3 = jstate_buffer_[2].lin_acc;
    for(auto it_acc3 = linacc3.begin(); it_acc3 <linacc3.end(); ++it_acc3){
        Vector3Stamped& v1 = *it_acc1;
        Vector3Stamped& v2 = *it_acc2;
        Vector3Stamped& v3 = *it_acc3;

        double dt1 = v2.stamp.substractionResultInSeconds(v1.stamp);
        double dt2 = v3.stamp.substractionResultInSeconds(v2.stamp);
        double dt3 = v3.stamp.substractionResultInSeconds(v1.stamp);;

        Vector3Stamped delta1 = (v2 - v1) / dt1;
        Vector3Stamped delta2 = (v3 - v2) / dt2;
        Vector3Stamped delta3 = (v3 - v1) / dt3;

        for(std::size_t j = 0; j < 3; ++ j){
            test[0] = test[0] || (fabs(delta1.vector(j)) > threshold_acc);
            test[1] = test[1] || (fabs(delta2.vector(j)) > threshold_acc);
            test[2] = test[2] || (fabs(delta3.vector(j)) > threshold_acc);

        }

        result |= ( test[0] ||  test[1]  || test[2]);

        ++it_acc1;
        ++it_acc2;
    }
    return result;
}

void JointStateOutlierFilter::removeOutlier(std::size_t i, std::size_t j, std::size_t outlier)
{
    removeJsOutlier(i, j, outlier);
    removeAccOutlier(i, j, outlier);
}

void JointStateOutlierFilter::removeJsOutlier(std::size_t i, std::size_t j, std::size_t outlier)
{
    JointStateData state;

    JointStateData& js_i = jstate_buffer_[i].joint_state;
    JointStateData& js_j = jstate_buffer_[j].joint_state;
    JointStateData& js_out = jstate_buffer_[outlier].joint_state;

    state.names = js_i.names;
    std::size_t nj =js_i.position.size();
    state.position.resize(nj);
    state.velocity.resize(nj);
    state.torque.resize(nj);
    state.acceleration.resize(nj);
    state.stamp.fromNSec(0.5 * (js_i.stamp.toMicroSec() + js_j.stamp.toMicroSec()));
    state.label = js_out.label;

    state.gravity =0.5 * (js_i.gravity + js_j.gravity);

    for(std::size_t k = 0; k < nj; ++k){
        state.position[k] = 0.5 * (js_i.position[k] + js_j.position[k]);
        state.velocity[k] = 0.5 * (js_i.velocity[k] + js_j.velocity[k]);
        state.acceleration[k] = 0.5 * (js_i.acceleration[k] + js_j.acceleration[k]);
        state.torque[k] = 0.5 * (js_i.torque[k] + js_j.torque[k]);
    }

    js_out = state;
}

void JointStateOutlierFilter::removeAccOutlier(std::size_t i, std::size_t j, std::size_t outlier)
{

    const AccelerometerData& a_i = jstate_buffer_[i].lin_acc;
    const AccelerometerData& a_j = jstate_buffer_[j].lin_acc;
    AccelerometerData& a_out = jstate_buffer_[outlier].lin_acc;
    std::size_t naccs = a_i.size();

    for(std::size_t k = 0; k < naccs; ++k){
        Vector3Stamped mean = (a_i[k] + a_j[k]) * 0.5;
        mean.stamp.fromNSec(0.5 * (a_i[k].stamp.toMicroSec() + a_j[k].stamp.toMicroSec()));
        a_out[k] = mean;
    }
}

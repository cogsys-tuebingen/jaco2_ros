#include <jaco2_driver/gripper_controller.h>
#include <math.h>
#include <kinova/KinovaTypes.h>

GripperController::GripperController(Jaco2State &state, Jaco2API& api)
    : Jaco2Controller(state, api),
      usePos_(false),
      threshold_(1),
      counter_(1)
{
    tp_.InitStruct();
    tp_.Position.Type = ANGULAR_VELOCITY;
    tp_.Position.HandMode = VELOCITY_MODE;
    for(std::size_t i = 0; i < 3; ++i){
        fingerVelocity_[i] = 2000;
    }

}

bool GripperController::isDone() const
{
    return done_;
}

void GripperController::start()
{
}

void GripperController::grabObj(const bool& useFinger1, const bool& useFinger2, const bool& useFinger3)
{
    useFingers_[0] = useFinger1;
    useFingers_[1] = useFinger2;
    useFingers_[2] = useFinger3;
    usePos_ = false;
    counter_ = 1;
    lastPosition_ = state_.getAngularPosition();
    done_ = false;
    last_command_ = std::chrono::high_resolution_clock::now();
}

void GripperController::grabObjSetUnusedFingerPos(const bool &useFinger1, const bool &useFinger2, const bool &useFinger3, const int posFinger1, const int posFinger2, const int posFinger3)
{
    fingerGoal_[0] = posFinger1;
    fingerGoal_[1] = posFinger2;
    fingerGoal_[2] = posFinger3;
    grabObj(useFinger1,useFinger2,useFinger3);
    usePos_ = true;
}

void GripperController::read()
{
    state_.readPosVel();
}

void GripperController::write()
{
    currentPosition_ = state_.getAngularPosition();
    double d_diff[3];
    diff_[0] = fingerGoal_[0] - currentPosition_.Fingers.Finger1;
    diff_[1] = fingerGoal_[1] - currentPosition_.Fingers.Finger2;
    diff_[2] = fingerGoal_[2] - currentPosition_.Fingers.Finger3;

//    std::cout << "current pos " << currentPosition_.Fingers.Finger1 << ", " << currentPosition_.Fingers.Finger2 << ", " << currentPosition_.Fingers.Finger3 << std::endl;
//    std::cout << "diffs " <<diff_[0] << ", " << diff_[1] << ", " << diff_[2] << std::endl;
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now - last_command_ ;
    double dt = std::chrono::duration_cast<std::chrono::microseconds>(duration).count()*1e-6;
    last_command_ = now;

    if(notMoving() && reachedGoal() && counter_ > 100){
          done_ = true;
          tp_.Position.Fingers.Finger1 = 0;
          tp_.Position.Fingers.Finger2 = 0;
          tp_.Position.Fingers.Finger3 = 0;
          api_.setAngularVelocity(tp_);
//          std::cout << "done" << std::endl;
          return;
    }
    else{
        for(std::size_t i = 0; i < 3; ++i){
            if(!useFingers_[i]){
                eSum_[i] += dt*diff_[i];
                d_diff[i] = (diff_[i] - eLast_[i])/dt;
                eLast_[i] = diff_[i];
            }
        }

        if(useFingers_[0]){
            tp_.Position.Fingers.Finger1 = fingerVelocity_[0];
        }
        else{
            tp_.Position.Fingers.Finger1 = gainP_[0] * diff_[0] + gainI_[0]*eSum_[0] + gainD_[0]*d_diff[0];
        }

        if(useFingers_[1]){
            tp_.Position.Fingers.Finger2 = fingerVelocity_[1];
        }
        else{
            tp_.Position.Fingers.Finger2 = gainP_[1] * diff_[1] + gainI_[1]*eSum_[1] + gainD_[1]*d_diff[1];
        }

        if(useFingers_[2]){
            tp_.Position.Fingers.Finger3 = fingerVelocity_[2];
        }
        else{
            tp_.Position.Fingers.Finger3 = gainP_[2] * diff_[2] + gainI_[2]*eSum_[2] + gainD_[2]*d_diff[2];
        }

//        std::cout <<" velocities: " <<tp_.Position .Fingers.Finger1 << ", " << tp_.Position.Fingers.Finger2 << ", " << tp_.Position.Fingers.Finger3 << std::endl;

        api_.setAngularVelocity(tp_);
//        start_ = false;
    }
}

bool GripperController::notMoving()
{
   bool result = true;
   if(useFingers_[0])
   {
       result &= fabs(currentPosition_.Fingers.Finger1 - lastPosition_.Fingers.Finger1) < threshold_;
//       std::cout << "finger 1 diff: " << fabs(currentPosition_.Fingers.Finger1 - lastPosition_.Fingers.Finger1) << std::endl;
   }
   if(useFingers_[1])
   {
       result &= fabs(currentPosition_.Fingers.Finger2 - lastPosition_.Fingers.Finger2) < threshold_;
//       std::cout << "finger 2 diff: " << fabs(currentPosition_.Fingers.Finger2 - lastPosition_.Fingers.Finger2) << std::endl;
   }
   if(useFingers_[2])
   {
       result &= fabs(currentPosition_.Fingers.Finger3 - lastPosition_.Fingers.Finger3) < threshold_;
//       std::cout << "finger 3 diff: " << fabs(currentPosition_.Fingers.Finger3 - lastPosition_.Fingers.Finger3) << std::endl;
   }
   if(counter_ % 100 == 0)
   {
       lastPosition_ = currentPosition_;
//       std::cout << "change" << std::endl;
   }
//   counter_ = (counter_ + 1) % 10;
   ++counter_;

   return result;
}

bool GripperController::reachedGoal()
{
    if(usePos_)
    {
        bool result = true;
        for(std::size_t i = 0; i < 3; ++i)
        {
            if(!useFingers_[i])
            {
                bool test = fabs(diff_[i]) < 20;
                result &= test;
                if(test)
                {
                    diff_[i] = 0;
                }
            }
        }
        return result;
    }
    else
    {
        return true;
    }
}

void GripperController::setGainP(const double finger1, const double finger2, const double finger3)
{
    gainP_[0] = finger1;
    gainP_[1] = finger2;
    gainP_[2] = finger3;
}

void GripperController::setGainI(const double finger1, const double finger2, const double finger3)
{
    gainI_[0] = finger1;
    gainI_[1] = finger2;
    gainI_[2] = finger3;
}

void GripperController::setGainD(const double finger1, const double finger2, const double finger3)
{
    gainD_[0] = finger1;
    gainD_[1] = finger2;
    gainD_[2] = finger3;
}

void GripperController::setFingerVelocity(const int finger1, const int finger2, const int finger3)
{
    fingerVelocity_[0] = finger1;
    fingerVelocity_[1] = finger2;
    fingerVelocity_[2] = finger3;
}




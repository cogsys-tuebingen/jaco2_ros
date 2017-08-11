#include <jaco2_driver/controller/trajectory_tracking_controller.h>

TrajectoryTrackingController::TrajectoryTrackingController(Jaco2State &state, Jaco2API &api, TerminationCallback& t )
    : Jaco2Controller(state, api, t),
      current_point_(0)
{

}


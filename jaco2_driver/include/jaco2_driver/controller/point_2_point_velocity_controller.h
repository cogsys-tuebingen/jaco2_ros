#ifndef POINT_2_POINT_VELOCITY_CONTROLLER_H
#define POINT_2_POINT_VELOCITY_CONTROLLER_H

#include <jaco2_driver/controller/trajectory_tracking_controller.h>
#include <jaco2_driver/joint_trajectory.h>
#include <jaco2_driver/manipulator_info.h>
#include <jaco2_driver/jaco2_api.h>
#include <chrono>

class Point2PointVelocityController : public TajectoryTrackingController
{
public:
    Point2PointVelocityController(Jaco2State &state, Jaco2API& api);

    virtual void write() override;
    virtual void start() override;

    virtual bool isDone() const;



protected:
    void pidController(const double dt);
    void simpleVelController(const double dt);





};
#endif // POINT_2_POINT_VELOCITY_CONTROLLER_H


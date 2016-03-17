#ifndef JOINTTRAJECTORY_H
#define JOINTTRAJECTORY_H
#include <vector>
#include <stdexcept>
#include <kinova/KinovaTypes.h>

class JointTrajectory{

public:
    JointTrajectory():
        points_(0)
    {
    }
    ~JointTrajectory()
    {
    }

    void set(const std::vector<AngularInfo>& pos, const std::vector<AngularInfo>& vel,
             const std::vector<AngularInfo>& acc, const std::vector<double> time_from_start)
    {
        if(pos.size() == acc.size() && acc.size() == vel.size() && acc.size() == time_from_start.size())
        {
            position_ = pos;
            velocity_ = vel;
            accerleration_ = acc;
            time_from_start_ = time_from_start;
            points_ = pos.size();
        }
        else
        {
            throw std::logic_error("Position, velocity and accerleraration have to conatin the same amount of points.");
        }
    }


    void setJointNames(const std::vector<std::string> & name)
    {
        jointNames_ = name;
    }

    void push_back(AngularInfo& pos, AngularInfo& vel, AngularInfo& acc, double time_from_start)
    {
        position_.push_back(pos);
        velocity_.push_back(vel);
        accerleration_.push_back(acc);
        time_from_start_.push_back(time_from_start);
        points_ = position_.size();
    }
    void push_back(std::string name)
    {
        jointNames_.push_back(name);
    }

    std::vector<std::string> getJointNames() const {return jointNames_;}
    std::vector<AngularInfo> getPosition() const {return position_;}
    std::vector<AngularInfo> getVelocity() const {return velocity_;}
    std::vector<AngularInfo> getAccerleration() const {return accerleration_;}
    std::size_t getNumPoints() const {return points_;}

    double getPosition(std::size_t point, std::size_t joint) const
    {
        switch (joint)
        {
        case 0:
            return position_[point].Actuator1;
        case 1:
            return position_[point].Actuator2;
        case 2:
            return position_[point].Actuator3;
        case 3:
            return position_[point].Actuator4;
        case 4:
            return position_[point].Actuator5;
        case 5:
            return position_[point].Actuator6;
        default :
            throw std::logic_error("Illegal Joint Index");
        }
    }

    double getVelocity(std::size_t point, std::size_t joint) const
    {
        switch (joint)
        {
        case 0:
            return velocity_[point].Actuator1;
        case 1:
            return velocity_[point].Actuator2;
        case 2:
            return velocity_[point].Actuator3;
        case 3:
            return velocity_[point].Actuator4;
        case 4:
            return velocity_[point].Actuator5;
        case 5:
            return velocity_[point].Actuator6;
        default :
            throw std::logic_error("Illegal Joint Index");
        }
    }

    double getAcceleration(std::size_t point, std::size_t joint) const
    {
        switch (joint)
        {
        case 0:
            return accerleration_[point].Actuator1;
        case 1:
            return accerleration_[point].Actuator2;
        case 2:
            return accerleration_[point].Actuator3;
        case 3:
            return accerleration_[point].Actuator4;
        case 4:
            return accerleration_[point].Actuator5;
        case 5:
            return accerleration_[point].Actuator6;
        default :
            throw std::logic_error("Illegal Joint Index");
        }
    }

    double getTimeFromStart(std::size_t idx)
    {
        return time_from_start_[idx];
    }

    void resize(std::size_t n)
    {
        position_.resize(n);
        velocity_.resize(n);
        accerleration_.resize(n);
        time_from_start_.resize(n);
        points_ = n;
    }
    std::size_t size() const { return points_;}
    std::size_t numJoints() const {return jointNames_.size();}
    void setTimeFromStart(std::size_t point, double value)
    {
        time_from_start_[point] = value;
    }

    void setPosition(std::size_t point, std::size_t joint, double value)
    {
        switch (joint)
        {
        case 0:
            position_[point].Actuator1 = value;
            break;
        case 1:
            position_[point].Actuator2 = value;
            break;
        case 2:
            position_[point].Actuator3 = value;
            break;
        case 3:
            position_[point].Actuator4 = value;
            break;
        case 4:
            position_[point].Actuator5 = value;
            break;
        case 5:
            position_[point].Actuator6 = value;
            break;
        default :
            throw std::logic_error("Illegal Joint Index");
        }
    }

    void setVelocity(std::size_t point, std::size_t joint, double value)
    {
        switch (joint)
        {
        case 0:
            velocity_[point].Actuator1 = value;
            break;
        case 1:
            velocity_[point].Actuator2 = value;
            break;
        case 2:
            velocity_[point].Actuator3 = value;
            break;
        case 3:
            velocity_[point].Actuator4 = value;
            break;
        case 4:
            velocity_[point].Actuator5 = value;
            break;
        case 5:
            velocity_[point].Actuator6 = value;
            break;
        default :
            throw std::logic_error("Illegal Joint Index");
        }
    }

    void setAcceleration(std::size_t point, std::size_t joint, double value)
    {
        switch (joint)
        {
        case 0:
            accerleration_[point].Actuator1 = value;
            break;
        case 1:
            accerleration_[point].Actuator2 = value;
            break;
        case 2:
            accerleration_[point].Actuator3 = value;
            break;
        case 3:
            accerleration_[point].Actuator4 = value;
            break;
        case 4:
            accerleration_[point].Actuator5 = value;
            break;
        case 5:
            accerleration_[point].Actuator6 = value;
            break;
        default :
            throw std::logic_error("Illegal Joint Index");
        }
    }


private:
    std::vector<std::string> jointNames_;
    std::vector<AngularInfo> position_;
    std::vector<AngularInfo> velocity_;
    std::vector<AngularInfo> accerleration_;
    std::vector<double> time_from_start_;
    std::size_t points_;
};

#endif // JOINTTRAJECTORY_H


#include <string>
#include <vector>
#include <math.h>
#include <random>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <jaco2_data/torque_offset_lut.hpp>
using namespace Jaco2Calibration;
TorqueOffsetLut lut;

TEST(TorqueOffsetLUT, setAt)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0, 1);

    for(std::size_t i = 1; i < 6; ++i){
        for(std::size_t n = 0; n < lut.steps(i); ++n){
            double angle = lut.lower_limits(i) + lut.resolution(i) * n;
            double rand = dis(gen);
            lut.set(i, angle, rand);
            EXPECT_NEAR(lut.at(i,angle),rand,1e-8);
        }
    }

}

TEST(TorqueOffsetLUT, saveLoad)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0, 1);

    for(std::size_t i = 1; i < 6; ++i){
        for(std::size_t n = 0; n < lut.steps(i); ++n){
            double angle = lut.lower_limits(i) + lut.resolution(i) * n;
            double rand = dis(gen);
            lut.set(i, angle, rand);
            EXPECT_NEAR(lut.at(i,angle),rand,1e-8);
        }
    }

    lut.save("/tmp/test_lut.yaml");

    TorqueOffsetLut lut2;
    lut2.load("/tmp/test_lut.yaml");

    int rows,cols,rows2,cols2;
    rows = lut.lut.rows();
    cols = lut.lut.cols();
    rows2 = lut2.lut.rows();
    cols2 = lut2.lut.cols();
    EXPECT_EQ(rows, 6);
    EXPECT_EQ(rows, rows2);
    EXPECT_EQ(cols, 2*72);
    EXPECT_EQ(cols, cols2);
    for(std::size_t i = 0; i < rows; ++i){
        for(std::size_t j = 0; j < cols; ++j){
            EXPECT_NEAR(lut.lut(i,j), lut2.lut(i,j),1e-10);
        }
    }
}

TEST(TorqueOffsetLUT, initialize)
{
    Eigen::VectorXd limits;
    limits.setOnes(lut.n_links);
    limits *= 720;

    Eigen::VectorXd res;
    res.setOnes(lut.n_links);
    res *= 10.0;


    for(std::size_t i = 0; i < 6; ++i){
        EXPECT_NEAR(lut.lower_limits(i), -limits(i), 1e-10);
        EXPECT_NEAR(lut.resolution(i), res(i), 1e-10);

        std::size_t step = std::floor((2.0 * limits(i))/ res(i));
        EXPECT_EQ(lut.steps(i), step);
    }



}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);

    Eigen::VectorXd limits;
    limits.setOnes(lut.n_links);
    limits *= 720;
    lut.lower_limits = - limits;
    Eigen::VectorXd res;
    res.setOnes(lut.n_links);
    res *= 10.0;
    lut.resolution = res;


    lut.initialize(-limits, limits, res);
    return RUN_ALL_TESTS();
    return 0;
}

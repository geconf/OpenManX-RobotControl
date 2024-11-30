#include <eigen3/Eigen/Dense>
#include "dh_parameters.cpp"

class ForwardKinematics
{
    public:
        Eigen::Matrix<float, 4, 4> getEndEffectorPoseDh(DhParameters dhParams)
        {
            return getJointPoseDh(dhParams, dhParams.getNumberOfJoints());
        }

        Eigen::Matrix<float, 4, 4> getJointPoseDh(DhParameters dhParams, const uint jointId)
        {
            Eigen::Matrix<float, 4, 4> currentPose;
            currentPose.setIdentity();
            for(uint i = 0; i < jointId; i++) 
            {
                currentPose = transformDh(currentPose, 
                        dhParams.getA()[i], 
                        dhParams.getD()[i], 
                        dhParams.getTheta()[i], 
                        dhParams.getAlpha()[i]);
            }
            return currentPose;
        }

    private:
        Eigen::Matrix<float, 4, 4> transformDh(Eigen::Matrix<float, 4, 4> currentPose, 
                float a, float d, float theta, float alpha) const
        {
            Eigen::Matrix<float, 4, 4> dhFormulation;
            dhFormulation << cosf(theta), -sinf(theta)*cosf(alpha), sinf(theta)*sinf(alpha), a*cosf(theta),
                          sinf(theta), cosf(theta)*cosf(alpha), -cosf(theta)*sinf(alpha), a*sinf(theta),
                          0, sinf(alpha), cosf(alpha), d,
                          0, 0, 0, 1;

            return currentPose*dhFormulation;
        }
};


#include "forward_kinematics.cpp"
#include <eigen3/Eigen/src/Core/Matrix.h>

// Meta robot class
class Robot 
{
    public:
      void updateDhParams(const std::vector<float> a, const std::vector<float> d,
                const std::vector<float> theta, const std::vector<float> alpha)
      {
          dhParams_.setDhParameters(a, d, theta, alpha);
      }

      Eigen::Matrix<float, 4, 4> getEndEffectorPose()
      {
          return fk_.getEndEffectorPoseDh(dhParams_);
      }

      Eigen::Matrix<float, 4, 4> getJointPose(uint jointId)
      {
          return fk_.getJointPoseDh(dhParams_, jointId);
      }

    public:
      ForwardKinematics fk_;
      DhParameters dhParams_;
};

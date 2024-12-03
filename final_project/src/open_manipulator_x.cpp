#include "robot.cpp"
#include <cmath>
#include "geometry_msgs/msg/twist.hpp"

namespace openManX
{
    namespace {
        void setDhParams(Robot &openManX, std::vector<float> jointValues) 
        {
            const std::vector<float> a = { 0, 130.23, 124, 133.4 };
            const std::vector<float> d = { 60.25, 0, 0, 0 };
            const std::vector<float> theta = { jointValues[0], jointValues[1]-(float)M_PI/2+atan2f(24,128), 
                jointValues[2]+(float)M_PI/2-atan2f(24,128), jointValues[3] };
            const std::vector<float> alpha { -(float)M_PI/2, 0, 0, 0 };

            openManX.updateDhParams(a, d, theta, alpha);
        }
    }

    Eigen::Matrix<float, 4, 4> getEndEffectorPose(Robot &openManX, std::vector<float> jointValues)
    {
      setDhParams(openManX, jointValues);
      return openManX.getEndEffectorPose();
    }

    std::vector<float> getJointValues(Robot &openManX, const float endEffectorX, 
            const float endEffectorY, const float endEffectorZ, 
            const Eigen::Matrix<float, 3, 3> rotEndEffector)
    {
        const Eigen::Vector<float, 3> wristCenterOrientation = { 1, 0, 0 };
        const float d4 = 133.4;
        const Eigen::Vector<float, 3> wristCenterVector = d4*rotEndEffector*wristCenterOrientation;
        const float wristX = endEffectorX-wristCenterVector[0];
        const float wristY  = endEffectorY-wristCenterVector[1];
        const float wristZ = endEffectorZ-wristCenterVector[2];
        const float q1 = atan2f(wristY, wristX);

        const double d1 = 60.25;
        const double a2 = sqrt(pow(128, 2) + pow(24, 2));
        const double a3 = 124;
        const double d = (pow(wristX, 2) + pow(wristY, 2) + pow(wristZ-d1, 2) - pow(a2, 2) - pow(a3, 2))/(2*a2*a3);
        float tempDOp = 1-pow(d, 2);
        // The following line is VERY important
        // in the case where the theoretical value of d is 1, 
        // if the pose is given with high precision
        // 1-pow(d, 2) could return a value that's different than 0.
        // It could be negative. And thus q2 and q3 would be nan (undefined).
        // Trust me, this has happened before.
        // if you don't believe me, feel free to remove the next line
        // and set the pose to:
        // x=1.414214
        // y=1.414214
        // z=1.0
        // :)
        tempDOp = std::abs(tempDOp) < 1e-5 ? 0 : tempDOp;
        const float q3Minus = atan2(-sqrt(tempDOp), d);
        const float q3 = q3Minus; // q3Minus corresponds with the elbow up position, which may be better to prevent collisions
        const float q2 = atan2f(wristZ-d1, sqrt(pow(wristX, 2) + pow(wristY, 2))) - atan2f(a3*sinf(q3), a2 + a3*cosf(q3)) - atan2f(24, 128);

        const std::vector<float> jointValues = { q1, q2, q3, 0 };
        setDhParams(openManX, jointValues);
        const Eigen::Matrix<float, 4, 4> endEffectorPose = openManX.getJointPose(3);

        const Eigen::Matrix<float, 3, 3> rotJoint3To0 = endEffectorPose.block<3, 3>(0, 0);

        const Eigen::Matrix<float, 3, 3> rotJoint4To3 = rotJoint3To0.transpose()*rotEndEffector;

        const float q4 = acosf(rotJoint4To3(0,0));

        return { q1, q2, q3, q4 };
    }

    const uint LinkNum = 4;
    Eigen::Matrix<float, 6, LinkNum> GetJacobian(Robot &openManX, const std::vector<float> jointPositions) 
    {
        setDhParams(openManX, jointPositions);

        // Vectors z and o are necessary to construct the jacobian
        Eigen::Vector3f z[LinkNum];
        Eigen::Vector3f o[LinkNum+1];
        for (uint i = 0; i <= LinkNum; i++)
        {
          const Eigen::Matrix<float, 4, 4> jointPose = openManX.getJointPose(i);
          // We fill in the z and o vectors from the current pose
          if (i < LinkNum) // We don't need the last value of z
          {
              z[i] = jointPose.col(2).head<3>();
          }
          o[i] = jointPose.col(3).head<3>();
        }

        // Build the jacobian, column by column
        Eigen::Matrix<float, 6, LinkNum> j = Eigen::Matrix<float, 6, LinkNum>::Zero();
        for (uint i = 0; i < LinkNum; i++)
        {
          const Eigen::Vector3f jv = z[i].cross(o[LinkNum] - o[i]);
          Eigen::Vector<float, 6> jColumn;
          jColumn << jv, z[i];
          j.col(i) = jColumn;
        }
        return j;
    }

    Eigen::Vector<float, 6> GetTwist(
            Robot &openManX, 
            const std::vector<float> jointPosition,
            std::vector<float> jointVelocity)
    {
        Eigen::Matrix<float, 6, LinkNum> j = GetJacobian(openManX, jointPosition);
        Eigen::Vector<float, 6> twist = j*Eigen::Map<Eigen::Vector<float, LinkNum>>(jointVelocity.data());
        return twist;
    }
    
    Eigen::Vector<float, LinkNum> GetJointVelocity(
            Robot &openManX,
            const std::vector<float> jointPosition,
            const Eigen::Vector<float, 6> twist)
    {
        Eigen::Matrix<float, 6, LinkNum> j = GetJacobian(openManX, jointPosition);

        Eigen::Matrix<float, LinkNum, 6> jInverse = j.completeOrthogonalDecomposition().pseudoInverse();
        Eigen::Vector<float, LinkNum> jointVelocity = jInverse*twist;

        return jointVelocity;
    }
};

/*
Created on Wed Dec 05 10:25:20 2018

@author: Cong Liu

 Software License Agreement (BSD License)

 Copyright (c) 2018, Han's Robot Co., Ltd.
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  * Neither the name of the copyright holders nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/
// author: Cong Liu

#include<elfin_kinematic_solver/elfin_kinematic_solver.h>

namespace elfin10_ikfast_simple_api {

#define IKFAST_NO_MAIN  // Don't include main() from IKFast

/// \brief The types of inverse kinematics parameterizations supported.
///
/// The minimum degree of freedoms required is set in the upper 4 bits of each type.
/// The number of values used to represent the parameterization ( >= dof ) is the next 4 bits.
/// The lower bits contain a unique id of the type.
enum IkParameterizationType
{
  IKP_None = 0,
  IKP_Transform6D = 0x67000001,    ///< end effector reaches desired 6D transformation
  IKP_Rotation3D = 0x34000002,     ///< end effector reaches desired 3D rotation
  IKP_Translation3D = 0x33000003,  ///< end effector origin reaches desired 3D translation
  IKP_Direction3D = 0x23000004,    ///< direction on end effector coordinate system reaches desired direction
  IKP_Ray4D = 0x46000005,          ///< ray on end effector coordinate system reaches desired global ray
  IKP_Lookat3D = 0x23000006,       ///< direction on end effector coordinate system points to desired 3D position
  IKP_TranslationDirection5D = 0x56000007,  ///< end effector origin and direction reaches desired 3D translation and
  /// direction. Can be thought of as Ray IK where the origin of the ray must
  /// coincide.
  IKP_TranslationXY2D = 0x22000008,             ///< 2D translation along XY plane
  IKP_TranslationXYOrientation3D = 0x33000009,  ///< 2D translation along XY plane and 1D rotation around Z axis. The
  /// offset of the rotation is measured starting at +X, so at +X is it 0,
  /// at +Y it is pi/2.
  IKP_TranslationLocalGlobal6D = 0x3600000a,  ///< local point on end effector origin reaches desired 3D global point

  IKP_TranslationXAxisAngle4D = 0x4400000b,  ///< end effector origin reaches desired 3D translation, manipulator
  /// direction makes a specific angle with x-axis  like a cone, angle is from
  /// 0-pi. Axes defined in the manipulator base link's coordinate system)
  IKP_TranslationYAxisAngle4D = 0x4400000c,  ///< end effector origin reaches desired 3D translation, manipulator
  /// direction makes a specific angle with y-axis  like a cone, angle is from
  /// 0-pi. Axes defined in the manipulator base link's coordinate system)
  IKP_TranslationZAxisAngle4D = 0x4400000d,  ///< end effector origin reaches desired 3D translation, manipulator
  /// direction makes a specific angle with z-axis like a cone, angle is from
  /// 0-pi. Axes are defined in the manipulator base link's coordinate system.

  IKP_TranslationXAxisAngleZNorm4D = 0x4400000e,  ///< end effector origin reaches desired 3D translation, manipulator
  /// direction needs to be orthogonal to z-axis and be rotated at a
  /// certain angle starting from the x-axis (defined in the manipulator
  /// base link's coordinate system)
  IKP_TranslationYAxisAngleXNorm4D = 0x4400000f,  ///< end effector origin reaches desired 3D translation, manipulator
  /// direction needs to be orthogonal to x-axis and be rotated at a
  /// certain angle starting from the y-axis (defined in the manipulator
  /// base link's coordinate system)
  IKP_TranslationZAxisAngleYNorm4D = 0x44000010,  ///< end effector origin reaches desired 3D translation, manipulator
  /// direction needs to be orthogonal to y-axis and be rotated at a
  /// certain angle starting from the z-axis (defined in the manipulator
  /// base link's coordinate system)

  IKP_NumberOfParameterizations = 16,  ///< number of parameterizations (does not count IKP_None)

  IKP_VelocityDataBit =
      0x00008000,  ///< bit is set if the data represents the time-derivate velocity of an IkParameterization
  IKP_Transform6DVelocity = IKP_Transform6D | IKP_VelocityDataBit,
  IKP_Rotation3DVelocity = IKP_Rotation3D | IKP_VelocityDataBit,
  IKP_Translation3DVelocity = IKP_Translation3D | IKP_VelocityDataBit,
  IKP_Direction3DVelocity = IKP_Direction3D | IKP_VelocityDataBit,
  IKP_Ray4DVelocity = IKP_Ray4D | IKP_VelocityDataBit,
  IKP_Lookat3DVelocity = IKP_Lookat3D | IKP_VelocityDataBit,
  IKP_TranslationDirection5DVelocity = IKP_TranslationDirection5D | IKP_VelocityDataBit,
  IKP_TranslationXY2DVelocity = IKP_TranslationXY2D | IKP_VelocityDataBit,
  IKP_TranslationXYOrientation3DVelocity = IKP_TranslationXYOrientation3D | IKP_VelocityDataBit,
  IKP_TranslationLocalGlobal6DVelocity = IKP_TranslationLocalGlobal6D | IKP_VelocityDataBit,
  IKP_TranslationXAxisAngle4DVelocity = IKP_TranslationXAxisAngle4D | IKP_VelocityDataBit,
  IKP_TranslationYAxisAngle4DVelocity = IKP_TranslationYAxisAngle4D | IKP_VelocityDataBit,
  IKP_TranslationZAxisAngle4DVelocity = IKP_TranslationZAxisAngle4D | IKP_VelocityDataBit,
  IKP_TranslationXAxisAngleZNorm4DVelocity = IKP_TranslationXAxisAngleZNorm4D | IKP_VelocityDataBit,
  IKP_TranslationYAxisAngleXNorm4DVelocity = IKP_TranslationYAxisAngleXNorm4D | IKP_VelocityDataBit,
  IKP_TranslationZAxisAngleYNorm4DVelocity = IKP_TranslationZAxisAngleYNorm4D | IKP_VelocityDataBit,

  IKP_UniqueIdMask = 0x0000ffff,   ///< the mask for the unique ids
  IKP_CustomDataBit = 0x00010000,  ///< bit is set if the ikparameterization contains custom data, this is only used
  /// when serializing the ik parameterizations
};

// Code generated by IKFast56/61
#include "elfin10_ikfast_solver_ebl.cpp"

int num_joints_=6;

int solve(const Eigen::Vector3d &trans, const Eigen::Matrix3d &orient, const std::vector<double> &vfree,
          IkSolutionList<IkReal>& solutions)
{
    solutions.Clear();

    double point[3];
    point[0]=trans(0);
    point[1]=trans(1);
    point[2]=trans(2);

    double vals[9];
    vals[0] = orient(0, 0);
    vals[1] = orient(0, 1);
    vals[2] = orient(0, 2);
    vals[3] = orient(1, 0);
    vals[4] = orient(1, 1);
    vals[5] = orient(1, 2);
    vals[6] = orient(2, 0);
    vals[7] = orient(2, 1);
    vals[8] = orient(2, 2);

    ComputeIk(point, vals, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
    return solutions.GetNumSolutions();
}

void getSolution(const IkSolutionList<IkReal>& solutions, int i, std::vector<double>& solution)
{
  solution.clear();
  solution.resize(num_joints_);

  // IKFast56/61
  const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
  std::vector<IkReal> vsolfree(sol.GetFree().size());
  sol.GetSolution(&solution[0], vsolfree.size() > 0 ? &vsolfree[0] : NULL);
}

bool Elfin10KinematicSolver::getPositionIK(const Eigen::Vector3d& trans, const Eigen::Matrix3d& orient, const std::vector<double>& vfree,
                   const std::vector<double>& seed_state, std::vector<double>& solution)
{
    if(seed_state.size()!=6)
    {
        return false;
    }

    IkSolutionList<IkReal> solutions;
    int numsol = solve(trans, orient, vfree, solutions);

    std::vector<LimitObeyingSol> solutions_obey_limits;
    LimitObeyingSol solution_tmp;

    if (numsol)
    {
      std::vector<double> solution_obey_limits;
      for (std::size_t s = 0; s < numsol; ++s)
      {
          // All elements of this solution obey limits
          getSolution(solutions, s, solution_obey_limits);
          double dist_from_seed = 0.0;
          for (std::size_t i = 0; i < seed_state.size(); ++i)
          {
            dist_from_seed += fabs(seed_state[i] - solution_obey_limits[i]);
          }

          solution_tmp.value=solution_obey_limits;
          solution_tmp.dist_from_seed=dist_from_seed;
          solutions_obey_limits.push_back(solution_tmp);
      }
    }

    if (!solutions_obey_limits.empty())
    {
      std::sort(solutions_obey_limits.begin(), solutions_obey_limits.end());
      solution = solutions_obey_limits[0].value;
      return true;
    }

    return false;
}

bool Elfin10KinematicSolver::getPositionFK(const std::vector<double>& joint_angles, Eigen::Vector3d& trans, Eigen::Matrix3d& orient)
{
    if(joint_angles.size()!=6)
    {
        return false;
    }

    IkReal eerot[9], eetrans[3];
    IkReal angles[joint_angles.size()];
    for (unsigned char i = 0; i < joint_angles.size(); i++)
      angles[i] = joint_angles[i];

    // IKFast56/61
    ComputeFk(angles, eetrans, eerot);

    trans << eetrans[0], eetrans[1], eetrans[2];
    orient << eerot[0], eerot[1], eerot[2],
              eerot[3], eerot[4], eerot[5],
              eerot[6], eerot[7], eerot[8];
    return true;
}

}


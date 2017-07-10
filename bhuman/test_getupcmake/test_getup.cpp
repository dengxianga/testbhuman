#include "Modules/MotionControl/MotionCombinator.h"
#include "Representations/MotionControl/GetUpEngineOutput.h"
#include "Tools/Math/Pose2f.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Modules/MotionControl/ArmMotionCombinator.h"
#include "Modules/MotionControl/LegMotionCombinator.h"


// #include "Representations/Infrastructure/SensorData/KeyStates.h"
// #include "Representations/Configuration/DamageConfiguration.h"

#include <iostream>

// unsigned recoveryTime;

// class MCombinator {
// private:
//   OdometryData odometryData; /**< The odometry data. */
//   MotionInfo motionInfo; /**< Information about the motion currently executed. */
//   Pose2f specialActionOdometry; /**< workaround for accumulating special action odometry. */

//   unsigned currentRecoveryTime;

//   bool headYawInSafePosition = false;
//   bool headPitchInSafePosition = false;

//   OdometryData lastOdometryData;
//   JointRequest lastJointRequest;

// public:
//   MCombinator()  {}

//   /**
//    * The method copies all joint angles from one joint request to another,
//    * but only those that should not be ignored.
//    * @param source The source joint request. All angles != JointAngles::ignore will be copied.
//    * @param target The target joint request.
//    */
//   void copy(const JointRequest& source, JointRequest& target,
//                             const StiffnessSettings& theStiffnessSettings,
//                             const Joints::Joint startJoint, const Joints::Joint endJoint)
// {
//   for(int i = startJoint; i <= endJoint; ++i)
//   {
//     if(source.angles[i] != JointAngles::ignore)
//       target.angles[i] = source.angles[i];
//     target.stiffnessData.stiffnesses[i] = target.angles[i] != JointAngles::off ? source.stiffnessData.stiffnesses[i] : 0;
//     if(target.stiffnessData.stiffnesses[i] == StiffnessData::useDefault)
//       target.stiffnessData.stiffnesses[i] = theStiffnessSettings.stiffnesses[i];
//   }
// }

// void interpolate(const JointRequest& from, const JointRequest& to,
//                                    float fromRatio, JointRequest& target, bool interpolateStiffness,
//                                    const StiffnessSettings& theStiffnessSettings, const JointAngles& lastJointAngles,
//                                    const Joints::Joint startJoint, const Joints::Joint endJoint)
// {
//   for(int i = startJoint; i <= endJoint; ++i)
//   {
//     float f = from.angles[i];
//     float t = to.angles[i];

//     if(t == JointAngles::ignore && f == JointAngles::ignore)
//       continue;

//     if(t == JointAngles::ignore)
//       t = target.angles[i];
//     if(f == JointAngles::ignore)
//       f = target.angles[i];

//     int fStiffness = f != JointAngles::off ? from.stiffnessData.stiffnesses[i] : 0;
//     int tStiffness = t != JointAngles::off ? to.stiffnessData.stiffnesses[i] : 0;
//     if(fStiffness == StiffnessData::useDefault)
//       fStiffness = theStiffnessSettings.stiffnesses[i];
//     if(tStiffness == StiffnessData::useDefault)
//       tStiffness = theStiffnessSettings.stiffnesses[i];

//     if(t == JointAngles::off || t == JointAngles::ignore)
//       t = lastJointAngles.angles[i];
//     if(f == JointAngles::off || f == JointAngles::ignore)
//       f = lastJointAngles.angles[i];
//     if(target.angles[i] == JointAngles::off || target.angles[i] == JointAngles::ignore)
//       target.angles[i] = lastJointAngles.angles[i];

//     ASSERT(target.angles[i] != JointAngles::off && target.angles[i] != JointAngles::ignore);
//     ASSERT(t != JointAngles::off && t != JointAngles::ignore);
//     ASSERT(f != JointAngles::off && f != JointAngles::ignore);

//     target.angles[i] += -fromRatio * t + fromRatio * f;
//     if(interpolateStiffness)
//       target.stiffnessData.stiffnesses[i] += int(-fromRatio * float(tStiffness) + fromRatio * float(fStiffness));
//     else
//       target.stiffnessData.stiffnesses[i] = tStiffness;
//   }
// }

// void debugReleaseArms(JointRequest& jointRequest)
// {
//   /*if(theKeyStates.pressed[theKeyStates.lHandBack] ||
//     theKeyStates.pressed[theKeyStates.lHandLeft] ||
//     theKeyStates.pressed[theKeyStates.lHandRight])
//     for(unsigned i = Joints::firstLeftArmJoint; i <= Joints::lHand; i++)
//       jointRequest.stiffnessData.stiffnesses[i] = 0;

//   if(theKeyStates.pressed[theKeyStates.rHandBack] ||
//     theKeyStates.pressed[theKeyStates.rHandLeft] ||
//     theKeyStates.pressed[theKeyStates.rHandRight])
//     for(unsigned i = Joints::firstRightArmJoint; i <= Joints::rHand; i++)
//       jointRequest.stiffnessData.stiffnesses[i] = 0;*/

//   if(theKeyStates.pressed[theKeyStates.headFront]
//      || theKeyStates.pressed[theKeyStates.headMiddle]
//      || theKeyStates.pressed[theKeyStates.headRear])
//     for(unsigned i = Joints::firstLeftArmJoint; i <= Joints::rHand; i++)
//       jointRequest.stiffnessData.stiffnesses[i] = 0;
// }

// };

int main() {
	// Get GetUpEngine to generate mof file
	// GetUpEngine* gup = new GetUpEngine();
	// GetUpEngine gup;
	// // GetUpEngine gup();
	// Mof::Motion front = Mof::Motion(front);
	// gup.generateMofOfMotion(front);

	// Motion Combinator
	// MotionCombinator* mc = new MotionCombinator();
	MotionCombinator mc;

	// // Init OdometryData
	// OdometryData od;

	// // Initialize an GetUpEngineOutput
	// GetUpEngineOutput gupo;

	// // Initialize a Pose2f
	// Pose2f p2(1.0f, 2.0f);
	// Pose2f p2_2(1.0f, 3.0f);
	// Pose2f p2_3(1.0f, 2.0f);
	// std::cout << "pose 2 equals? " << (p2 == p2_2) << std::endl;
	// std::cout << "pose 3 equals? " << (p2 == p2_3) << std::endl;

	// // Initialize a MotionInfo
	// MotionInfo mi;
	// std::cout<< "is standing? " << mi.isStanding() << std::endl;

	// JointRequest jr;
	// std::cout << "joint request isValid? " << jr.isValid() << std::endl;

	// unsigned currentRecoveryTime;

	// MCombinator mc;

  // Initialize Arm and Leg motion combinator
  // ArmMotionCombinator amc;
  // LegMotionCombinator lmc;


	std::cout << "\nHello World!" << std::endl;
	return 0;
}
enum Joints { //ALL ENUMS HERE MUST BE IN THE SAME ORDER LIKE THE ENUMS BELOW
    headYaw = 0,
    headPitch,

    firstArmJoint,
    firstLeftArmJoint = firstArmJoint,

    lShoulderPitch = firstLeftArmJoint,
    lShoulderRoll,
    lElbowYaw,
    lElbowRoll,
    lWristYaw,
    lHand, //< not an Angle, instead %

    firstRightArmJoint,

    rShoulderPitch = firstRightArmJoint,
    rShoulderRoll,
    rElbowYaw,
    rElbowRoll,
    rWristYaw,
    rHand, //< not an Angle, instead %

    firstLegJoint,
    firstLeftLegJoint = firstLegJoint,

    lHipYawPitch = firstLeftLegJoint,
    lHipRoll,
    lHipPitch,
    lKneePitch,
    lAnklePitch,
    lAnkleRoll,

    firstRightLegJoint,

    rHipYawPitch = firstRightLegJoint, //< not a joint in the real nao
    rHipRoll,
    rHipPitch,
    rKneePitch,
    rAnklePitch,
    rAnkleRoll,
    numOfJoints
  };

enum JointArmVarieties {
    shoulderPitch,
    shoulderRoll,
    elbowYaw,
    elbowRoll,
    wristYaw,
    hand
  };

enum JointLegVarieties {
    hipYawPitch, //< not a joint in the real nao
    hipRoll,
    hipPitch,
    kneePitch,
    anklePitch,
    ankleRoll
  };


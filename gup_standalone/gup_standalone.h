#pragma once
#include "Joints.h"
#include "Pose2f.h"
#include "File.h"

struct StiffnessPair {
  StiffnessPair() {};
  StiffnessPair(Joints joint, int s) : joint(joint), s(s) {};
  Joints joint;
  int s;
};

struct MofLine {
  MofLine() {};
  float head[2];
  float leftArm[6];
  float rightArm[6];
  float leftLeg[6];
  float rightLeg[6];
  std::vector<StiffnessPair> singleMotorStiffnessChange;
  float duration;
  bool critical;
};

struct Odometry {
  Odometry() {};
  Odometry(float x, float y, float angle) : x(x), y(y), angle(angle) {};
  float x;
  float y;
  float angle;
};


struct Mof {
  Mof() {};
  enum Motion {
    front,
    frontFast,
    back,
    backFast,
    recovering,
    recoverFromSide,
    stand,
    fromSumo,
    fromSumoKeeper,
    fromSafeFall,
    numOfMotions
  };

  Motion t_;
  Mof(Motion t) : t_(t) {}
  operator Motion () const {return t_;}

  std::vector<MofLine> lines;
  Motion name;
  int baseLimbStiffness[5];
  int balanceStartLine;
  // Pose2f odometryOffset;
  Odometry odometryOffset;
};

struct GetUpEngineParameters {
  GetUpEngineParameters() {};
  int maxNumOfUnsuccessfulTries;
  Mof mofs[Mof::numOfMotions];
  bool forceOldGetUp;
  std::string hello;
};

class GetUpEngine  {
  public:
    GetUpEngineParameters p;

    GetUpEngine();
    void parseCfgFile();
    void generateMofOfMotion(Mof::Motion motionName);
    void generateLuaMof(Mof::Motion motionName);
    void printme();
};



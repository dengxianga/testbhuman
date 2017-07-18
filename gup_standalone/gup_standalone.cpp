#include <string>
#include <functional>
#include <vector>
#include "gup_standalone.h"
#include <iostream>
#include "BHAssert.h"
#include "InStreams.h"
#include <fstream>
#include <Angle.h>
#include <Pose2f.h>
#include <Eigen/Dense>

void GetUpEngine::printme() {
  std::cout << "here 1" <<std::endl;
}

float extract_num(char* line) {
  double val;
  sscanf(line, "%*[^-0-9]%lf", &val);
  return (float) val;
}

Joints getJoint(std::string joint_string) {
  if (joint_string.compare("headYaw;") == 0) {
    return Joints::headYaw;
  } else if (joint_string.compare("headPitch;") == 0) {
    return Joints::headPitch;
  } else if (joint_string.compare("lShoulderPitch;") == 0) {
    return Joints::lShoulderPitch;
  } else if (joint_string.compare("lShoulderRoll;") == 0) {
    return Joints::lShoulderRoll;
  } else if (joint_string.compare("lElbowYaw;") == 0) {
    return Joints::lElbowYaw;
  } else if (joint_string.compare("lElbowRoll;") == 0) {
    return Joints::lElbowRoll;
  } else if (joint_string.compare("lWristYaw;") == 0) {
    return Joints::lWristYaw;
  } else if (joint_string.compare("lHand;") == 0) {
    return Joints::lHand;
  } else if (joint_string.compare("rShoulderPitch;") == 0) {
    return Joints::rShoulderPitch;
  } else if (joint_string.compare("rShoulderRoll;") == 0) {
    return Joints::rShoulderRoll;
  } else if (joint_string.compare("rElbowYaw;") == 0) {
    return Joints::rElbowYaw;
  } else if (joint_string.compare("rElbowRoll;") == 0) {
    return Joints::rElbowRoll;
  } else if (joint_string.compare("rWristYaw;") == 0) {
    return Joints::rWristYaw;
  } else if (joint_string.compare("rHand;") == 0) {
    return Joints::rHand;  
  } else if (joint_string.compare("lHipYawPitch;") == 0) {
    return Joints::lHipYawPitch;
  } else if (joint_string.compare("lHipPitch;") == 0) {
    return Joints::lHipPitch;
  } else if (joint_string.compare("lHipRoll;") == 0) {
    return Joints::lHipRoll;
  } else if (joint_string.compare("lKneePitch;") == 0) {
    return Joints::lKneePitch;
  } else if(joint_string.compare("lAnklePitch;") == 0) {
    return Joints::lAnklePitch;
  } else if(joint_string.compare("lAnkleRoll;") == 0) {
    return Joints::lAnkleRoll;
  } else if (joint_string.compare("rHipYawPitch;") == 0) {
    return Joints::rHipYawPitch;
  } else if (joint_string.compare("rHipPitch;") == 0) {
    return Joints::rHipPitch;
  } else if (joint_string.compare("rHipRoll;") == 0) {
    return Joints::rHipRoll;
  } else if (joint_string.compare("rKneePitch;") == 0) {
    return Joints::rKneePitch;
  } else if(joint_string.compare("rAnklePitch;") == 0) {
    return Joints::rAnklePitch;
  } else if(joint_string.compare("rAnkleRoll;") == 0) {
    return Joints::rAnkleRoll;
  } else {
    std::cout << joint_string << std::endl;
    std::cout << "we have a problem translating joint names!" << std::endl;
  }

  return Joints::numOfJoints;
  
}

void GetUpEngine::parseCfgFile() {
  std::ifstream config_file("../getUpEngine.cfg");

  if (!config_file) {
    std::cout<<"Cannot open file" <<std::endl;
    return;
  }

  // position variables
  int mof_num = -1;
  Mof mof_names[Mof::Motion::numOfMotions];
  mof_names[0] = Mof::front;
  mof_names[1] = Mof::frontFast;
  mof_names[2] = Mof::back;
  mof_names[3] = Mof::backFast;
  mof_names[4] = Mof::recovering;
  mof_names[5] = Mof::recoverFromSide;
  mof_names[6] = Mof::stand;
  mof_names[7] = Mof::fromSumo;
  mof_names[8] = Mof::fromSumoKeeper;
  mof_names[9] = Mof::fromSafeFall;

  // p.hello = "hello";
  // std::cout << p.hello << std::endl;

  char holder[255] = "";

  config_file.getline(holder, 255); // maxNumOfUnsuccessfulTries
  p.maxNumOfUnsuccessfulTries = (int) extract_num(holder);

  int count = 0;
  // iterate through Mofs
  while (strncmp("forceOldGetUp = ", holder, 15) && !std::string(holder).empty()) {
    config_file.getline(holder, 255);
    // check for beginning of next mof
    if (strcmp("  {", holder) == 0) {
      mof_num++;
      // Mof this_mof = p.mofs[mof_names[mof_num]];
      p.mofs[mof_num].name = mof_names[mof_num];
      config_file.getline(holder, 255); // name = 
      // iterate through single mof
      while ((strcmp("  },", holder) && strcmp("  }", holder)) 
              && !std::string(holder).empty()) { // is this mof finished?
        config_file.getline(holder, 255); // baseLimbStiffness, lines, balanceStartLine or OdometryOffset

        if (strcmp("    baseLimbStiffness = [", holder) == 0) {
          for (int i = 0; i < 5; i++) {
            config_file.getline(holder, 255);
            p.mofs[mof_num].baseLimbStiffness[i] = (int) extract_num(holder);
          }
          config_file.getline(holder, 255); // ];

        } else if (strcmp("    lines = [", holder) == 0) {
          config_file.getline(holder, 255); // {

          while (strcmp("    ];", holder)) {
            MofLine new_mofline;
            config_file.getline(holder, 255); // head = [
            for (int i = 0; i < 2; i++) {
              config_file.getline(holder, 255);
              new_mofline.head[i] =  extract_num(holder);
            }

            config_file.getline(holder, 255); // ];
            config_file.getline(holder, 255); // leftArm = [
            for (int i = 0; i < 6; i++) {
              config_file.getline(holder, 255);
              new_mofline.leftArm[i] =  extract_num(holder);
            }

            config_file.getline(holder, 255); // ];
            config_file.getline(holder, 255); // rightArm = [
            for (int i = 0; i < 6; i++) {
              config_file.getline(holder, 255);
              new_mofline.rightArm[i] =  extract_num(holder);
            }

            config_file.getline(holder, 255); // ];
            config_file.getline(holder, 255); // leftLeg = [
            for (int i = 0; i < 6; i++) {
              config_file.getline(holder, 255);
              new_mofline.leftLeg[i] =  extract_num(holder);
            }

            config_file.getline(holder, 255); // ];
            config_file.getline(holder, 255); // rightLeg = [
            for (int i = 0; i < 6; i++) {
              config_file.getline(holder, 255);
              new_mofline.rightLeg[i] =  extract_num(holder);
            }

            config_file.getline(holder, 255); // ];
            config_file.getline(holder, 255); // singleMotorStiffnessChange

            if (strcmp("        singleMotorStiffnessChange = [", holder) == 0) {
                while (strcmp("        ];", holder)) {
                config_file.getline(holder, 255);
                if (strncmp("            joint = ", holder, 20) == 0) {
                  std::string holder_string(holder);
                  std::string joint_string = holder_string.substr(20);
                  Joints joint = getJoint(joint_string);

                  config_file.getline(holder, 255);
                  int value = (int) extract_num(holder);
                  new_mofline.singleMotorStiffnessChange.push_back(StiffnessPair(joint, value));
                }
              } 
            } else if (strcmp("        singleMotorStiffnessChange = [];", holder) == 0) {
              // no single motor stiffness change specification
            } else {
              std::cout << "oh no! we've got a problem with single motor stiffness change " << mof_num << std::endl;
            } 

            config_file.getline(holder, 255); // duration
            new_mofline.duration = extract_num(holder);

            config_file.getline(holder, 255); // critical
            std::string holder_string(holder);
            if ((holder_string.substr(19)).compare("false;") == 0) {
              new_mofline.critical = false;
            } else if ((holder_string.substr(19)).compare("true;") ==0) {
              new_mofline.critical = true;
            } else {
              std::cout << "oh no! we've got a problem with critical, line " << mof_num << std::endl;
            }
      
            config_file.getline(holder, 255); // },
            p.mofs[mof_num].lines.push_back(new_mofline);
            config_file.getline(holder, 255); // next, could be { or ];,
          }

        } else if (strncmp("    balanceStartLine = ", holder, 15) == 0) {
          p.mofs[mof_num].balanceStartLine = (int) extract_num(holder);
        } else if (strcmp("    odometryOffset = {", holder) == 0) {
          // Pose2f odo;

          config_file.getline(holder, 255); // rotation = angledeg
          float angle = extract_num(holder);
          // Angle rotation(angle);

          config_file.getline(holder, 255); // translation = {
          config_file.getline(holder, 255); // x
          float x = extract_num(holder);
          config_file.getline(holder, 255); // y
          float y = extract_num(holder);
          // Vector2f translation(x, y);

          // Pose2f pose(rotation, translation);
          Odometry pose(x, y, angle);
          p.mofs[mof_num].odometryOffset = pose;
          config_file.getline(holder, 255); // }; ending translation
          config_file.getline(holder, 255); // }; ending odometryOffset
          config_file.getline(holder, 255); // }; ending mof
        } else {
          std::cout << "none of the above" << std::endl;
          std::cout << "holder: " << holder << std::endl;
        }
      } 
    }
  } 

  std::string holder_string(holder);
  if ((holder_string.substr(16)).compare("false;")) {
    p.forceOldGetUp = false;
  } else if ((holder_string.substr(16)).compare("true;")) {
    p.forceOldGetUp = true;
  } else {
    std::cout << "we've got another option for forceOldGetUp: " << holder << std::endl;
  }

  config_file.close();
  std::cout << "Done parsing" << std::endl;
}

GetUpEngine::GetUpEngine() {
  parseCfgFile();
}

void GetUpEngine::generateMofOfMotion(Mof::Motion motionName)
{
  int motion = -1;
  for(int i = 0; i < Mof::numOfMotions; ++i) {
    if(p.mofs[i].name == motionName)
    {
      motion = i;
      break;
    }
  }
  if(motion < 0)
    return;

  char name[512];
  sprintf(name, "/home/nao/gu_test2/getUpEngineDummy.mof");
  FILE* f = fopen(name, "w"); //for simplicity discard the whole content
  if(!f)
    std::cout << "GetUpEngineDummy.mof was not found" << std::endl;
  else
  {
    fputs("\" IF YOU HAVE CHANGED THIS FILE DO NOT COMMIT IT!\n", f);
    fputs("motion_id = getUpEngineDummy\n", f);
    fputs("label start\n", f);

    // keep trac of stiffness changes, init with base stiffness
    int internalStiffness[Joints::numOfJoints];

    auto putStiffness = [&](FILE * f)
    {
      char stiffness[250] = "\nstiffness";
      for(int i = 0; i < Joints::numOfJoints; ++i)
      {
        strcat(stiffness, " ");
        strcat(stiffness, std::to_string(internalStiffness[i]).c_str());
      }
      strcat(stiffness, " 0\n\n");
      fputs(stiffness, f);
    };

    for(int i = 0; i < 2; ++i)
      internalStiffness[i] = p.mofs[motion].baseLimbStiffness[0];
    //set arm joints
    for(int i = 0; i < 6; ++i)
    {
      internalStiffness[Joints::lShoulderPitch + i] = p.mofs[motion].baseLimbStiffness[1];
      internalStiffness[Joints::rShoulderPitch + i] = p.mofs[motion].baseLimbStiffness[2];
      internalStiffness[Joints::lHipYawPitch + i] = p.mofs[motion].baseLimbStiffness[3];
      internalStiffness[Joints::rHipYawPitch + i] = p.mofs[motion].baseLimbStiffness[4];
    }

    putStiffness(f);

    std::cout << "lines size" << p.mofs[motion].lines.size() <<std::endl;

    //add lines now
    int lastLine = (int) p.mofs[motion].lines.size() - 1;
    for(int i = 0; i < (int) p.mofs[motion].lines.size(); i++)
    {
      if(!p.mofs[motion].lines[i].singleMotorStiffnessChange.empty())
      {
        for(unsigned j = 0; j < p.mofs[motion].lines[i].singleMotorStiffnessChange.size(); ++j)
          internalStiffness[p.mofs[motion].lines[i].singleMotorStiffnessChange[j].joint] = p.mofs[motion].lines[i].singleMotorStiffnessChange[j].s;

        putStiffness(f);
      }

      if(p.mofs[motion].lines[i].critical)
        fputs("\n\"@critical (upright check at start of interpolation to next line)\n\n", f);

      if(i == p.mofs[motion].balanceStartLine - 1) //add it before the start line
        fputs("\n\"@balanceStartLine (turn on balance at start of interpolation to next line)\n\n", f);

      //add label repeat before last line
      if(i == lastLine)
        fputs("label repeat\n", f);

      char line[250];
      sprintf(line, "%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f 1 %d\n",
              p.mofs[motion].lines[i].head[0], p.mofs[motion].lines[i].head[1],
              p.mofs[motion].lines[i].leftArm[0], p.mofs[motion].lines[i].leftArm[1], p.mofs[motion].lines[i].leftArm[2],
              p.mofs[motion].lines[i].leftArm[3], p.mofs[motion].lines[i].leftArm[4], p.mofs[motion].lines[i].leftArm[5],
              p.mofs[motion].lines[i].rightArm[0], p.mofs[motion].lines[i].rightArm[1], p.mofs[motion].lines[i].rightArm[2],
              p.mofs[motion].lines[i].rightArm[3], p.mofs[motion].lines[i].rightArm[4], p.mofs[motion].lines[i].rightArm[5],
              p.mofs[motion].lines[i].leftLeg[0], p.mofs[motion].lines[i].leftLeg[1], p.mofs[motion].lines[i].leftLeg[2],
              p.mofs[motion].lines[i].leftLeg[3], p.mofs[motion].lines[i].leftLeg[4], p.mofs[motion].lines[i].leftLeg[5],
              p.mofs[motion].lines[i].rightLeg[0], p.mofs[motion].lines[i].rightLeg[1], p.mofs[motion].lines[i].rightLeg[2],
              p.mofs[motion].lines[i].rightLeg[3], p.mofs[motion].lines[i].rightLeg[4], p.mofs[motion].lines[i].rightLeg[5],
              (int) p.mofs[motion].lines[i].duration);
      fputs(line, f);
    }
    //add odometry offset as comment
    char odometry[100];
    // sprintf(odometry, "\n\"@odometryOffset %.2f %.2f %.2f\n", p.mofs[motion].odometryOffset.translation.x(), p.mofs[motion].odometryOffset.translation.y(), p.mofs[motion].odometryOffset.rotation.toDegrees());
    sprintf(odometry, "\n\"@odometryOffset %.2f %.2f %.2f\n", p.mofs[motion].odometryOffset.x, p.mofs[motion].odometryOffset.y, p.mofs[motion].odometryOffset.angle);
    fputs(odometry, f);
    //add footer
    fputs("\ntransition getUpEngineDummy getUpEngineDummy repeat\n", f);
    fputs("transition allMotions extern start\n", f);
    fclose(f);
  }
}

int luaToBHuman(int lua_index) {
  int converter[22];
  converter[0] = Joints::headYaw;
  converter[1] = Joints::headPitch;
  converter[2] = Joints::lShoulderPitch;
  converter[3] = Joints::lShoulderRoll;
  converter[4] = Joints::lElbowYaw;
  converter[5] = Joints::lElbowRoll;
  converter[6] = Joints::lHipYawPitch;
  converter[7] = Joints::lHipRoll;
  converter[8] = Joints::lHipPitch;
  converter[9] = Joints::lKneePitch;
  converter[10] = Joints::lAnklePitch;
  converter[11] = Joints::lAnkleRoll;
  converter[12] = Joints::rHipYawPitch;
  converter[13] = Joints::rHipRoll;
  converter[14] = Joints::rHipPitch;
  converter[15] = Joints::rKneePitch;
  converter[16] = Joints::rAnklePitch;
  converter[17] = Joints::rAnkleRoll;
  converter[18] = Joints::rShoulderPitch;
  converter[19] = Joints::rShoulderRoll;
  converter[20] = Joints::rElbowYaw;
  converter[21] = Joints::rElbowRoll;

  return converter[lua_index];
}

enum luaJointNames {
HeadYaw = 0,
HeadPitch,
LShoulderPitch, 
LShoulderRoll,
LElbowYaw, 
LElbowRoll,
LHipYawPitch, 
LHipRoll,
LHipPitch,
LKneePitch, 
LAnklePitch, 
LAnkleRoll,
RHipYawPitch, 
RHipRoll, 
RHipPitch,
RKneePitch, 
RAnklePitch, 
RAnkleRoll,
RShoulderPitch, 
RShoulderRoll,
RElbowYaw, 
RElbowRoll, 
none,
numOfLuaJoints = none};


int BHumanToLua(int bh_index) {
  int converter[26];
  converter[0] = HeadYaw;
  converter[1] = HeadPitch;
  converter[2] = LShoulderPitch;
  converter[3] = LShoulderRoll;
  converter[4] = LElbowYaw;
  converter[5] = LElbowRoll;
  converter[6] = none;
  converter[7] = none;
  converter[8] = RShoulderPitch;
  converter[9] = RShoulderRoll;
  converter[10] = RElbowYaw;
  converter[11] = RElbowRoll;
  converter[12] = none;
  converter[13] = none;
  converter[14] = LHipYawPitch;
  converter[15] = LHipRoll;
  converter[16] = LHipPitch;
  converter[17] = LKneePitch;
  converter[18] = LAnklePitch;
  converter[19] = LAnkleRoll;
  converter[20] = RHipYawPitch;
  converter[21] = RHipRoll;
  converter[22] = RHipPitch;
  converter[23] = RKneePitch;
  converter[24] = RAnklePitch;
  converter[25] = RAnkleRoll;

  return converter[bh_index];
}


void GetUpEngine::generateLuaMof(Mof::Motion motionName) {
  int motion = -1;
  for(int i = 0; i < Mof::numOfMotions; ++i) {
    if(p.mofs[i].name == motionName)
    {
      motion = i;
      break;
    }
  }
  if(motion < 0)
    return;

  char name[512];
  sprintf(name, "bh_getup.lua");
  FILE* f = fopen(name, "w"); //for simplicity discard the whole content
  if(!f)
    std::cout << "could not open getUpEngineFrontMof.lua" << std::endl;
  else {
    fputs("local mot = {}; \n", f);
    fputs("mot.servos = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22};\n", f);
    fputs("mot.keyframes = { \n", f);

    // keep track of stiffness changes, init with base stiffness
    int internalStiffness[Joints::numOfJoints];

    // set head joints
    for(int i = 0; i < 2; ++i)
      internalStiffness[i] = p.mofs[motion].baseLimbStiffness[0];
    // set arm joints
    for(int i = 0; i < 6; ++i){
      internalStiffness[Joints::lShoulderPitch + i] = p.mofs[motion].baseLimbStiffness[1];
      internalStiffness[Joints::rShoulderPitch + i] = p.mofs[motion].baseLimbStiffness[2];
      internalStiffness[Joints::lHipYawPitch + i] = p.mofs[motion].baseLimbStiffness[3];
      internalStiffness[Joints::rHipYawPitch + i] = p.mofs[motion].baseLimbStiffness[4];
    }

    // get stiffnesses for lua
    float jointStiffness[numOfLuaJoints];
    for (int i = 0; i < numOfLuaJoints; i++) {
      jointStiffness[i] = (float) internalStiffness[luaToBHuman(i)] / 100;
    }

    for (int i = 0; i < p.mofs[motion].lines.size(); i++) {
      fputs("  {\n", f);

      char angle[255] = "    angles = vector.new({";
      for (int j = 0; j < 2; j++) { // head
        char angle_char[10];
        sprintf(angle_char, "%.3f, ", (double) p.mofs[motion].lines[i].head[j]);
        strcat(angle, angle_char);
      } 
      for (int j = 0; j < 4; j++) { // left arm 
        char angle_char[10];
        sprintf(angle_char, "%.3f, ", (double) p.mofs[motion].lines[i].leftArm[j]);
        strcat(angle, angle_char);
      } 
      for (int j = 0; j < 6; j++) { // left leg
        char angle_char[10];
        sprintf(angle_char, "%.3f, ", (double) p.mofs[motion].lines[i].leftLeg[j]);
        strcat(angle, angle_char);
      } 
      for (int j = 0; j < 6; j++) { // right leg
        char angle_char[10];
        sprintf(angle_char, "%.3f, ", (double) p.mofs[motion].lines[i].rightLeg[j]);
        strcat(angle, angle_char);
      } 
      for (int j = 0; j < 4; j++) { // right arm
        char angle_char[10];
        sprintf(angle_char, "%.3f, ", (double) p.mofs[motion].lines[i].rightArm[j]);
        strcat(angle, angle_char);
      } 
      strcat(angle, "}) * math.pi / 180,\n");
      fputs(angle, f);

      if (!p.mofs[motion].lines[i].singleMotorStiffnessChange.empty()) {
        for (int j = 0; j < p.mofs[motion].lines[i].singleMotorStiffnessChange.size(); j++) {
          int this_joint = p.mofs[motion].lines[i].singleMotorStiffnessChange[j].joint;
          int new_value = p.mofs[motion].lines[i].singleMotorStiffnessChange[j].s;
          jointStiffness[BHumanToLua(this_joint)] = (float) new_value/100;
        }
      }

      char stiffness[255] = "    stiffnesses = {";
      for (int j = 0; j < numOfLuaJoints; j++) {
        char stiff_char[10];
        sprintf(stiff_char, "%.3f, ", (double) jointStiffness[j]);
        strcat(stiffness, stiff_char);
      } 
      strcat(stiffness, "},\n");
      fputs(stiffness, f);

      float dur = (float) p.mofs[motion].lines[i].duration / 1000;
      char duration[50];
      sprintf(duration, "    duration = %.3f,\n", dur);
      fputs(duration, f);

      char critical[20];
      sprintf(critical, "    critical = %s\n;", p.mofs[motion].lines[i].critical ? "true" : "false");
      fputs(critical, f);

      fputs("  },\n", f);
    }

    fputs("};\n", f);
    fputs("return mot;\n", f);
  }
}
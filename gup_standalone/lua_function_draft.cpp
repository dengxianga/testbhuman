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
  sprintf(name, "getUpEngineFrontMof.lua");
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
    int jointStiffness[numOfLuaJoints];
    for (int i = 0; i < numOfLuaJoints; i++) {
      jointStiffness[i] = internalStiffness[luaToBHuman(i)];
    }

    for (int i = 0; i < p.mofs[motion].lines.size(); i++) {
      fputs("  {\n", f);

      char angle[255] = "    angle = ({";
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
          jointStiffness[BHumanToLua(this_joint)] = new_value;
        }
      }

      char stiffness[255] = "    stiffness = {";
      for (int j = 0; j < numOfLuaJoints; j++) {
        char stiff_char[10];
        sprintf(stiff_char, "%.3f, ", (double) jointStiffness[j]);
        strcat(stiffness, stiff_char);
      } 
      strcat(stiffness, "},\n");
      fputs(stiffness, f);

      char duration[50];
      sprintf(duration, "    duration = %.3f;\n", p.mofs[motion].lines[i].duration);
      fputs(duration, f);

      fputs("  },\n", f);
    }

    fputs("};\n", f);
    fputs("return mot;\n", f);
  }
}
local mot = {}; 
mot.servos = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22};
mot.keyframes = { 
  {
    angles = vector.new({0.000, 30.000, 90.000, 90.000, 0.000, 0.000, 0.000, -0.000, 0.000, 0.000, 0.000, -0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 90.000, -90.000, -0.000, -0.000, }) * math.pi / 180,
    stiffnesses = {0.300, 0.300, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, },
    duration = 1.000;
  },
};
return mot;
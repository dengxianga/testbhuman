local mot = {}; 
mot.servos = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22};
mot.keyframes = { 
  {
    angles = vector.new({7.500, 30.000, 119.800, 7.800, -71.100, -31.400, -67.900, 18.600, -91.600, 75.500, 51.600, -4.800, -67.900, -20.700, -91.000, 73.000, 52.200, 7.600, 121.600, -0.400, 87.500, 22.200, }) * math.pi / 180,
    stiffnesses = {0.300, 0.300, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, },
    duration = 0.250;
  },
  {
    angles = vector.new({6.900, 30.000, 119.900, -4.800, -85.300, -18.600, -65.000, 41.700, -80.200, 76.500, 49.300, -7.200, -65.000, -32.500, -84.000, 81.800, 47.100, 1.100, 121.600, -0.900, 87.400, 22.400, }) * math.pi / 180,
    stiffnesses = {0.300, 0.300, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, },
    duration = 0.300;
  },
  {
    angles = vector.new({5.500, 29.400, 115.800, -7.600, -84.600, -15.900, -68.900, 11.700, -29.700, 124.600, -40.900, -18.900, -68.900, -20.300, -89.400, 48.200, 52.600, -1.000, 120.700, -39.700, 99.400, 8.200, }) * math.pi / 180,
    stiffnesses = {0.300, 0.300, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, },
    duration = 0.500;
  },
  {
    angles = vector.new({5.500, 29.400, 115.800, -7.600, -84.600, -15.900, -68.900, 11.700, -29.700, 124.600, -40.900, -18.900, -68.900, -20.300, -89.400, 48.200, 52.600, -1.000, 120.700, -39.700, 99.400, 8.200, }) * math.pi / 180,
    stiffnesses = {0.300, 0.300, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, },
    duration = 0.100;
  },
  {
    angles = vector.new({4.800, 29.600, 101.500, 0.900, -79.200, -1.100, -65.600, 3.800, -42.700, 123.900, -36.500, -4.100, -65.600, -29.800, -93.000, 121.800, -7.900, 8.300, 104.900, -17.400, 102.000, 15.100, }) * math.pi / 180,
    stiffnesses = {0.300, 0.300, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, },
    duration = 0.250;
  },
  {
    angles = vector.new({1.230, 29.620, 96.500, 0.260, -80.160, -1.760, -0.540, -6.500, -54.920, 122.780, -68.910, 4.480, -0.540, -8.260, -55.360, 122.610, -69.430, 7.120, 88.160, -2.200, 102.220, 8.530, }) * math.pi / 180,
    stiffnesses = {0.300, 0.300, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, },
    duration = 0.200;
  },
  {
    angles = vector.new({0.000, 30.000, 90.000, 11.500, -90.000, -11.500, 0.000, -0.000, -45.000, 71.400, -35.100, -0.000, 0.000, 0.000, -45.000, 71.400, -35.100, 0.000, 90.000, -11.500, 90.000, 11.500, }) * math.pi / 180,
    stiffnesses = {0.300, 0.300, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, },
    duration = 0.500;
  },
  {
    angles = vector.new({0.000, 30.000, 90.000, 11.500, -90.000, -11.500, 0.000, -0.000, -45.000, 71.400, -35.100, -0.000, 0.000, 0.000, -45.000, 71.400, -35.100, 0.000, 90.000, -11.500, 90.000, 11.500, }) * math.pi / 180,
    stiffnesses = {0.300, 0.300, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, 0.900, },
    duration = 0.100;
  },
};
return mot;
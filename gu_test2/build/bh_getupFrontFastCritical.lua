local mot = {}; 
mot.servos = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22};
mot.keyframes = { 
  {
    angles = vector.new({-0.500, 2.300, 90.000, 10.100, -90.000, -8.200, 0.000, 0.000, -21.300, 48.900, -27.600, 0.000, 0.000, 0.000, -21.200, 49.000, -27.600, 0.000, 90.500, -9.800, 89.800, 8.400, }) * math.pi / 180,
    stiffnesses = {1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, },
    duration = 0.300;
    critical = false
  },
  {
    angles = vector.new({0.000, 24.200, 68.500, 71.900, -121.900, -4.600, -3.900, -0.200, -84.600, 123.700, -52.900, 2.300, -3.900, -0.700, -80.200, 122.000, -47.900, 2.200, 84.600, -72.900, 120.400, 6.400, }) * math.pi / 180,
    stiffnesses = {1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, },
    duration = 0.200;
    critical = false
  },
  {
    angles = vector.new({0.000, 24.200, -13.900, 68.200, -122.200, -11.100, 0.300, -1.600, -57.600, 41.700, -46.000, 4.500, 0.300, -0.600, -57.200, 42.700, -43.700, 0.300, -13.500, -67.900, 121.900, 8.000, }) * math.pi / 180,
    stiffnesses = {1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, },
    duration = 0.200;
    critical = false
  },
  {
    angles = vector.new({0.000, 29.620, -39.640, -3.870, -90.620, -88.070, -28.400, -17.520, -92.590, -7.310, 3.540, -20.800, -28.400, 17.610, -93.290, -7.220, 3.900, 20.010, -49.390, 16.610, 103.360, 80.510, }) * math.pi / 180,
    stiffnesses = {1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, },
    duration = 0.200;
    critical = false
  },
  {
    angles = vector.new({0.000, -29.880, 17.660, 0.700, -123.670, -0.960, -54.240, -19.800, -87.050, 97.720, -23.530, -4.630, -54.240, 19.370, -90.830, 82.780, -4.630, 6.920, 15.470, 5.360, 117.330, 1.500, }) * math.pi / 180,
    stiffnesses = {0.250, 0.250, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, },
    duration = 0.200;
    critical = false
  },
  {
    angles = vector.new({-0.002, -44.300, 40.713, -18.108, -118.305, -12.215, -67.750, -7.590, -93.652, 124.518, -8.024, 3.131, -67.750, 20.762, -95.616, 122.088, -5.558, -16.883, 40.048, 12.566, 119.003, 17.581, }) * math.pi / 180,
    stiffnesses = {0.250, 0.250, 0.400, 0.400, 0.400, 0.400, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 0.400, 0.400, 0.400, 0.400, },
    duration = 0.300;
    critical = false
  },
  {
    angles = vector.new({-3.342, -37.708, 75.870, -15.735, -94.925, -88.856, -54.654, 15.789, -34.677, 124.606, -58.826, -2.758, -54.654, -6.397, -38.750, 122.263, -52.229, -3.612, 70.019, 15.730, 82.967, 83.939, }) * math.pi / 180,
    stiffnesses = {0.250, 0.250, 0.400, 0.400, 0.400, 0.400, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 0.400, 0.400, 0.400, 0.400, },
    duration = 0.300;
    critical = false
  },
  {
    angles = vector.new({-2.639, -39.290, 75.694, -1.672, -98.617, -76.200, -10.269, -5.305, -49.618, 124.606, -67.966, 5.768, -10.269, 7.666, -47.891, 122.175, -67.873, -5.809, 90.322, 13.006, 89.911, 81.742, }) * math.pi / 180,
    stiffnesses = {0.250, 0.250, 0.400, 0.400, 0.400, 0.400, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 0.400, 0.400, 0.400, 0.400, },
    duration = 0.300;
    critical = true
  },
  {
    angles = vector.new({-0.500, 2.300, 90.000, 10.100, -90.000, -8.200, 0.000, 0.000, -21.300, 48.900, -27.600, 0.000, 0.000, 0.000, -21.200, 49.000, -27.600, 0.000, 90.500, -9.800, 89.800, 8.400, }) * math.pi / 180,
    stiffnesses = {0.250, 0.250, 0.400, 0.400, 0.400, 0.400, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 0.400, 0.400, 0.400, 0.400, },
    duration = 0.500;
    critical = false
  },
  {
    angles = vector.new({-0.500, 2.300, 90.000, 10.100, -90.000, -8.200, 0.000, 0.000, -21.300, 48.900, -27.600, 0.000, 0.000, 0.000, -21.200, 49.000, -27.600, 0.000, 90.500, -9.800, 89.800, 8.400, }) * math.pi / 180,
    stiffnesses = {0.250, 0.250, 0.400, 0.400, 0.400, 0.400, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 0.400, 0.400, 0.400, 0.400, },
    duration = 0.100;
    critical = false
  },
};
return mot;

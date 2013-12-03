% Recover data with images 335 & 386 from my studio

%  %  Generated running:
%  %  
%  %  ./test_pose paired/studio_morning_27_Nov_02_bgr_335.png paired/studio_morning_27_Nov_02_bgr_386.png 
%  %  			paired/studio_morning_27_Nov_02_depth_335.png paired/studio_morning_27_Nov_02_depth_386.png
%  %  
%  %  num_goodmatch = 30; //30 Original Number
%  %  Output of pts:
%  %  pts1_studio335.txt & pst2_studio386.txt
%  %  25 matches, 3 outliers
%  %  	Outliers (technically the last ones):
%  %  		Point of image1, 21: [287.391, 198.774] ||      Point of image2, 21: [112.344, 193.34]
%  %  		Point of image1, 22: [419.878, 225.63]  ||      Point of image2, 22: [57.1009, 82.3134]
%  %  		Point of image1, 24: [287.462, 225.062] ||      Point of image2, 24: [112.344, 193.34]
%  %  
%  %  
%  %  
%  %  Matrices output:
%  %  k = [[520.28075824744883 0.0 328.59418282611989]; [ 0.0 517.35099060486289 265.23529733631955]; [0.0 0.0 1.0]];
%  %  
%  %  e = [-1.080266980032385, -12.15807854842378, -3.340389244757281;
%  %  13.17110603224156, 0.02773484671219164, 1.021160443209442;
%  %  3.104756593744098, -5.030586896902129, -1.108772595488977];
%  %  
f = [-3.990759649509577e-06, -4.516914764318681e-05, 0.006871433719111364;
4.893270187594865e-05, 1.036227705834435e-07, -0.01413266036273986;
-0.005699875519757897, 0.005091094016551168, 1];

%% Essential matrix with R negative determinant
F1=[9.229412839299126e-07, -1.442776056021839e-05, 0.004504506417229076;
  1.506212069190375e-05, -7.471502128165758e-08, -0.005461405788963258;
  -0.005383103612567212, 0.002863767883339863, 1]
E=[1.452534250546615, -22.57869077467164, 2.967536248192763;
  23.57143120679362, -0.1162666808393443, -1.599889924708;
  -3.281511990461602, -5.705750376225152, 1];
% ** ALTERNATIVE COMMAND
% ./test_pose paired/studio_library2_bgr_94.png paired/studio_library2_bgr_112.png 
% paired/studio_library2_depth_94.png paired/studio_library2_depth_112.png

%% Time solving Features:508256 us with FUNCTION solveFeaturesEuclidean (without drawing or printing debug data)

%% ***IF I correct the Rotation matrix (when its det = -1 from E) then the z angle is close to 180°
%% ***IF the Rotation matrix is ok from E (when its det = 1) then the z angle is correct.

% ******************** 20pts pose (all inliers) wihtout removing points having a bad depth value *******************

%  F keeping 20 / 20
%  20 matches before, 20 new matches after Fundamental Matrix
%  Fundamental Matrix 
F = [1.482615958408929e-06, -1.417615789293155e-05, 0.005117449824506071;
  1.282640577657642e-05, -3.546491354549262e-08, -0.007114895274185629;
  -0.005822345474973352, 0.005335976647381335, 1];
% Calibration matrix
k = [[520.28075824744883 0.0 328.59418282611989]; [ 0.0 517.35099060486289 265.23529733631955]; [0.0 0.0 1.0]];

x1data = [[388.608, 329.05];
[578.698, 308.094];
[578.698, 308.094];
[561.368, 222.685];
[519.147, 303.097];
[499.424, 408.245];
[519.147, 303.097];
[499.385, 408.772];
[356.031, 186.507];
[287.666, 187.917];
[306.227, 336.518];
[297.282, 443.226];
[494.266, 215.774];
[269.029, 408.47];
[407.811, 416.36];
[403.844, 414.919];
[498.445, 344.934];
[501.627, 202.419];
[403.844, 414.919];
[498.445, 344.934]];
  
x2data= [[223.567, 337.623];
[393.846, 310.495];
[393.846, 310.495];
[379.717, 234.095];
[345.096, 307.384];
[328.428, 405.965];
[345.096, 307.384];
[328.428, 405.965];
[189.105, 193.84];
[112.344, 193.34];
[134, 351.883];
[123.407, 465.612];
[323.954, 227.192];
[90.5995, 431.772];
[241.977, 423.216];
[237.767, 422.282];
[326.284, 346.947];
[329.479, 214.608];
[237.767, 422.282];
[326.284, 346.947]];

%  Point of image1, 0: [388.608, 329.05]   ||      Point of image2, 0: [223.567, 337.623]
%  Point of image1, 1: [578.698, 308.094]  ||      Point of image2, 1: [393.846, 310.495]
%  Point of image1, 2: [578.698, 308.094]  ||      Point of image2, 2: [393.846, 310.495]
%  Point of image1, 3: [561.368, 222.685]  ||      Point of image2, 3: [379.717, 234.095]
%  Point of image1, 4: [519.147, 303.097]  ||      Point of image2, 4: [345.096, 307.384]
%  Point of image1, 5: [499.424, 408.245]  ||      Point of image2, 5: [328.428, 405.965]
%  Point of image1, 6: [519.147, 303.097]  ||      Point of image2, 6: [345.096, 307.384]
%  Point of image1, 7: [499.385, 408.772]  ||      Point of image2, 7: [328.428, 405.965]
%  Point of image1, 8: [356.031, 186.507]  ||      Point of image2, 8: [189.105, 193.84]
%  Point of image1, 9: [287.666, 187.917]  ||      Point of image2, 9: [112.344, 193.34]
%  Point of image1, 10: [306.227, 336.518] ||      Point of image2, 10: [134, 351.883]
%  Point of image1, 11: [297.282, 443.226] ||      Point of image2, 11: [123.407, 465.612]
%  Point of image1, 12: [494.266, 215.774] ||      Point of image2, 12: [323.954, 227.192]
%  Point of image1, 13: [269.029, 408.47]  ||      Point of image2, 13: [90.5995, 431.772]
%  Point of image1, 14: [407.811, 416.36]  ||      Point of image2, 14: [241.977, 423.216]
%  Point of image1, 15: [403.844, 414.919] ||      Point of image2, 15: [237.767, 422.282]
%  Point of image1, 16: [498.445, 344.934] ||      Point of image2, 16: [326.284, 346.947]
%  Point of image1, 17: [501.627, 202.419] ||      Point of image2, 17: [329.479, 214.608]
%  Point of image1, 18: [403.844, 414.919] ||      Point of image2, 18: [237.767, 422.282]
%  Point of image1, 19: [498.445, 344.934] ||      Point of image2, 19: [326.284, 346.947]

%  Essential Matrix:
E = [1.19268358093512, -11.3397278660789, 2.852092996587438;
  10.26004028064625, -0.02820918354669254, -4.473449347835507;
  -2.988990756136342, 1.027593570598862, 1];
%  Rotation Matrix Rot1(W):
R1 = [0.947803  0.0077022  -0.318764;
-0.00495423   0.999943 0.00943058;
  0.318818 -0.0073591   0.947787];
%  Rotation Matrix Rot2(WT):
R2 = [-0.881184 0.0376624  0.471271;
 0.213043 -0.858245  0.466936;
 0.422052  0.511857  0.748247];
%  Rotation Value (W):
anglesR1 = [-0.57008; -18.5882; -0.465597];
%  Rotation Value (WT):
anglesR2 = [-31.9658; 28.1169; -177.553];
%  Translation Value (+u): 
t1 = [0.08624772001349551; 0.2693995233777621; 0.9591586039838815];
%  Translation Value (-u):
t2 = [-0.08624772001349551; -0.2693995233777621; -0.9591586039838815];

% ******************   KINECT DATA   **********************************************************

%  Mean of data 1, centroidA:
%  0.419643 0.200553  1.73732
%  Mean of data 2, centroidB:
%  -0.146112 0.216527  1.72447
%  Points Projected data 1:
Xdata1_kinect = [0.207514  0.221906     1.799;
  0.96363  0.166068    2.0046;
  0.96363  0.166068    2.0046;
 0.873861 -0.160645    1.9532;
 0.734185  0.146706    2.0046;
 0.607563  0.511499    1.8504;
 0.734185  0.146706    2.0046;
 0.607425  0.513385    1.8504;
0.0786051 -0.226832    1.4906;
-0.105128 -0.199725    1.3364;
-0.0640818   0.20538    1.4906;
-0.0835231  0.477463    1.3878;
 0.556484 -0.167078    1.7476;
-0.158883  0.384229    1.3878;
 0.266086  0.510497    1.7476;
 0.252761  0.505628    1.7476;
 0.553742  0.261302    1.6962;
 0.598304 -0.218431     1.799;
 0.252761  0.505628    1.7476;
 0.553742  0.261302    1.6962];
%  Points Projected data 2:
Xdata2_kinect = [-0.373535  0.258909    1.8504;
 0.238518  0.166374    1.9018;
 0.238518  0.166374    1.9018;
 0.191921 -0.117567    1.9532;
0.0619488  0.159128    1.9532;
-0.000639431  0.545292    2.0046;
0.0619488  0.159128    1.9532;
-0.000639431  0.545292    2.0046;
-0.454758 -0.234078    1.6962;
-0.662282 -0.221433    1.5934;
       -0         0         0;
-0.608129  0.597235     1.542;
-0.0174204 -0.143628    1.9532;
-0.658341  0.463283    1.4392;
-0.299501   0.54935     1.799;
-0.314058  0.546104     1.799;
-0.00753268  0.267903    1.6962;
0.00332333 -0.191139    1.9532;
-0.314058  0.546104     1.799;
-0.00753268  0.267903    1.6962];
%  Recover Rotation:
R_p3d = [0.96885  0.0346949  -0.245205;
-0.0219188   0.998265  0.0546429;
  0.246675 -0.0475662    0.96793];
%  Euler angles values:
angles_R_p3d = [-3.23111;
-14.1939;
-2.05091];
%  Recover translation:
 t_p3d = [-0.133643;
-0.0694122;
-0.0511104];

%  Points Projected data R1 with t2: Triangulated with unitary t2
X_triang_unitT = [8.77622  9.09563  74.6091        1;
 107.137  18.9458  223.536        1;
 107.137  18.9458  223.536        1;
 54.1297  -9.6283  121.427        1;
 41.7955  8.41246   114.24        1;
 70.5726  58.9929   214.15        1;
 41.7955  8.41246   114.24        1;
 84.3277  70.6524  255.732        1;
 1.81406 -5.20774  34.1303        1;
-2.14804 -4.18635  27.8535        1;
-2.40858   8.1881  59.8402        1;
-7.22346  43.4905  126.684        1;
 18.1844  -5.3843  57.2969        1;
-11.5021  28.7207  104.006        1;
 23.6271  44.5834  153.182        1;
 22.4272  44.1623  153.144        1;
  53.483  25.2595  163.874        1;
 22.3377 -8.03245  67.4426        1;
 22.4272  44.1623  153.144        1;
  53.483  25.2595  163.874        1];
x1data_hom = [x1data ones(length(x1data),1)];
x2data_hom = [x2data ones(length(x2data),1)];

scale = norm(t_p3d) % Probing scale of translation vector obtained from 3D with translation vector obtained from 2D.
P1 = [k , zeros(3,1)]
P2 = [k*R1, t2] % I use | scale*t2 | in the last column and I don't have good results. *DOESN'T WORK
X_triang = linearTriangulation_normalized( P1, P2, x1data_hom, x2data_hom ) % Compare this with KINECT DATA 1, apparently a scale of 100
% X_triang is identical to X_triang_unitT **in this case

% Good results in rotation and translation from 3D data, Testing the error
% [RR_p3d tt_p3d]=posefrom3Dpoints( Xdata1_kinect, Xdata2_kinect );
% error = norm(R_p3d - RRp3d)

% DD = (R_p3d*Xdata1_kinect' + t_p3d)
% error = norm(DD - Xdata2_kinect)

% Good results in rotation and translation from 2D data (with fundamental matrix), Testing the error
% [R1m R2m t1m t2m]=posefromFundamental(F,k)
% norm(R1m - R1)


%% IMPROVING FEATURES functions

%% TESTING REPEATED KEYPOINTS
%% Enabled by default in featuresfunctions.cpp

% Distance of matches 200 - 300 // if is lesser than 200 pass, if it is between 200 - 300 check euclidean distance
% for euclidean distance find mean, and than the var not be greater than 8 or 10
%  p=[165.264;
%  184.867;
%  182.009;
%  174.104;
%  171.011;
%  167.087;
%  175.406;
%  172.912;
%  175.309;
%  179.945;
%  172.173;
%  172.579;
%  166.24;
%  184.794;
%  178.139;
%  265.867;
%  93.4289]

%% outliers 
% a1 = [384.172, 226.265]; b1 = [57.1009, 82.3134];
% d1 = norm(a1-b1);
% a2 = [614.156, 443.783]; b2 = [559.109, 126.578];
% d2 = norm(a2-b2);



GOOD:
Match 0: 57.4456
Match 1: 68.1909
Match 2: 76.948
Match 3: 77.8267
Match 4: 81.0679
Match 5: 101.681
Match 6: 103.402
Match 7: 106.597
Match 8: 118.76
Match 9: 130.257
Match 10: 138.625
Match 11: 139.061
Match 12: 140.95
Match 13: 149.65
Match 14: 231.899
Match 15: 238.103
Match 16: 253.458
Match 17: 255.171
Match 18: 255.5
Match 19: 272.149
Match 20: 278.927
Match 21: 291.549
Match 22: 292.192
Match 23: 296.378
Match 24: 300.98





Points Projected data 1:
  a=[0.0298762   0.207404     1.4906;
  0.829271   0.267112     1.7476;
  0.772443   0.465592     2.0046;
  0.117496   0.223532     1.5934;
  0.377148   0.196308      1.799;
  0.377148   0.196308      1.799;
  0.873861  -0.160645     1.9532;
  0.978274    0.47457     2.0046;
  0.367148   0.416359     1.7476;
  0.161008 -0.0608843     1.5934;
-0.0658313   0.189506     1.4906;
-0.0785942  -0.121061     1.4906;
  0.607425   0.513385     1.8504;
-0.0673584   -0.13677     1.4906;
 0.0786051  -0.226832     1.4906;
 -0.105128  -0.199725     1.3364;
  0.556484  -0.167078     1.7476;
  0.280126   0.507644     1.7476;
 -0.146788   0.386178     1.3878;
 -0.158883   0.384229     1.3878;
  0.598304  -0.218431      1.799;
  0.252761   0.505628     1.7476;
  0.949652   0.604048     1.7476;
 -0.158221   0.362876     1.3878;
  0.518596  -0.163325     1.7476;
 -0.126021   0.380414     1.3878;
-0.0527238  -0.116123     1.4906;
 -0.105835  -0.171681     1.3364;
   0.70339   0.479105     1.9532;
-0.0544737 -0.0587375     1.4906]
Points Projected data 2:
  b=[-0.513645    0.27853     1.6962;
  0.213172   0.266883     1.7476;
 0.0715948   0.343996     1.4906;
 -0.427635   0.282308     1.7476;
 -0.210886   0.233414     1.9532;
 -0.210886   0.233414     1.9532;
  0.191921  -0.117567     1.9532;
  0.206008   0.348716      1.542;
 -0.217858   0.478132     1.9532;
 -0.353193 -0.0335831     1.6448;
 -0.617372   0.255003     1.6448;
 -0.632368  -0.112599     1.6448;
-0.000639431   0.545292     2.0046;
 -0.618192  -0.126375     1.6448;
 -0.454758  -0.234078     1.6962;
 -0.662282  -0.221433     1.5934;
-0.0174204  -0.143628     1.9532;
 -0.284996   0.546406      1.799;
 -0.666677   0.482602     1.4906;
 -0.658341   0.463283     1.4392;
0.00332333  -0.191139     1.9532;
 -0.314058   0.546104      1.799;
  0.369809   0.619929     1.9532;
  -0.65746   0.440218     1.4392;
-0.0562156  -0.144623     1.9532;
 -0.680878   0.495472     1.5932;
 -0.600793  -0.101259     1.6448;
 -0.662015    -0.1015     1.5934;
 0.0509342   0.481271     2.0046;
 -0.624518 -0.0395768     1.6954]
 
 Euler angles values:
 -7.79907
7.33353
-3.22195
Recover translation:
 -0.79421
-0.171593
0.184254



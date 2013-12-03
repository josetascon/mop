%% NOTE: Rotations are recovered perfectly

%  Number of cameras: 5    ||      Number of Features: 55
%  Width in Images: 1024   ||      Height in Images: 1024
%  Camera Pose Variables 
%  Rotation matrix 0:
R1=[1 0 0;
0 1 0;
0 0 1];
%  Angles of Rotation 0:
%  -0  0 -0
%  Translation 0:
T1=[0 0 0]';

%  Rotation matrix 1:
R12=[0.995128 -0.0887286 -0.0429848;
 0.0909081   0.994515  0.0517221;
 0.0381598 -0.0553778   0.997736];
%  Angles of Rotation 1:
%  -2.96753 -2.46361  5.09519
%  Translation 1:
T12=[0.431 -0.548  0.654]';

%  Rotation matrix 2:
R13=[0.991167  -0.129036 -0.0306242;
  0.131972   0.982471   0.131656;
 0.0130991  -0.134534   0.990822];
%  Angles of Rotation 2:
%  -7.56885 -1.75491  7.41741
%  Translation 2:
T13=[0.861877 -0.0834116   0.571368]';

%  Rotation matrix 3:
R14=[0.998425 -0.0270483  0.0491614;
 0.0238589   0.997643  0.0643433;
-0.0507859  -0.063069   0.996716];
%  Angles of Rotation 3:
%  -3.69362 2.81788 1.55182
%  Translation 3:
T14=[0.988864 -0.273888  0.864982]';

%  Rotation matrix 4:
R15=[0.996969  0.0507104 -0.0589939;
 -0.050698   0.998713 0.00170676;
 0.0590045 0.00128928   0.998257];
%  Angles of Rotation 4:
%  -0.097961 -3.38207 -2.91181
%  Translation 4:
T15=[1.73695 -1.11019  1.25947]';


%% RECOVER *************************************

%  Rotation matrix 1:
%    0.995128 -0.0887281  -0.042984
%   0.0909077   0.994515  0.0517234
%   0.0381589 -0.0553789   0.997736
%  Angles of Rotation 1:
%   -2.9676 -2.46356  5.09516
%  Translation 1:
t12rcv=[0.450875 -0.57327 0.684158]';

%  Rotation matrix 2:
%    0.991167  -0.129035 -0.0306253
%    0.131971   0.982472   0.131656
%   0.0131002  -0.134535   0.990822
%  Angles of Rotation 2:
%  -7.56888 -1.75497  7.41734
%  Translation 2:
t23rcv=[1.19886 0.271503 0.555863]';
 
%   Rotation matrix 3:
%    0.998425  -0.027047  0.0491599
%   0.0238577   0.997643  0.0643437
%  -0.0507843 -0.0630695   0.996716
%  Angles of Rotation 3:
%  -3.69364 2.81779 1.55175
%  Translation 3:
t34rcv=[1.49355 0.182011  1.42472]';
 
%   Rotation matrix 4:
%    0.996969  0.0507103 -0.0589977
%   -0.050698   0.998713 0.00170624
%   0.0590082 0.00128999   0.998257
%  Angles of Rotation 4:
%  -0.0979312 -3.38228 -2.91181
%  Translation 4:
t45rcv=[2.1618 -0.499223   1.77852]';

%  Rotation matrix 1-3:
%    0.991167  -0.129036 -0.0306241
%    0.131972   0.982471   0.131656
%   0.0130989  -0.134534   0.990822
%  Angles of Rotation 1-3:
%  -7.56885 -1.7549  7.4174
%  Translation 1-3:
t13rcv=[0.830784 -0.0804025   0.550756]';
   
%  Rotation matrix 2-4:
%    0.994991 -0.0749403  0.0661553
%   0.0648605   0.987567   0.143192
%  -0.0760637  -0.138184   0.987481
%  Angles of Rotation 2-4:
%  -8.25078 3.79319 4.30724
%  Translation 2-4:
t24rcv=[1.68461 0.500491 0.859164]';

%  Rotation matrix 3-5:
%      0.993734     0.104986    0.0383581
%     -0.104912     0.994474  -0.00394818
%    -0.0385606 -0.000100769     0.999256
%  Angles of Rotation 3-5:
%  0.226381 2.19829 -6.0308
%  Translation 3-5:
t35rcv=[2.42413 -0.250594    1.2788]';






K = [[520.28075824744883 0.0 328.59418282611989]; [ 0.0 517.35099060486289 265.23529733631955]; [0.0 0.0 1.0]];

C1 = [K*R1 T1];
C2 = [K*R12 T12];
C3 = [K*R13 T13];
C4 = [K*R14 T14];
C5 = [K*R15 T15];
C2rcv = [K*R12 t12rcv];

%  R23 = (R13*R12');
%  t13rcv = t23rcv + (R13*R12')*t12rcv;

C3rcv = [K*R13 t13rcv];



X_structure=[0.726026    0.108043    0.299281;
   0.682107   -0.339594    0.756658;
   0.953464   -0.753608    0.427927;
   0.708114 -0.00230235    0.884567;
   0.898393    0.883889    0.374114;
   0.783693    0.559422     0.15438;
   0.288401    0.326673   0.0673584;
   0.934411   -0.485397    0.444107;
   0.249339    0.984028     0.26811;
    0.67273     0.50029    0.588824;
   0.626921  -0.0476589    0.142846;
   0.926201   -0.683445    0.473049;
   0.682859   -0.776517    0.596245;
   0.110786    0.639712   0.0950935;
   0.995354    0.436499   0.0370383;
   0.369468  0.00388468    0.816749;
   0.523848    0.580687    0.480086;
   0.591206    0.449509    0.737388;
  0.0353138    0.948188    0.729402;
   0.303424    0.293648    0.479547;
   0.892249   -0.452511    0.955717;
  0.0350945   -0.600108    0.113995;
   0.508143     0.76561    0.225736;
   0.104388    0.987182   0.0455924;
   0.199482     0.97789    0.763842;
    0.23652   -0.283175    0.265784;
   0.053269    0.764521   0.0561275;
   0.533355  -0.0530661    0.780882;
   0.270743   0.0175615    0.754976;
0.000144309     0.62441      0.4018;
   0.479691    0.408907    0.675544;
   0.435408    0.479096     0.87549;
   0.549403   -0.504617    0.758295;
   0.775139   -0.295841    0.751886;
   0.820732    0.103122    0.740831;
   0.584573    0.576162   0.0992437;
   0.850357      0.6827    0.981504;
   0.906485    -0.25059    0.454971;
   0.687367    0.290895    0.963752;
   0.442343    0.291184    0.775957;
   0.844143   -0.749434    0.480411;
   0.519687    0.121382    0.219959;
   0.395178   -0.779812     0.46765;
   0.153473    0.770466     0.81973;
   0.905359    0.411929    0.371291;
    0.64619   -0.418924    0.159372;
   0.745434   -0.718209 0.000721678;
   0.726938    -0.90524    0.375427;
    0.18191    0.469495    0.020874;
   0.145662   -0.645819    0.666466;
   0.921619   -0.957534    0.791749;
   0.402029   0.0818408    0.352439;
   0.621988    0.872196    0.462533;
  0.0896387   -0.820858    0.347766;
   0.909369     0.98986    0.053731];
  
  
X_hom = [X_structure ones(length(X_structure),1)];

imx1C1 = (C1*X_hom')';
imx1C2 = (C2*X_hom')';
imx1C3 = (C3*X_hom')';
imx1C4 = (C4*X_hom')';
imx1C5 = (C5*X_hom')';

imx1C2rcv = (C2rcv*X_hom')';
imx1C3rcv = (C3rcv*X_hom')';

n = length(X_structure);
for k = 1:n
    imx1C1(k,:) /= imx1C1(k,3);
    imx1C2(k,:) /= imx1C2(k,3);
    imx1C3(k,:) /= imx1C3(k,3);
    imx1C2rcv(k,:) /= imx1C2rcv(k,3);
    imx1C3rcv(k,:) /= imx1C3rcv(k,3);
end;

X3rcv1 = linearTriangulation( C1, C2rcv, imx1C1, imx1C2rcv);
X3rcv2 = linearTriangulation( C2rcv, C3rcv, imx1C2rcv, imx1C3rcv);

X3_c1 = linearTriangulation( C1, C2, imx1C1, imx1C2);
X3_c2 = linearTriangulation( C2, C3, imx1C2, imx1C3);

%  num_features;
%  num_cam;
%  
%  for cam = 1:num_cam
%      {
%          for (int ft = 0; ft < num_features ; ft++)
%          {
%  	  bool uaxis = (Observation[cam](0,ft) > 0.0 && Observation[cam](0,ft) < im_width);  // u axis
%  	  bool vaxis = (Observation[cam](1,ft) > 0.0 && Observation[cam](1,ft) < im_height);  // v axis
%  	  if (uaxis && vaxis) 
%  	      visibility(cam,ft) = true;
%  	      coordinates(cam,ft) = Observation[cam].col(ft);
%          }
%      }
%  
%  for
%  visibility



% RATIO
%  norm(X3rcv1(1,:)  - X3rcv1(4,:)) / norm( X3rcv2(1,:) - X3rcv2(4,:) );
%  norm(X3_c1(1,:)  - X3_c1(4,:)) / norm( X3_c2(1,:) - X3_c2(4,:) );

R23 = R13*R12';

a2nrcv = -cross(t13rcv,R23*t12rcv);
a2drcv = cross(t13rcv,t23rcv);
%Minimize the ratio ||a2nrcv - lambda*a2drcv||
lambda = 1.0; %initial value
lambda =  lambda + pinv(a2drcv)*(a2nrcv - lambda*a2drcv); % Normal equations
a2 = lambda;

a1nrcv = R23*t12rcv + a2*t23rcv;
a1drcv = t13rcv;
%Minimize the ratio ||a1nrcv - lambda*a1drcv||
lambda = 1.0; %initial value
lambda =  lambda + pinv(a1drcv)*(a1nrcv - lambda*a1drcv); % Normal equations
a1 = lambda;
%  
%  
%  T23 = T13 - R23*T12;
%  
%  % Normalized T12
%  T12n = T12/norm(T12);
%  T13n = T23 + R23*T12n; 
%  
%  C1n = [K*R1 T1];
%  C2n = [K*R12 T12n];
%  C3n = [K*R13 T13n];
%  imx1C1n = (C1n*X_hom')';
%  imx1C2n = (C2n*X_hom')';
%  imx1C3n = (C3n*X_hom')';
%  
%  for k = 1:n
%      imx1C1n(k,:) /= imx1C1n(k,3);
%      imx1C2n(k,:) /= imx1C2n(k,3);
%      imx1C3n(k,:) /= imx1C3n(k,3);
%  end;


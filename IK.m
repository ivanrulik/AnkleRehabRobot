clc
clear
digits(4)
close all

 


% Modified DH Parameter
% i     a(i-1)    alpla(i-1)    di      theta
% 1     0           0           0       q1
% 2     0           -pi/2       0       (-pi/2+q2)
% 3     L1 (65mm)   -pi/2       0       (-pi/2+q3)
% 4     L2 (105mm)  0           0       0

 


%=========================================================================
%  Forward Kinematics
%=========================================================================
syms L1 L2 H M real; % Link Length
syms q1 q2 q3 real; % Joint Angle
syms q1d q2d q3d real;
syms q1dd q2dd q3dd real;
syms m1 m2 m3 m4 real; 
syms Fx Fy Fz Tx Ty Tz real;
syms Tr1 Tr2 Tr3 real;
syms g real;

 


T10=[ cos(q1)    -sin(q1)     0       0;
      sin(q1)     cos(q1)     0       0;
        0           0         1       0;
        0           0         0       1 ];
    %....................................
R10=T10([1 2 3],[1 2 3]);
P10=T10([1 2 3],[4]);

 


T21=[ cos(-pi/2 + q2)   -sin(-pi/2 + q2)   0   0;
      0           0        1   0;
      -sin(-pi/2 + q2)   -cos(-pi/2 + q2)  0   0;
      0           0        0   1];

 

R21=T21([1 2 3],[1 2 3]);
P21=T21([1 2 3],[4]);

 


T32=[ cos(-pi/2 + q3)   -sin(-pi/2 + q3)   0   L1;
      0          0         1   0;
      -sin(-pi/2 + q3)  -cos(-pi/2 + q3)   0   0;
      0          0         0   1];

 

R32 =T32([1 2 3],[1 2 3]);
P32 =T32([1 2 3],[4]);

 

% Modified DH Parameter
% i     a(i-1)    alpla(i-1)    di      theta
% 1     0           0           0       q1
% 2     0           -pi/2       0       (-pi/2+q2)
% 3     L1 (0.065mm)   -pi/2       0       (-pi/2+q3)
% 4     L2 (0.105mm)  0           0       0

 


T43=[ 1   0   0   L2;
      0   1   0   0;
      0   0   1   0;
      0   0   0   1];

 

R43=T43([1 2 3],[1 2 3]); 
P43=T43([1 2 3],[4]);

 


%========================================================================
% Homogeneous Transformation matrix relates frame{8} to Base{0}
%........................................................................

 

T40=T10*T21*T32*T43;
% disp('T40= Homogeneous Transformation matrix relates frames {4} to {0}'); disp(T40);
P40 =T40([1 2 3],[4]);
% disp('P40=  Joint-4 Position wrt Base{0} '); disp(P40);
Px4=P40(1,1);
Py4=P40(2,1);
Pz4=P40(3,1);

 

%==================================================================
%  Numerical Diagnosis
%...................................................................
H = 1.7272; % Total height in m
W = 75; % Weight in kg
L1 = 0.065;
L2 = 0.105;

 


% % Elbow and Shoulder joint internal/external rotation are at 90o, other joints are at 0o
% q1=0;   q2=0;   q3=pi/2; q4=0;  q5=0;  q6=pi/2;  q7=0;   
% Joint_Angle=[q1 q2 q3 q4 q5 q6 q7]*180/pi
% 
% disp('P80='); disp(vpa(subs(P80')));

 

T30=T10*T21*T32;
P30 = T30([1 2 3],[4]);

 

T20 = T10*T21;

 

R10 =T10([1 2 3],[1 2 3]);
R20 =T20([1 2 3],[1 2 3]);
R30 =T30([1 2 3],[1 2 3]);
R40 =T40([1 2 3],[1 2 3]);

 


%  Centre of Gravity (C.G) in cm
L1c=L1/2; 
L2c=L2/2;

 

% Centre of gravity vector
Pc11   = [0 0 0]';
Pc22   = [L1c 0 0]';
Pc33   = [L2c 0 0]';
Pc44   = [0 0 0]';

 

% Inertia Tensor at the center of mass,Kg.m^2
% Equation: Moment of Inertia of hand around X axis (kg.cm2) = -19.5 + 0.17×Body weight (kg) +0.116×Stature (m)
Ixx1=0.0;   Iyy1=0.0;   Izz1=0.0; 
Ixx2=0.0004025;   Iyy2=0.0004025;   Izz2=0.0004025;
% Ixx2=0.0;   Iyy2=0.0;   Izz2=0.0;
% Ixx3=0.00287; Iyy3=0.003183; Izz3=0.0003233;
Ixx3=0.0;   Iyy3=0.0;   Izz3=0.0;
Ixx4=0.0;   Iyy4=0.0;   Izz4=0.0;

 

%=======================================
I1c1   = [Ixx1 0 0;0 Iyy1 0;0 0 Izz1];  
I2c2   = [Ixx2 0 0;0 Iyy2 0;0 0 Izz2];  
I3c3   = [Ixx3 0 0;0 Iyy3 0;0 0 Izz3];
I4c4   = [Ixx4 0 0;0 Iyy4 0;0 0 Izz4];

 


% The Iterative Newton Euler Dynamics Begins
%======================================================
w00 = zeros(3,1);  % Base of the robot {0} is not moving
wd00 = zeros(3,1); % Base of the robot {0} is not moving 
vd00 = [g 0 0]';% Gravity +ve in upward direction of X0

 

% Segment Weight
m1 = 0;
m2 = 0.4159;
% m3 = 0.370;
m3 = 0;
m4 = 0;

 

% Outward iteration, i=0 (where, i= 0 to 2)
% frame {0}, {1} are at the same point
R01 = R10'; 
w11  = R01*w00+[0 0 q1d]';
wd11 = R01*wd00+cross(R01*w00,[0 0 q1d]')+[0 0 q1dd]';
vd11 = R01*(cross(wd00,P10)+cross(w00,cross(w00,P10))+vd00);
vdc11= cross(wd11,Pc11)+cross(w11,cross(w11,Pc11))+vd11;
F11  = m1*vdc11;
N11  = I1c1*wd11+cross(w11,I1c1*w11);

 

%Outward iteration, i=1 (where, i= 0 to 2)
%frame {1}, {2} are at the same point
R12 = R21';
w22  = R12*w11+[0 0 q2d]';
wd22 = R12*wd11+cross(R12*w11,[0 0 q2d]')+[0 0 q2dd]';
vd22 = R12*(cross(wd11,P21)+cross(w11,cross(w11,P21))+vd11);
vdc22= cross(wd22,Pc22)+cross(w22,cross(w22,Pc22))+vd22;
F22  = m2*vdc22;
N22  = I2c2*wd22+cross(w22,I2c2*w22);

 

%Outward iteration, i=2 (where, i= 0 to 2)
R23 = R32';
w33   = R23*w22+[0 0 q3d]';
wd33  = R23*wd22+cross(R23*w22,[0 0 q3d]')+[0 0 q3dd]';
vd33  = R23*(cross(wd22,P32)+cross(w22,cross(w22,P32))+vd22);
vdc33 = cross(wd33,Pc33)+cross(w33,cross(w33,Pc33))+vd33;
F33   = m3*vdc33;
N33   = I3c3*wd33+cross(w33,I3c3*w33);

 

% %Outward iteration, i=3 (where, i= 0 to 6)
% R34 = R43';
% w44   = R34*w33+[0 0 q4d]';
% wd44  = R34*wd33+cross(R34*w33,[0 0 q4d]')+[0 0 q4dd]';
% vd44  = R34*(cross(wd33,P43)+cross(w33,cross(w33,P43))+vd33);
% vdc44 = cross(wd44,Pc44)+cross(w44,cross(w44,Pc44))+vd44;
% F44   = m4*vdc44;
% N44   = I4c4*wd44+cross(w44,I4c4*w44);

 


%Inward iteration, i=7 (where, i= 7 to 1)

 

% %Inward iteration, i=4 (where, i= 7 to 1)
% f44  = R54*f55+F44;  
% n44  = N44+R54*n55+cross(Pc44,F44)+cross(P54,R54*f55);
% tau4=n44([3],:);

 

%Inward iteration, i=3 (where, i= 7 to 1)
f33   = F33; % No External Force/Torques,
n33   = N33+cross(Pc33,F33);
tau3  =n33([3],:);
%Inward iteration, i=2 (where, i= 7 to 1)
f22   = R32*f33+F22;
n22   = N22+R32*n33+cross(Pc22,F22)+cross(P32,R32*f33);
tau2   =n22([3],:);
%Inward iteration, i=1 (where, i= 7 to 1)
f11   = R21*f22+F11;
n11   = N11+R21*n22+cross(Pc11,F11)+cross(P21,R21*f22);
tau1  =n11([3],:);
Torque= subs([tau1 tau2 tau3]);

 

% =======================================================
% Extracting Mass/inertia (M) element from torque equation
%-------------------------------------------------------
M11 = simplify(diff(tau1, q1dd)); 
M12 = simplify(diff(tau1, q2dd));
M13 = simplify(diff(tau1, q3dd));

 

M21 = simplify(diff(tau2, q1dd)); 
M22 = simplify(diff(tau2, q2dd)); 
M23 = simplify(diff(tau2, q3dd));

 


M31 = simplify(diff(tau3, q1dd)); 
M32 = simplify(diff(tau3, q2dd));
M33 = simplify(diff(tau3, q3dd));

 


Mass=vpa(subs([ M11 M12 M13;
                M21 M22 M23;
                M31 M32 M33]));
save Mass
% Generating Function    
matlabFunction(Mass,'File','PMassF1.m','Optimize',true);

 


% Alternate Method to extract Mass/inertia element
%---------------------------------
q = [q1 q2 q3]';
qd =[q1d q2d q3d]';
qdd=[q1dd, q2dd, q3dd]';
R_qd = [q1 q2 q3 q1d q2d q3d]';
M = jacobian(Torque, [qdd]);
% Generating Function
matlabFunction(M,'File','PMassF2.m','vars',{q});

 


% =======================================================
% Extracting Gravity elements from torque
%-------------------------------------------------------
G1 = simplify(diff(tau1, g)); 
G2 = simplify(diff(tau2, g));
G3 = simplify(diff(tau3, g)); 
Gravity= vpa(subs([G1*g G2*g G3*g]'));
save Gravity
% Generating Function    
matlabFunction(Gravity,'File','PGravityF1.m','Optimize',true);

 

% Alternate Method to Extract gravity term
%--------------------------------------------
G= jacobian(Torque, [g])*g;
save G
matlabFunction(G,'file','PGravityF2.m','vars',{q,g});

 


%=============================================================
% Extracting Coriolis/Centrifugal term (V)from torque equation
%-------------------------------------------------------------
q1dd=0; q2dd=0; q3dd=0; g=0;
V1t=vpa(subs(tau1)); 
V2t=vpa(subs(tau2));
V3t=vpa(subs(tau3));

 

VelC=[V1t V2t V3t]';
% Generating Function    
matlabFunction(VelC,'File','PVF1.m','Optimize',true);

 


% Alternate Method
%-------------------------------------
V = subs(Torque' - M*qdd-G);
save V
matlabFunction(V,'file','PVF2.m','vars',{R_qd});

 

%Testing
g=9.81;

 

% Data Set-2: For Dynamic Analysis: Produce Dynamic Joint Torque
% q1=pi/4; q2=pi/4; q3=pi/4;  q4=pi/4; q5=pi/4; q6=pi/4;  q7=pi/4;
% q1d=pi/6; q2d=pi/6; q3d=pi/6; q4d=pi/6; q5d=pi/6; q6d=pi/6; q7d=pi/6; 
% q1dd=pi/8; q2dd=pi/8; q3dd=pi/8; q4dd=pi/8; q5dd=pi/8; q6dd=pi/8; q7dd=pi/8;
q1=(48.6*pi/180); q2=(48.6*pi/180); q3=(48.6*pi/180);
q1d=(-27)*pi/180; q2d=(-27)*pi/180; q3d=(-27)*pi/180; 
q1dd=(-240)*pi/180; q2dd=(-240)*pi/180; q3dd=(-240)*pi/180;
qddr=[q1dd q2dd q3dd]';
%--------------------------------------------------------------

 

disp('Torque Recursive Newton-Euler Formulation= '); disp(vpa(subs([tau1 tau2 tau3])))
disp('The torque calculated adding M,V,G terms: Torque= M(q).qdd+V(q,qd)+G(q)')
disp(((vpa(subs(Mass))*vpa(subs(qddr)))+vpa(subs(VelC))+vpa(subs(Gravity)))')

 

% disp('From Alternate Method: The torque calculated adding M,V,G terms: Torque= M(q).qdd+V(q,qd)+G(q)')
% disp(((vpa(subs(M*qddr)))+vpa(subs(V))+vpa(subs(G)))')
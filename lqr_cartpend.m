clear all, close all, clc

g = 9.8067;      

Tstall = 0.085;  
RPMnoload = 300;      
wnl = (RPMnoload*2*pi)/60;   
Inl = 0.06;            
Vin = 12;                 
Istall = 2.4;            
Ng = 30;                
Dg = 0.0001;           
La = 0;   
              
Ra = Vin/Istall;            
Kt = (Tstall/(Vin*Ng))*Ra;  
Kb = (Vin-Ra*Inl)/(wnl*Ng); 
Kmd = 1.0;   
R = 0.035;                     
mw = 0.046;                  
Jw = 0.5*mw*R*R;        
mb = 0.800;                 
lb = 0.065; 
a = (Ng*Kt/R); 
fm = 0.0022; % Friction coefficient between body and DC motor
b = ((Ng*Kb*Kt)/R)+fm;                
Jb = 0.85*mb*lb*lb;
W = 0.27; % Width
D = 0.09; % Depth     
J = ((W*W)*0.5*b)/(R*R);
Jphi = 1.2168 * 10 ^ (-3);
K = W*a*0.5/R;
I = 0.5*mb*W*W + Jphi + (W*W)*0.5*(Jw+Ng*Ng*Jb)/(R*R);
Cm1 = Ng*Kt/Ra;
Cm2 = Cm1*Kb;
M = mb + 2*(mw+Jw/(R*R)) - ((mb*mb*lb*lb)/(mb*lb*lb + Jb));
J = (mb*lb*lb) + Jb - ((mb*mb*lb*lb)/(mb + 2*(mw+Jw/(R*R))));
C1 = (1/M)*((mb*mb*lb*lb*g)/(Jb + mb*lb*lb));
C2 = (2/M)*((1/R) + ((mb*lb)/(Jb + mb*lb*lb)));
C3 = (mb*lb*g)/J;
C4 = (2/(J*R))*(R + ((mb*lb)/(mb + 2*(mw + Jw/(R*R)))));

a11 = 0;
a12 = 1;
a13 = 0;
a14 = 0;
a21 = 0;
a22 = -C2*Cm2/(R*Ng);
a23 = -C1;
a24 = C2*Cm2;
a31 = 0;
a32 = 0;
a33 = 0;
a34 = 1;
a41 = 0;
a42 = C4*Cm2/(R*Ng);
a43 = C3;
a44 = -C4*Cm2;

b11 = 0;
b21 = C2*Cm1*Kmd;
b31 = 0;
b41 = -C4*Cm1*Kmd;

A=[a11 a12 a13 a14;
   a21 a22 a23 a24;
   a31 a32 a33 a34;
   a41 a42 a43 a44;];

B=[b11;b21;b31;b41;];

C=[
   1 0 0 0;
   0 1 0 0;
   0 0 1 0;
   0 0 0 1;];
D=[0;0;0;0];   
 
A2 = [0 1; (-K/I) (-J/I)];
B2 = [0  ; (-K/I)];
eig(A);

Q=[0.1 0 0 0;
   0 0.1 0 0;
   0 0 100 0;
   0 0 0 100;];

R=01;

K1=lqr(A,B,Q,R)

eigs=eig(A-B*K);
Q2 =([0.1 0;0 0.1]);
sys=ss(A,B,C);
R2=1;
AC=A-B*K;
K2=lqr(A2,B2,Q2,R2);
s= c2d(ss(A,B,C),0.005);
k1=dlqr(s.a,s.b,Q,R)
#a=ss(s.a)
eig(s.a-s.b*k1);
eig(s.a);
%step(s)
%hold on
%step(sys)

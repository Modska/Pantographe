% 5 BAR PARALLEL ROBOT
%%
clear all; clc;
a1=0;
a2=0.2; a3=0.35; a4=a2; a5=a3; a32=a3; a3D=2*a3;

%%
% Forward kinematics of planar manipulandum
syms a1 a2 a3 a4 a5 th2 th5 Cx Cy
eq1=((Cx-a2*cos(th2))^2)+((Cy-a2*sin(th2))^2)-a3^2;
eq2=((a5*cos(th5)-Cx)^2)+((Cy-a5*sin(th5))^2)-a4^2;

S=solve([eq1;eq2],[Cx;Cy]);
pretty(simplify(S.Cx))

%% Inverse kinematics of planar manipulandum
clear all; clc;
syms a1 a2 a3 a32 a3D a4 a5 th2 th5 XD YD

eq1=XD-a2*cos(th2)-a3D*cos(th5);
eq2=YD-a2*sin(th2)-a3D*sin(th5);
S=solve([eq1;eq2],[th2;th5]);

%% Simulation with inverse kinematics
clear all, clc
a1=0;
a2=0.2; a3=0.35; a4=a2; a5=a3; a32=a3; a3D=2*a3;

t=0:0.001:10;
YD=0.55+0.15*sin(2*pi*(1/5)*t);
XD=0.45*ones(1,length(t));

% load data_fw2inv

% th2_1=2*atan2((2*YD*a2 - (- XD.^4 - 2*XD.^2.*YD.^2 + 2*XD.^2*a2^2 + 2*XD.^2*a3D^2 - YD.^4 + 2*YD.^2*a2^2 + 2*YD.^2*a3D^2 - a2^4 + 2*a2^2*a3D^2 - a3D^4).^(1/2)),(XD.^2 + 2*XD*a2 + YD.^2 + a2^2 - a3D^2));
th2_2=2*atan2((2*YD*a2 + (- XD.^4 - 2*XD.^2.*YD.^2 + 2*XD.^2*a2^2 + 2*XD.^2*a3D^2 - YD.^4 + 2*YD.^2*a2^2 + 2*YD.^2*a3D^2 - a2^4 + 2*a2^2*a3D^2 - a3D^4).^(1/2)),(XD.^2 + 2*XD*a2 + YD.^2 + a2^2 - a3D^2));
% th5_1=2*atan2((2*YD*a3D + ((- XD.^2 - YD.^2 + a2^2 + 2*a2*a3D + a3D^2).*(XD.^2 + YD.^2 - a2^2 + 2*a2*a3D - a3D^2)).^(1/2)),(XD.^2 + 2*XD*a3D + YD.^2 - a2^2 + a3D^2));
th5_2=2*atan2((2*YD*a3D - ((- XD.^2 - YD.^2 + a2^2 + 2*a2*a3D + a3D^2).*(XD.^2 + YD.^2 - a2^2 + 2*a2*a3D - a3D^2)).^(1/2)),(XD.^2 + 2*XD*a3D + YD.^2 - a2^2 + a3D^2));

for i=1:length(th2_2);
    th2=th2_2(i);
    th5=th5_2(i);
    Cx_1(i)=(a2^3*cos(th2) + a5^3*cos(th5) - a2*a3^2*cos(th2) + a2*a4^2*cos(th2) + a3^2*a5*cos(th5) - a4^2*a5*cos(th5) - a2*sin(th2)*(2*a2^2*a3^2 - a3^4 - a4^4 - a5^4 - a2^4 + 2*a2^2*a4^2 - 4*a2^2*a5^2 + 2*a3^2*a4^2 + 2*a3^2*a5^2 + 2*a4^2*a5^2 - 2*a2^2*a5^2*cos(2*th2 - 2*th5) + 4*a2*a5^3*cos(th2 - th5) + 4*a2^3*a5*cos(th2 - th5) - 4*a2*a3^2*a5*cos(th2 - th5) - 4*a2*a4^2*a5*cos(th2 - th5))^(1/2) + a5*sin(th5)*(2*a2^2*a3^2 - a3^4 - a4^4 - a5^4 - a2^4 + 2*a2^2*a4^2 - 4*a2^2*a5^2 + 2*a3^2*a4^2 + 2*a3^2*a5^2 + 2*a4^2*a5^2 - 2*a2^2*a5^2*cos(2*th2 - 2*th5) + 4*a2*a5^3*cos(th2 - th5) + 4*a2^3*a5*cos(th2 - th5) - 4*a2*a3^2*a5*cos(th2 - th5) - 4*a2*a4^2*a5*cos(th2 - th5))^(1/2) - a2*a5^2*cos(th2 - 2*th5) - a2^2*a5*cos(2*th2 - th5))/(2*(a2^2 - 2*cos(th2 - th5)*a2*a5 + a5^2));
    Cx_2(i)=(a2^3*cos(th2) + a5^3*cos(th5) - a2*a3^2*cos(th2) + a2*a4^2*cos(th2) + a3^2*a5*cos(th5) - a4^2*a5*cos(th5) + a2*sin(th2)*(2*a2^2*a3^2 - a3^4 - a4^4 - a5^4 - a2^4 + 2*a2^2*a4^2 - 4*a2^2*a5^2 + 2*a3^2*a4^2 + 2*a3^2*a5^2 + 2*a4^2*a5^2 - 2*a2^2*a5^2*cos(2*th2 - 2*th5) + 4*a2*a5^3*cos(th2 - th5) + 4*a2^3*a5*cos(th2 - th5) - 4*a2*a3^2*a5*cos(th2 - th5) - 4*a2*a4^2*a5*cos(th2 - th5))^(1/2) - a5*sin(th5)*(2*a2^2*a3^2 - a3^4 - a4^4 - a5^4 - a2^4 + 2*a2^2*a4^2 - 4*a2^2*a5^2 + 2*a3^2*a4^2 + 2*a3^2*a5^2 + 2*a4^2*a5^2 - 2*a2^2*a5^2*cos(2*th2 - 2*th5) + 4*a2*a5^3*cos(th2 - th5) + 4*a2^3*a5*cos(th2 - th5) - 4*a2*a3^2*a5*cos(th2 - th5) - 4*a2*a4^2*a5*cos(th2 - th5))^(1/2) - a2*a5^2*cos(th2 - 2*th5) - a2^2*a5*cos(2*th2 - th5))/(2*(a2^2 - 2*cos(th2 - th5)*a2*a5 + a5^2));
    Cy_1(i)=(a2^3*sin(th2) + a5^3*sin(th5) + a2*cos(th2)*(2*a2^2*a3^2 - a3^4 - a4^4 - a5^4 - a2^4 + 2*a2^2*a4^2 - 4*a2^2*a5^2 + 2*a3^2*a4^2 + 2*a3^2*a5^2 + 2*a4^2*a5^2 - 2*a2^2*a5^2*cos(2*th2 - 2*th5) + 4*a2*a5^3*cos(th2 - th5) + 4*a2^3*a5*cos(th2 - th5) - 4*a2*a3^2*a5*cos(th2 - th5) - 4*a2*a4^2*a5*cos(th2 - th5))^(1/2) - a5*cos(th5)*(2*a2^2*a3^2 - a3^4 - a4^4 - a5^4 - a2^4 + 2*a2^2*a4^2 - 4*a2^2*a5^2 + 2*a3^2*a4^2 + 2*a3^2*a5^2 + 2*a4^2*a5^2 - 2*a2^2*a5^2*cos(2*th2 - 2*th5) + 4*a2*a5^3*cos(th2 - th5) + 4*a2^3*a5*cos(th2 - th5) - 4*a2*a3^2*a5*cos(th2 - th5) - 4*a2*a4^2*a5*cos(th2 - th5))^(1/2) - a2*a3^2*sin(th2) + a2*a4^2*sin(th2) + a3^2*a5*sin(th5) - a4^2*a5*sin(th5) + a2*a5^2*sin(th2 - 2*th5) - a2^2*a5*sin(2*th2 - th5))/(2*(a2^2 - 2*cos(th2 - th5)*a2*a5 + a5^2));
    Cy_2(i)=(a2^3*sin(th2) + a5^3*sin(th5) - a2*cos(th2)*(2*a2^2*a3^2 - a3^4 - a4^4 - a5^4 - a2^4 + 2*a2^2*a4^2 - 4*a2^2*a5^2 + 2*a3^2*a4^2 + 2*a3^2*a5^2 + 2*a4^2*a5^2 - 2*a2^2*a5^2*cos(2*th2 - 2*th5) + 4*a2*a5^3*cos(th2 - th5) + 4*a2^3*a5*cos(th2 - th5) - 4*a2*a3^2*a5*cos(th2 - th5) - 4*a2*a4^2*a5*cos(th2 - th5))^(1/2) + a5*cos(th5)*(2*a2^2*a3^2 - a3^4 - a4^4 - a5^4 - a2^4 + 2*a2^2*a4^2 - 4*a2^2*a5^2 + 2*a3^2*a4^2 + 2*a3^2*a5^2 + 2*a4^2*a5^2 - 2*a2^2*a5^2*cos(2*th2 - 2*th5) + 4*a2*a5^3*cos(th2 - th5) + 4*a2^3*a5*cos(th2 - th5) - 4*a2*a3^2*a5*cos(th2 - th5) - 4*a2*a4^2*a5*cos(th2 - th5))^(1/2) - a2*a3^2*sin(th2) + a2*a4^2*sin(th2) + a3^2*a5*sin(th5) - a4^2*a5*sin(th5) + a2*a5^2*sin(th2 - 2*th5) - a2^2*a5*sin(2*th2 - th5))/(2*(a2^2 - 2*cos(th2 - th5)*a2*a5 + a5^2));

    Dx_1(i)=Cx_1(i)+a32*(Cx_1(i)-a2*cos(th2))/a3;
    Dx_2(i)=Cx_2(i)+a32*(Cx_2(i)-a2*cos(th2))/a3;
    Dy_1(i)=Cy_1(i)+a32*(Cy_1(i)-a2*sin(th2))/a3;
    Dy_2(i)=Cy_2(i)+a32*(Cy_2(i)-a2*sin(th2))/a3;

    Dx=Dx_2(i);
    Dy=Dy_2(i);
end

Vyd=0.15*2*pi*(1/5)*cos(2*pi*(1/5)*t);
Vxd=0*ones(1,length(t));

for j=1:length(t);
    th_D=inv([-a2*sin(th2_2(1,j)) -a3D*sin(th5_2(1,j));a2*cos(th2_2(1,j)) a3D*cos(th5_2(1,j))])*[Vyd(j);Vxd(j)];
    dth2_d(j)=th_D(1);
    dth5_d(j)=th_D(2);
end

th2_sim=th2_2;
th5_sim=th5_2;
XD_sim=XD;
YD_sim=YD;

save data_sim_control th2_sim th5_sim XD_sim YD_sim

%% Simulation with forward kinematics (Cx and Cy calculated as a1=0)
clear all; clc;
a1=0;
a2=0.2; a3=0.35; a4=a2; a5=a3; a32=a3; a3D=2*a3;
TH2=(65:5:130)*pi/180;
TH5=(45:5:110)*pi/180;

for i=1:length(TH2);
    th2=TH2(i);
    th5=TH5(i);
    Cx_1(i)=(a2^3*cos(th2) + a5^3*cos(th5) - a2*a3^2*cos(th2) + a2*a4^2*cos(th2) + a3^2*a5*cos(th5) - a4^2*a5*cos(th5) - a2*sin(th2)*(2*a2^2*a3^2 - a3^4 - a4^4 - a5^4 - a2^4 + 2*a2^2*a4^2 - 4*a2^2*a5^2 + 2*a3^2*a4^2 + 2*a3^2*a5^2 + 2*a4^2*a5^2 - 2*a2^2*a5^2*cos(2*th2 - 2*th5) + 4*a2*a5^3*cos(th2 - th5) + 4*a2^3*a5*cos(th2 - th5) - 4*a2*a3^2*a5*cos(th2 - th5) - 4*a2*a4^2*a5*cos(th2 - th5))^(1/2) + a5*sin(th5)*(2*a2^2*a3^2 - a3^4 - a4^4 - a5^4 - a2^4 + 2*a2^2*a4^2 - 4*a2^2*a5^2 + 2*a3^2*a4^2 + 2*a3^2*a5^2 + 2*a4^2*a5^2 - 2*a2^2*a5^2*cos(2*th2 - 2*th5) + 4*a2*a5^3*cos(th2 - th5) + 4*a2^3*a5*cos(th2 - th5) - 4*a2*a3^2*a5*cos(th2 - th5) - 4*a2*a4^2*a5*cos(th2 - th5))^(1/2) - a2*a5^2*cos(th2 - 2*th5) - a2^2*a5*cos(2*th2 - th5))/(2*(a2^2 - 2*cos(th2 - th5)*a2*a5 + a5^2));
    Cx_2(i)=(a2^3*cos(th2) + a5^3*cos(th5) - a2*a3^2*cos(th2) + a2*a4^2*cos(th2) + a3^2*a5*cos(th5) - a4^2*a5*cos(th5) + a2*sin(th2)*(2*a2^2*a3^2 - a3^4 - a4^4 - a5^4 - a2^4 + 2*a2^2*a4^2 - 4*a2^2*a5^2 + 2*a3^2*a4^2 + 2*a3^2*a5^2 + 2*a4^2*a5^2 - 2*a2^2*a5^2*cos(2*th2 - 2*th5) + 4*a2*a5^3*cos(th2 - th5) + 4*a2^3*a5*cos(th2 - th5) - 4*a2*a3^2*a5*cos(th2 - th5) - 4*a2*a4^2*a5*cos(th2 - th5))^(1/2) - a5*sin(th5)*(2*a2^2*a3^2 - a3^4 - a4^4 - a5^4 - a2^4 + 2*a2^2*a4^2 - 4*a2^2*a5^2 + 2*a3^2*a4^2 + 2*a3^2*a5^2 + 2*a4^2*a5^2 - 2*a2^2*a5^2*cos(2*th2 - 2*th5) + 4*a2*a5^3*cos(th2 - th5) + 4*a2^3*a5*cos(th2 - th5) - 4*a2*a3^2*a5*cos(th2 - th5) - 4*a2*a4^2*a5*cos(th2 - th5))^(1/2) - a2*a5^2*cos(th2 - 2*th5) - a2^2*a5*cos(2*th2 - th5))/(2*(a2^2 - 2*cos(th2 - th5)*a2*a5 + a5^2));
    Cy_1(i)=(a2^3*sin(th2) + a5^3*sin(th5) + a2*cos(th2)*(2*a2^2*a3^2 - a3^4 - a4^4 - a5^4 - a2^4 + 2*a2^2*a4^2 - 4*a2^2*a5^2 + 2*a3^2*a4^2 + 2*a3^2*a5^2 + 2*a4^2*a5^2 - 2*a2^2*a5^2*cos(2*th2 - 2*th5) + 4*a2*a5^3*cos(th2 - th5) + 4*a2^3*a5*cos(th2 - th5) - 4*a2*a3^2*a5*cos(th2 - th5) - 4*a2*a4^2*a5*cos(th2 - th5))^(1/2) - a5*cos(th5)*(2*a2^2*a3^2 - a3^4 - a4^4 - a5^4 - a2^4 + 2*a2^2*a4^2 - 4*a2^2*a5^2 + 2*a3^2*a4^2 + 2*a3^2*a5^2 + 2*a4^2*a5^2 - 2*a2^2*a5^2*cos(2*th2 - 2*th5) + 4*a2*a5^3*cos(th2 - th5) + 4*a2^3*a5*cos(th2 - th5) - 4*a2*a3^2*a5*cos(th2 - th5) - 4*a2*a4^2*a5*cos(th2 - th5))^(1/2) - a2*a3^2*sin(th2) + a2*a4^2*sin(th2) + a3^2*a5*sin(th5) - a4^2*a5*sin(th5) + a2*a5^2*sin(th2 - 2*th5) - a2^2*a5*sin(2*th2 - th5))/(2*(a2^2 - 2*cos(th2 - th5)*a2*a5 + a5^2));
    Cy_2(i)=(a2^3*sin(th2) + a5^3*sin(th5) - a2*cos(th2)*(2*a2^2*a3^2 - a3^4 - a4^4 - a5^4 - a2^4 + 2*a2^2*a4^2 - 4*a2^2*a5^2 + 2*a3^2*a4^2 + 2*a3^2*a5^2 + 2*a4^2*a5^2 - 2*a2^2*a5^2*cos(2*th2 - 2*th5) + 4*a2*a5^3*cos(th2 - th5) + 4*a2^3*a5*cos(th2 - th5) - 4*a2*a3^2*a5*cos(th2 - th5) - 4*a2*a4^2*a5*cos(th2 - th5))^(1/2) + a5*cos(th5)*(2*a2^2*a3^2 - a3^4 - a4^4 - a5^4 - a2^4 + 2*a2^2*a4^2 - 4*a2^2*a5^2 + 2*a3^2*a4^2 + 2*a3^2*a5^2 + 2*a4^2*a5^2 - 2*a2^2*a5^2*cos(2*th2 - 2*th5) + 4*a2*a5^3*cos(th2 - th5) + 4*a2^3*a5*cos(th2 - th5) - 4*a2*a3^2*a5*cos(th2 - th5) - 4*a2*a4^2*a5*cos(th2 - th5))^(1/2) - a2*a3^2*sin(th2) + a2*a4^2*sin(th2) + a3^2*a5*sin(th5) - a4^2*a5*sin(th5) + a2*a5^2*sin(th2 - 2*th5) - a2^2*a5*sin(2*th2 - th5))/(2*(a2^2 - 2*cos(th2 - th5)*a2*a5 + a5^2));

    Dx_1(i)=Cx_1(i)+a32*(Cx_1(i)-a2*cos(th2))/a3;
    Dx_2(i)=Cx_2(i)+a32*(Cx_2(i)-a2*cos(th2))/a3;
    Dy_1(i)=Cy_1(i)+a32*(Cy_1(i)-a2*sin(th2))/a3;
    Dy_2(i)=Cy_2(i)+a32*(Cy_2(i)-a2*sin(th2))/a3;

    Dx=Dx_2(i);
    Dy=Dy_2(i);
end

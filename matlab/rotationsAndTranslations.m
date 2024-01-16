function [ angles,XYZ ] = rotationsAndTranslations( T,right )
%Takes a transformation matrix and outputs rotations and translations

R=[T(1:4,1:3),[0;0;0;1]];%T is translation of the femur to the tibia in the femoral reference frame

%RotationMatrixSymbolic.m is useful for seeing full rotation matricies and
%determining these formulae

%For the knee
if right
    Rx = atan2(R(2,3),R(3,3))*180/pi;
    Ry = asind(R(1,3));
    Rz = atan2(R(1,2),R(1,1))*180/pi;
else
    Rx = atan2(R(2,3),R(3,3))*180/pi;
    Ry = -asind(R(1,3));
    Rz = -atan2(R(1,2),R(1,1))*180/pi;

end

%For the Hip
% Rx = asind(-R(3,2));%see RotationMatrixSymbolic.m
% Ry = atan2(R(3,1),R(3,3))*180/pi;
% Rz = atan2(R(1,2),R(2,2))*180/pi;

Tl=R\T;%Tl is translation of the femur to the tibia in the tibial reference frame
X=Tl(1,4);
Y=Tl(2,4);
Z=Tl(3,4);

angles=[Rx,Ry,Rz];
XYZ=[X,Y,Z];

end


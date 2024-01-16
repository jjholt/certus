function [ gTt0 ] = defineTrackerFixedFrame(eulerAngles,XYZ)
%

i=1;

Rx=eulerAngles(i,1);
Ry=eulerAngles(i,2);
Rz=eulerAngles(i,3);

Qx=[1   0         0
    0   cosd(Rx)  -sind(Rx)
    0   sind(Rx)  cosd(Rx)];

Qy=[cosd(Ry)    0   sind(Ry)
    0           1   0
    -sind(Ry)   0   cosd(Ry)];

Qz=[cosd(Rz)    -sind(Rz)   0
    sind(Rz)    cosd(Rz)    0
    0           0           1];

% Qx=[1   0         0
%     0   cosd(Rx)  sind(Rx)
%     0   -sind(Rx)  cosd(Rx)];
% 
% Qy=[cosd(Ry)    0   -sind(Ry)
%     0           1   0
%     sind(Ry)   0   cosd(Ry)];
% 
% Qz=[cosd(Rz)    sind(Rz)   0
%     -sind(Rz)    cosd(Rz)    0
%     0           0           1];

rot=Qz*Qy*Qx;
%rot=Qz*Qx*Qy;
rot=[rot,[0 0 0]';0 0 0 1];

trans=[1 0 0 XYZ(i,1);
       0 1 0 XYZ(i,2);
       0 0 1 XYZ(i,3)
       0 0 0 1];

% trans=eye(4);

gTt0=trans+rot;



end

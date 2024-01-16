function [ RxRyRz ] = quaternion2euler( Q )
%Converts quaternions to euler angles: http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

q0=Q(1);
q1=Q(2);
q2=Q(3);
q3=Q(4);

Rx=atan2((2*(q0*q1+q2*q3)),(1-2*(q1^2+q2^2)))*180/pi;

Ry=asind(2*(q0*q2-q3*q1));

Rz=atan2((2*(q0*q3+q1*q2)),(1-2*(q2^2+q3^2)))*180/pi;

RxRyRz=[Rx,Ry,Rz];
end


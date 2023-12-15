function gst = ur5FwdKin_old(q)

%Lengths of arms of UR5
l0 = 89.2/1000; %lo is the distance between the base link and the 2nd joint above it.
l1 = 425/1000;
l2 = 392/1000;
l3 = 109.3/1000;
l4 = 94.75/1000;
l5 = 82.5/1000;

%Home configuration of UR5 as in RVIZ. In RVIZ, the tool frame and the base
%link frame are not aligned to each other and the home position of the tool
%frame with respect to the base link below reflects that.

M = [-1 0 0 l1+l2;0 0 1 l3+l5;0 1 0 l0-l4;0 0 0 1];

%Screw axis.
s1 = [0;0;0;0;0;1];
s2 = [-l0;0;0;0;1;0];
s3 = [-l0;0;l1;0;1;0];
s4 = [-l0;0;l1+l2;0;1;0];
s5 = [0;-l1-l2;0;0;0;1];
s6 = [l4-l0;0;l1+l2;0;1;0];
Slist = [s1 s2 s3 s4 s5 s6];


T = M;
for i = size(q): -1: 1
    T = exp6(Slist(:,i),q(i)) * T;
end

%This part of the function checks for values whose norm is less than 10^-6
%and assigns them a zero value.


sz = size(T);
elements = sz(1)*sz(2);
for i = 1:elements
    if(norm(T(i))<1e-6)
        T(i) = 0;
    end
end

gst = T; %Our final configuration of the tool tip of UR5 arm.

%This fucntion calculates the exp6 of a given screw and theta.
function T =exp6(S,theta)

 omega = S(4:end); % Extracting omega
 omega_hat = SKEW3(omega); %Calculating the skew symmetric matrix
 v = S(1:3); %extracting v

 a_1 = rodriguez(omega_hat,theta);
 a_2 = (eye(3) - rodriguez(omega_hat,theta))*(omega_hat*v) + omega*omega'*v*theta;
 a_3 = [0 0 0];
 a_4 = 1;
 T = [a_1 a_2;a_3 a_4];
end

%SKEW3 function converts a vector containing omega values to its
%corresponding skew symmetric matrix.

function skew = SKEW3(x)
 x1 = x(1);
 x2 = x(2);
 x3 = x(3);

 skew = [0 -x3 x2;x3 0 -x1;-x2 x1 0];
end

%rodriguez function calcualtes the Rotation matrix given a skew symmetric
%omega_hat and theta.

function exp3 = rodriguez(omega_hat, theta)

 I = eye(3);
 exp3 = I + omega_hat*sin(theta) + omega_hat*omega_hat*(1 - cos(theta));

end

end
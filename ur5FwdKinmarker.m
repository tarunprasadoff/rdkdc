function map = ur5FwdKinmarker(thetas)

%note that measurements for L are in millimeters
%link lengths
l0 = 89.2/1000;
l1 = 425/1000;
l2 = 392/1000;
l3 = 109.3/1000;
l4 = 94.75/1000;
l5 = 82.5/1000;
lm1 = 122.28/1000; %note that this one may need to be adjusted
lm2 = 49/1000;
%extract thetas
theta1 = thetas(1);
theta2 = thetas(2);
theta3 = thetas(3);
theta4 = thetas(4);
theta5 = thetas(5);
theta6 = thetas(6);
twist1 = createtwist([0;0;1], [0;0;0]);
twist2 = createtwist([1;0;0], [0;0;l0]);
twist3 = createtwist([1;0;0], [0;0;l1]);
twist4 = createtwist([1;0;0], [0;0;l1+l2]);
twist5 = createtwist([0;0;1], [l3;0;0]);
twist6 = createtwist([1;0;0], [0;0;l1+l2+l4]);

gst = [1,0,0,l3+l5;0,1,0,0;0,0,1,l0+l1+l2+l4;0,0,0,1];

g1 = EXPCR(twist1(4:6), theta1);
g1(1:3,4) = ((eye(3) - g1)*cross(twist1(4:6),twist1(1:3))+twist1(4:6)*transpose(twist1(4:6))*twist1(1:3)*theta1);
g1(4,4) = 1;
g2 = EXPCR(twist2(4:6), theta2);
g2(1:3,4) = ((eye(3) - g2)*cross(twist2(4:6),twist2(1:3))+twist2(4:6)*transpose(twist2(4:6))*twist2(1:3)*theta2);
g2(4,4) = 1;
g3 = EXPCR(twist3(4:6), theta3);
g3(1:3,4) = ((eye(3) - g3)*cross(twist3(4:6),twist3(1:3))+twist3(4:6)*transpose(twist3(4:6))*twist3(1:3)*theta3);
g3(4,4) = 1;
g4 = EXPCR(twist4(4:6), theta4);
g4(1:3,4) = ((eye(3) - g4)*cross(twist4(4:6),twist4(1:3))+twist4(4:6)*transpose(twist4(4:6))*twist4(1:3)*theta4);
g4(4,4) = 1;
g5 = EXPCR(twist5(4:6), theta5);
g5(1:3,4) = ((eye(3) - g5)*cross(twist5(4:6),twist5(1:3))+twist5(4:6)*transpose(twist5(4:6))*twist5(1:3)*theta5);
g5(4,4) = 1;
g6 = EXPCR(twist6(4:6), theta6);
g6(1:3,4) = ((eye(3) - g6)*cross(twist6(4:6),twist6(1:3))+twist6(4:6)*transpose(twist6(4:6))*twist6(1:3)*theta6);
g6(4,4) = 1;
%may just remove this, since the other position isn't necessary?
g7(1:3,1:3) = eye(3);
g7(1:3,4) = [lm1;0;lm2];
g7(4,4) = 1;
map = g1*g2*g3*g4*g5*g6*g7*gst;
end
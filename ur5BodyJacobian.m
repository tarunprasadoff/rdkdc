function bjacobian = ur5BodyJacobian(thetainput)
%adjust theta inputs for the home position of the ur5
thetas = [thetainput(1);thetainput(2)+pi/2;thetainput(3);thetainput(4)+pi/2;thetainput(5);thetainput(6)];
%define lengths
l0 = 89.2/1000;
l1 = 425/1000;
l2 = 392/1000;
l3 = 109.3/1000;
l4 = 94.75/1000;
l5 = 82.5/1000;
%define twists
twist1 = createtwist([0;0;1], [0;0;0]);
twist2 = createtwist([0;1;0], [0;0;l0]);
twist3 = createtwist([0;1;0], [0;0;l0+l1]);
twist4 = createtwist([0;1;0], [0;0;l0+l1+l2]);
twist5 = createtwist([0;0;1], [0;l3;0]);
twist6 = createtwist([0;1;0], [0;0;l0+l1+l2+l4]);
%setup variables and gstzero
sjacobian = zeros(6);
gstzero = [1,0,0,0;0,1,0,l3+l5;0,0,1,l0+l1+l2+l4;0,0,0,1];
twists = {twist1;twist2;twist3;twist4;twist5;twist6};
mygs = {};
adjoints = {};
adjointer = eye(6);
fulltransform = eye(4);
%for loop to calculate each twist and each adjoint to get the spatial
%jacobian
for i = 1:6
    transformation = eye(4);
    adjoint = eye(6);
    mytwist = twists{i};
    rotation = EXPCR(mytwist(4:6), thetas(i));
    position = ((eye(3) - rotation)*cross(mytwist(4:6),mytwist(1:3))+mytwist(4:6)*transpose(mytwist(4:6))*mytwist(1:3)*thetas(i));
    
    transformation(1:3, 1:3) = rotation;
    transformation(1:3,4) = position;
    transformation(4,4) = 1;
    mygs{i} = transformation;
    adjoint(1:3,1:3) = rotation;
    adjoint(4:6,4:6) = rotation;
    adjoint(1:3,4:6) = SKEW3(position)*rotation;
    adjoints{i} = adjoint;
    col = [0;0;0;0;0;0];
    adjointer = adjointer*adjoints{i};
    %fulltransform = fulltransform * transformation;
    if(i > 1)
        col = adjointer * twists{i};
    else
        col = twists{i};
    end
    sjacobian(1:6,i) = col;
    %fulltransform = fulltransform*transformation;
end
%get the proper transform for the inverse adjoint
fulltransform = ur5FwdKin(thetainput);
%convert to body jacobian
gstadjoint = zeros(6);
gstadjoint(1:3,1:3) = transpose(fulltransform(1:3,1:3));
gstadjoint(4:6,4:6) = transpose(fulltransform(1:3,1:3));
gstadjoint(1:3,4:6) = -transpose(fulltransform(1:3,1:3))*SKEW3(fulltransform(1:3,4));
%return body jacobian
bjacobian = gstadjoint*sjacobian;
end
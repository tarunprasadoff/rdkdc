function map = ur5FwdKin(thetas)

%note that measurements for L are in millimeters, converted to meters for
%this code
%link lengths
l0 = 89.2/1000;
l1 = 425/1000;
l2 = 392/1000;
l3 = 109.3/1000;
l4 = 94.75/1000;
l5 = 82.5/1000;
lm1 = 122.28/1000;
lm2 = 49/1000;

marker_transform = [
    [ eye(3), [lm2;0;lm1] ];
    [ 0, 0, 0, 1 ]
];

%setup twists
twist1 = createtwist([0;0;1], [0;0;0]);
twist2 = createtwist([0;1;0], [0;0;l0]);
twist3 = createtwist([0;1;0], [0;0;l0+l1]);
twist4 = createtwist([0;1;0], [0;0;l0+l1+l2]);
twist5 = createtwist([0;0;1], [0;l3;0]);
twist6 = createtwist([0;1;0], [0;0;l0+l1+l2+l4]);
%extract thetas, adjusting for the configuration of the thetas
%NOTE: COULD SETUP GST WITH PROPER ROTATION, BUT THIS WORKAROUND WORKS AND
%I DONT WANT TO TOUCH IT FURTHER!!
thetagroup = [thetas(1);thetas(2)+pi/2;thetas(3);thetas(4)+pi/2;thetas(5);thetas(6)];
twistgroup = {twist1;twist2;twist3;twist4;twist5;twist6};
map = eye(4);
%calculate forward kinematics
for i = 1:6
    theta = thetagroup(i);
    twist = twistgroup{i};

    g = EXPCR(twist(4:6), theta);
    g(1:3,4) = ((eye(3) - g)*cross(twist(4:6),twist(1:3))+twist(4:6)*transpose(twist(4:6))*twist(1:3)*theta);
    g(4,4) = 1;
    map = map*g;
end
%setup gst at t = 0
%REPEAT NOTE ABOVE; GST'S ROTATION IS NOT IDENTITY, BUT SINCE CODE WORKS
%RIGHT NOW I DONT WANT TO ADJUST
gst = [1,0,0,0;0,1,0,l3+l5;0,0,1,l0+l1+l2+l4;0,0,0,1];
%this extra rotation can probably be thrown into the gst zero configuration
%too, but again, workaround for now
extrarotation = ROTX(3*pi/2);
extrarotation(4,4) = 1;
%return forward map
%map = map*gst*extrarotation*marker_transform;
map = map*gst*extrarotation;

end
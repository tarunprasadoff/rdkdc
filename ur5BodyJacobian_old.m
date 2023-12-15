% function bodyjacobian = ur5BodyJacobian(thetas)
%calculate forward kinematics with theta input
function bjacobian = ur5BodyJacobian(thetasreal)
    sjacobian = sym([]);
    syms theta1 theta2 theta3 theta4 theta5 theta6;
    thetas = [theta1;theta2;theta3;theta4;theta5;theta6];
    gst = ur5FwdKin([theta1;theta2;theta3;theta4;theta5;theta6]);
    %take inverse
    gstinv = inv(gst);
    %take derivatives
    for i = 1:6
        gstderiv = diff(gst, thetas(i));
        colm = gstderiv*gstinv;
        rotation = [colm(3,2);colm(1,3);colm(2,1)];
        pos = [colm(1,4);colm(2,4);colm(3,4)];
        col = [pos;rotation];
        sjacobian(1:6,i) = col;
   
    end
    
%define adjoint of gst
gst = subs(gst, thetas, thetasreal);
sjacobian = subs(sjacobian, thetas, thetasreal);
gst = simplify(gst);
sjacobian = simplify(sjacobian);
adjoint(1:3,1:3) = transpose(gst(1:3,1:3));
adjoint(4:6,4:6) = transpose(gst(1:3,1:3));
adjoint(4:6,1:3) = -transpose(gst(1:3,1:3))*SKEW3(gst(1:3,4));


bjacobian = adjoint*sjacobian;
end
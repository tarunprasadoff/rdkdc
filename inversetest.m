ur5 = ur5_interface();
disp("Move Robot to start position.")
extrarot = ROTX(pi/2);
extrarot(4,4) = 1;
start_frame = [1,0,0,200/1000;0,1,0,200/1000;0,0,1,200/1000;0,0,0,1]*extrarot
w = waitforbuttonpress();
currentangles = ur5.get_current_joints();
starttransform = ur5FwdKin(currentangles);
newval = ur5InverseControl(ur5, starttransform, 200/1000,200/1000,200/1000);
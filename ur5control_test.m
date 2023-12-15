extrarot = ROTX(pi/2);
extrarot(4,4) = 1;
start_frame = [1,0,0,200/1000;0,1,0,200/1000;0,0,1,200/1000;0,0,0,1]*extrarot
new_transform_over1 = start_frame*[1,0,0,0;0,1,0,-overdistance1;0,0,1,0;0,0,0,1];
frame2movement = 
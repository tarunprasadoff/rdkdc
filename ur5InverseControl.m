function done = ur5InverseControl(ur5, start_frame, overdistance1, downdistance, overdistance2)
    %first setup the start and end frames
    %w = waitforbuttonpress;
    stepsize = 100;
    timing = 0.15;
    
    %check orientation of the start_frame

    %start_frame = [1,0,0,200/1000;0,1,0,200/1000;0,0,1,50/1000;0,0,0,1]*extrarot;
    % start_frame = ur5FwdKin(ur5.home());
    %ur5.move_joints(ur5InvKin(start_frame), 30);
    
    %pause(30);
    %Frame_A = tf_frame("base_link", "frame_A", start_frame);
    %goalframe = ur5FwdKin([pi/6,-pi/3,pi,pi/12,pi/8,pi]);
    
    %now we need to move over a given distance to the right, and then down
    %a given distance, coinciding with the overdistance and downdistance
    %variables
    %i write the vector in the 6th frame's reference, so need to go over
    %negative distance, down distance is positive
    for i = 1:stepsize
    new_transform_over1 = start_frame*[1,0,0,0;0,1,0,-overdistance1*i/stepsize;0,0,1,0;0,0,0,1];
    %theta calculation to move ur5 over distance
    thetasolutionsover1 = ur5InvKin(new_transform_over1);
    %we now feed this transformation to the ur5, figuring out what the best
    %using the body jacobian as the calculation, and it's determinant for
    %best configuration in terms of manipulability
    largestdeterminant = 0;
    jointiteration = determineangle(thetasolutionsover1,ur5)
    
    %solver for best angle combination here
    
    ur5.move_joints(thetasolutionsover1(1:6,jointiteration),timing);
    
    disp("Moving first position")
    pause(timing);
    
    Frame_B = tf_frame("base_link", "frame1_" + i, new_transform_over1);
    end
    %we now repeat this for the down distance
    for i = 1:stepsize
    new_transform_down = new_transform_over1 * [1,0,0,downdistance*i/stepsize;0,1,0,0;0,0,1,0;0,0,0,1];
    thetasolutionsdown = ur5InvKin(new_transform_down);
    jointiteration = determineangle(thetasolutionsdown,ur5);

    ur5.move_joints(thetasolutionsdown(1:6,jointiteration),timing);
    
    disp("Moving second position")
    pause(timing);
    
    Frame_C = tf_frame("base_link", "frame2_"+i, new_transform_down);
    end
    %and repeat again for the second over
    for i = 1:stepsize
    new_transform_over2 = new_transform_down * [1,0,0,0;0,1,0,-overdistance2*i/stepsize;0,0,1,0;0,0,0,1];
    thetasolutionsover2 = ur5InvKin(new_transform_over2);
  
  
    jointiteration = determineangle(thetasolutionsover2,ur5);
    ur5.move_joints(thetasolutionsover2(1:6,jointiteration),timing);
  
    disp("Moving third position")
    pause(timing);
    Frame_D = tf_frame("base_link", "frame3_"+i, new_transform_over2);
    end
    if(ur5.get_current_joints() == ur5InvKin(goalframe))
        done = 1;
    else
        done = 0;
    end
end
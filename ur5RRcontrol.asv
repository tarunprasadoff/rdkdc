function fin_err = ur5RRcontrol(gdesired,K,ur5)

    current_config = ur5.get_current_joints();
    pause(3);

    initial_config = [-pi/3;pi/4;-pi/2;0.2;0.15;0.05]; % initial non-singular configuration

    if double(det(ur5BodyJacobian(current_config))) > 0.001
        initial_config = current_config;
    else
        disp("Current UR5 configuration is singular");
        disp("Changing to pre-defined non-singular configuration");
    end
    
    delay = 15; % time delay between commands to ur5
    ur5.move_joints(initial_config, delay); % moving ur5 to initial configuration
    pause(delay);
    delay = .3;
    
    step = 0.1; % control resolution parameter
    gstar = gdesired; % goal configuration
    gst = ur5FwdKin(initial_config); % forward transformation for initial configuration

    e = FINV(gstar)*gst; % body velocity computation
    q_inv = double(getXi(e));
 
    v_norm = norm(q_inv(1:3)); % computing velocity magnitude
    w_norm = norm(q_inv(4:end)); % computing angular velocity magnitude
    
    max_v = 0.03; % maximum limit for v
    max_w = pi/60; % maximum limit for w
    
    q = initial_config;
    count = 1;
    
    while ( v_norm > max_v ) || ( w_norm > max_w ) % we continue controlling the ur5 till velcoity or angular velocity is significant
    
        disp(count);
        count = count + 1;
        
        jac = double(ur5BodyJacobian(q));
        jac_det = double(det(jac)); % computing the jacobian
        
        if ( jac_det < 0.001 )
        
            disp("Singular Configuration Reached: Breaking Control Loop");
            fin_err = -1;
            break;
        
        end
        
        q1 = double(q - ((K*step)*(jac\q_inv))); % Computing the next joint configuration in the control loop
        
        gst = double(ur5FwdKin(q1)); % Current UR5 transformation
        
        ur5.move_joints(q1, 2); % Moving UR5 to the current configuration of the loop
        pause(delay);

        e = double(FINV(gstar)*gst);

        q_inv = getXi(e);

        v_norm = norm(q_inv(1:3)); % Computing next velocity norm
        w_norm = norm(q_inv(4:end)); % Computing next angular velocity norm
        fin_err = [v_norm w_norm]; % Computing the error to the desired configuration as a function of residual velocity to goal

        % disp('fin_err');
        % disp(fin_err);
        
        q = q1;
    
    end


end
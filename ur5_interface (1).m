% Class ur5_interface provides Matlab API to ur_modern_driver which controls the ur5 robots through ROS

% Author: Mengze Xu, 07/24/2017
% Updated by: Zhuoqun (Ray) Zhang, 08/04/2021
% Updated by: Jakub Piwowarczyk, 08/27/2023

classdef ur5_interface < handle
    % Class used to interface with ROS topics to communicate with UR5
    % To create a robot interface
    
    % settings that are not supposed to change after constructor
    properties (SetAccess = immutable)
        speed_limit = 0.25 %0.075; % percentage of maxium speed, do not multiply by pi
        home = [0 -pi 0 -pi 0 0]'/2;  % joint states in home position [rad]
        %DH_base = [-pi 0 0 0 pi 0]/2; % init joint states in DH_base [rad]
        joint_names = {...
            'shoulder_pan_joint',...
            'shoulder_lift_joint',...
            'elbow_joint',...
            'wrist_1_joint',...
            'wrist_2_joint',...
            'wrist_3_joint'};
    end
    
    % values set by this class, can be read by others
    properties (SetAccess = protected)
        tftree                 % Last received tftree
    end
   
    % only this class methods can view/modify
    properties (SetAccess = private, Hidden = true)
        % node handle for ROS2
        node_handle

        % subscribers
        current_joint_states_sub
     
        % publishers
        trajectory_goal_pub
        urscript_pub
        velocity_goal_pub

        % service clients
        pendant_control_client
        ros_control_client
        swtich_to_pos_ctrl_client
        swtich_to_vel_ctrl_client
        
    end
    
    methods
        % constructor
        function self = ur5_interface()
            self.node_handle = ros2node("/ur5_interface");
            self.current_joint_states_sub = ros2subscriber(self.node_handle,'/joint_states','sensor_msgs/JointState','Depth',10);
            %self.trajectory_goal_pub = ros2publisher(self.node_handle,'rdkdc/joint_pos_msg','trajectory_msgs/JointTrajectory');
            self.trajectory_goal_pub = ros2publisher(self.node_handle,'/scaled_joint_trajectory_controller/joint_trajectory','trajectory_msgs/JointTrajectory');
            self.velocity_goal_pub = ros2publisher(self.node_handle,'rdkdc/joint_vel_msg','std_msgs/Float64MultiArray');
            self.urscript_pub = ros2publisher(self.node_handle,'/urscript_interface/script_command','std_msgs/String');
            self.pendant_control_client = ros2svcclient(self.node_handle,'/io_and_status_controller/hand_back_control','std_srvs/Trigger');
            self.ros_control_client = ros2svcclient(self.node_handle,'/io_and_status_controller/resend_robot_program','std_srvs/Trigger');
            self.swtich_to_pos_ctrl_client = ros2svcclient(self.node_handle, "rdkdc/swtich_to_pos_ctrl", 'std_srvs/Trigger');
            self.swtich_to_vel_ctrl_client = ros2svcclient(self.node_handle, "rdkdc/swtich_to_vel_ctrl", 'std_srvs/Trigger');
            self.tftree = ros2tf(self.node_handle);
                        
        end
        
        % update current_joint _states and return array of current joint angles
        function joint_angles = get_current_joints(self)
            current_joint_states = receive(self.current_joint_states_sub);
            joint_angles = zeros(6,1);
            for i=1:6
                for j=1:6
                    if strcmp(current_joint_states.name{i},self.joint_names{j})
                        joint_angles(j) = current_joint_states.position(i);
                    end
                end
            end
        end
        
        % get current transformation from target frame to source frame 
        function g = get_current_transformation(self,target,source)
            tran = getTransform(self.tftree,target,source);
            t = [tran.transform.translation.x, tran.transform.translation.y, tran.transform.translation.z];
            R = quat2rotm([tran.transform.rotation.w, tran.transform.rotation.x, tran.transform.rotation.y, tran.transform.rotation.z]);
            g = [R t';0 0 0 1];
        end
        
        % move in joint space
        % goal should be 6*1 vector
        function move_joints(self,joint_goal,time_interval)
            % check input
            validateattributes(joint_goal,{'numeric'},{'nrows',6,'2d'})
            validateattributes(time_interval,{'numeric'},{'nonnegative','nonzero'})
            
            trajectory_goal = ros2message('trajectory_msgs/JointTrajectory');

            % set joint names
            trajectory_goal.joint_names{1} = 'shoulder_pan_joint';
            trajectory_goal.joint_names{2} = 'shoulder_lift_joint';
            trajectory_goal.joint_names{3} = 'elbow_joint';
            trajectory_goal.joint_names{4} = 'wrist_1_joint';
            trajectory_goal.joint_names{5} = 'wrist_2_joint';
            trajectory_goal.joint_names{6} = 'wrist_3_joint';

            % set current time
            trajectory_goal.header.stamp = ros2time(self.node_handle,'now');
            
            % check speed limit
            joint_v = zeros(size(joint_goal));
            joint_v(:,1) = joint_goal(:,1)-self.get_current_joints();
            if size(joint_goal,2)>=2
                for i=2:size(joint_goal,2)
                    joint_v(:,i) = joint_goal(:,i)-joint_goal(:,i-1);
                end
            end
            if max(max(abs(joint_v)))/time_interval > self.speed_limit %pi*self.speed_limit
                error('Velocity over speed limit, please increase time_interval');
            end
            
            % set trajectory
            for i=1:size(joint_goal,2)
                trajectory_point = ros2message('trajectory_msgs/JointTrajectoryPoint');
                trajectory_point.velocities = zeros(6,1);
                trajectory_point.accelerations = zeros(6,1);
                trajectory_point.effort = [];
                trajectory_point.positions = joint_goal(:,i);
                trajectory_point.time_from_start = ros2duration(time_interval*i);
                trajectory_goal.points(i) = trajectory_point;
            end
            
            % publish the trajectory
            send(self.trajectory_goal_pub,trajectory_goal);
        end

        % % move in joint space
        % % goal should be 6*1 vector
        % function move_joints_no_safe(self,joint_goal,time_interval)
        %     % check input
        %     validateattributes(joint_goal,{'numeric'},{'size',[6,1]})
        %     validateattributes(time_interval,{'numeric'},{'nonnegative','nonzero'})
        % 
        %     trajectory_goal = ros2message('trajectory_msgs/JointTrajectory');
        % 
        %     % set joint names
        %     trajectory_goal.joint_names{1} = 'shoulder_pan_joint';
        %     trajectory_goal.joint_names{2} = 'shoulder_lift_joint';
        %     trajectory_goal.joint_names{3} = 'elbow_joint';
        %     trajectory_goal.joint_names{4} = 'wrist_1_joint';
        %     trajectory_goal.joint_names{5} = 'wrist_2_joint';
        %     trajectory_goal.joint_names{6} = 'wrist_3_joint';
        % 
        %     % set current time
        %     trajectory_goal.header.stamp = ros2time(self.node_handle,'now');
        % 
        %     % set goal
        %     trajectory_point = ros2message('trajectory_msgs/JointTrajectoryPoint');
        %     trajectory_point.positions = joint_goal;
        %     trajectory_point.velocities = zeros(6,1);
        %     trajectory_point.accelerations = zeros(6,1);
        %     trajectory_point.effort = [];
        %     trajectory_point.time_from_start = ros2duration(time_interval);
        %     trajectory_goal.points = trajectory_point;
        % 
        %     % publish the trajectory
        %     send(self.trajectory_goal_pub,trajectory_goal);
        % end

        % move in joint vel space
        % goal should be 6*1 vector
        function move_joints_vel(self,joint_vel_goal)
            % check input
            validateattributes(joint_vel_goal,{'numeric'},{'size',[6,1]})
            
            % check the speed limit
            if max(abs(joint_vel_goal)) > self.speed_limit %pi*self.speed_limit
                error('Velocity over speed limit, please decrease command');
            end

            % prepare the message
            vel_goal = ros2message('std_msgs/Float64MultiArray');
            vel_goal.data = joint_vel_goal;

            % publish the command
            send(self.velocity_goal_pub,vel_goal);
        end

        % % move in joint vel space
        % % goal should be 6*1 vector
        % function move_joints_vel_no_safe(self,joint_vel_goal)
        %     % check input
        %     validateattributes(joint_vel_goal,{'numeric'},{'size',[6,1]})
        % 
        %     % prepare the messages
        %     vel_goal = ros2message('std_msgs/Float64MultiArray');
        %     vel_goal.data = joint_vel_goal;
        % 
        %     % publish the command
        %     send(self.velocity_goal_pub,vel_goal);
        % end

        % allow students to use the freedrive button easily
        % ros2 driver prohibits pendant and ros both controlling robot
        function pendant_control_resp = swtich_to_pendant_control(self)
            pendant_control_req = ros2message(self.pendant_control_client);
            waitForServer(self.pendant_control_client,"Timeout",1);
            pendant_control_resp = call(self.pendant_control_client,pendant_control_req,"Timeout",1).success;
        end

        % swtich back to ROS control after students are done with
        % freedrive, or if they try to control the robot from the pendant
        % and sever the link
        function ros_control_resp = swtich_to_ros_control(self)
            ros_control_req = ros2message(self.ros_control_client);
            waitForServer(self.ros_control_client,"Timeout",1);
            ros_control_resp = call(self.ros_control_client,ros_control_req,"Timeout",1).success;
        end

        % enable freedrive on the robot with new URScript Interface
        % will hold freedrive until user clicks "yes" to end,
        % user will need to run "switch_to_ros_control" to use
        % matlab control again
        function enable_freedrive(self)
            urscript_msg = ros2message('std_msgs/String');
            urscript_msg.data = ['def my_prog():',newline,'freedrive_mode()',newline,'while True: my_variable=request_boolean_from_primary_client("Would you like to end FreeDrive?") if my_variable: end_freedrive_mode() break end end',newline,'end'];
            send(self.urscript_pub,urscript_msg);
        end

        % enable the scaled_joint_trajectrory_controller
        % allows students to use the move_joints command if they previously
        % switched to veclocity control
        function activate_pos_control(self)
            % stop robot before switching controllers
            self.move_joints_vel(zeros(6,1));
            pause(0.5)

            % send a request to swtich the controllers
            pos_control_req = ros2message(self.swtich_to_pos_ctrl_client);
            try
                waitForServer(self.swtich_to_pos_ctrl_client,"Timeout",1);
            catch
                error("Error: Velocity Functionality Not Enabled")
            end
            call(self.swtich_to_pos_ctrl_client,pos_control_req,"Timeout",1).success;
        end

        % enable the forward_veleocity_controller
        % allows students to use move_joints_vel
        % need to run launch script with velocity option to use this
        function activate_vel_control(self)
            % ensure robot is not moving before switching controllers
            robot_in_motion = true;
            while robot_in_motion
                jp_1 = self.get_current_joints;
                pause(0.1)
                jp_2 = self.get_current_joints;
                robot_in_motion = max(abs(jp_2-jp_1)) > 0.0005;
            end

            % send a request to switch the controllers
            vel_control_req = ros2message(self.swtich_to_vel_ctrl_client);
            try
                waitForServer(self.swtich_to_vel_ctrl_client,"Timeout",1);
            catch
                error("Error: Velocity Functionality Not Enabled")
            end
            call(self.swtich_to_vel_ctrl_client,vel_control_req,"Timeout",1).success;
        end

    end % methods
    
end % class

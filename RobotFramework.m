classdef RobotFramework
    %ROBOT Represents a general fixed-base kinematic chain.
    
    properties (SetAccess = 'immutable')
        dof
        link_lengths
        link_masses
        joint_masses
        end_effector_mass
    end
    
    methods
        % Constructor: Makes a brand new robot with the specified parameters.
        function robot = RobotFramework(link_lengths, link_masses, joint_masses, end_effector_mass)
            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(link_lengths, 2) ~= 1
               error('Invalid link_lengths: Should be a column vector, is %dx%d.', size(link_lengths, 1), size(link_lengths, 2));
            end
            
            if size(link_masses, 2) ~= 1
               error('Invalid link_masses: Should be a column vector, is %dx%d.', size(link_masses, 1), size(link_masses, 2));
            end
            
            if size(joint_masses, 2) ~= 1
               error('Invalid joint_masses: Should be a column vector.');
            end
            
            if ~isnumeric(end_effector_mass)
               error('Invalid end_effector_mass: Should be a number.'); 
            end
            
            robot.dof = size(link_lengths, 1);
            
            if size(link_masses, 1) ~= robot.dof
                error('Invalid number of link masses: should match number of link lengths.');
            end
            
            if size(joint_masses, 1) ~= robot.dof
                error('Invalid number of joint masses: should match number of link lengths. Did you forget the base joint?');
            end
            
            robot.link_lengths = link_lengths;
            robot.link_masses = link_masses;
            robot.joint_masses = joint_masses;
            robot.end_effector_mass = end_effector_mass;  
        end
       
        % Returns the forward kinematic map for each frame, one for the base of
        % each link, and one for the end effector. Link i is given by
        % frames(:,:,i), and the end effector frame is frames(:,:,end).
        function frames = forward_kinematics(robot, thetas)
            if size(thetas, 2) ~= 1
                error('Expecting a column vector of joint angles.');
            end
            
            if size(thetas, 1) ~= robot.dof
                error('Invalid number of joints: %d found, expecting %d', size(thetas, 1), robot.dof);
            end
            
            % Allocate a variable containing the transforms from each frame
            % to the base frame.
            frames = zeros(3,3,robot.dof + 1);
            n = robot.dof;
            % The transform from the base of link 'i' to the base frame (H^0_i)
            % is given by the 3x3 matrix frames(:,:,i).
            
            % The transform from the end effector to the base frame (H^0_i) is
            % given by the 3x3 matrix frames(:,:,end).

            %% FILL IN 3x3 HOMOGENEOUS TRANSFORM FOR n + 1 FRAMES
            % Create H_0_1, transformation from base frame to 1st frame
            H_0_1 = eye(3, 3); % create rotational matrix;
            % calculate angles
            c_theta = cos(thetas(1));
            s_theta = sin(thetas(1));
            % save to array
            rotations = num2cell([c_theta, -1*s_theta, s_theta, c_theta]);
            % distribute angles to respective indices
            [H_0_1(1, 1), H_0_1(1, 2), H_0_1(2, 1), H_0_1(2, 2) ...
                ] = deal(rotations{:});
            % Store first frame in transforms matrix
            frames(:, :, 1) = H_0_1;
            for i = 2:n
                H_translate = eye(3, 3); % homogenous translation matrix
                % translate along link length
                H_translate(1, 3) = robot.link_lengths(i-1);
                H_rotate = eye(3,3); % homogenous rotation matrix
                % calculate angles
                c_theta = cos(thetas(i));
                s_theta = sin(thetas(i));
                % save angles to array
                rotations = num2cell([c_theta,-1*s_theta,s_theta,c_theta]);
                % distribute angles to respective indices
                [H_rotate(1, 1), H_rotate(1, 2), H_rotate(2, 1), ...
                    H_rotate(2, 2)] = deal(rotations{:});
                % store transform from H_i-1_i in frames
                frames(:, :, i) = (frames(:, :, i-1) * H_translate) * H_rotate;
            end
            H_translate = eye(3, 3); % homogenous translation matrix
            % translate along link length
            H_translate(1, 3) = robot.link_lengths(n);
            frames(:, :, end) = frames(:, :, n) * H_translate;
        end
       
        % Shorthand for returning the forward kinematics.
        function fk = fk(robot, thetas)
            fk = robot.forward_kinematics(thetas);
        end
       
        % Returns [x; y; theta] for the end effector given a set of joint
        % angles. 
        function ee = end_effector(robot, thetas)
            % Find the transform to the end-effector frame.
            frames = robot.fk(thetas);
            H_0_ee = frames(:,:,end);
           
            % Extract the components of the end_effector position and
            % orientation.
            x = H_0_ee(1,3);
            y = H_0_ee(2,3);
            th = atan2(H_0_ee(2, 1), H_0_ee(1, 1));
           
            % Pack them up nicely.
            ee = [x; y; th];
        end
       
        % Shorthand for returning the end effector position and orientation. 
        function ee = ee(robot, thetas)
            ee = robot.end_effector(thetas);
        end
        
        function jacobians = jacobians(robot, thetas)
        % Returns the SE(2) Jacobian for each frame (as defined in the forward
        % kinematics map). Note that 'thetas' should be a column vector.

        % Make sure that all the parameters are what we're expecting.
        % This helps catch typos and other lovely bugs.
        if size(thetas, 1) ~= robot.dof || size(thetas, 2) ~= 1
           error('Invalid thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(thetas, 1), size(thetas, 2));
        end

        % Allocate a variable containing the Jacobian matrix from each frame
        % to the base frame.
        jacobians = zeros(3,robot.dof,robot.dof+1); 
        epsilon = 0.001;

        % The jacobian for frame 'i' (as defined by the forward kinematics
        % function above) is the 3xn matrix jacobians(:,:,i), where n is the
        % number of degrees of freedom.

        % In this function, compute the _numerical derivative_ for each
        % element of the Jacobian.

        % Hint: The third row of the Jacobian (the resulting frame angular
        % velocity given the joint angular velocity) is either 1 or 0,
        % depending on whether or not that joint contributes to that frame's
        % rotation.

        % Hint: jacobians(1,j,frame) is the partial differential of
        % the 'x' coordinate of 'frame' (with respect to the base frame) due
        % to motion of the joint j.  Mathematically, this is
        % d{x^0_frame}/d{theta_j}.
        %
        % To find this quantity numerically, one can compute the 'numerical
        % derivative' instead of the 'analytic derivative'.  This is
        % essentially approximating g'(x), the derivative of some function
        % g(x) at x, by computing (g(x+dx) - g(x-dx)) / (2*dx).
        %
        % Applying this to the task of finding jacobians(1,j,frame), we know
        % that:
        %   x represents theta
        %   g is the forward kinematics to this point, or the (1, 3, frame)
        %     index of the result of calling fk.
        %   dx is a vector of zeros for each joint except for joint j, and a
        %     small value for j (use the 'epsilon' defined above).
        %
        % This same logic can be used to find the y component, or
        % jacobians(2,j,frame).

        % Reminder: to get the x and y coordinates of the origin of each
        % frame, one could call the following commands (essentially pulling
        % out the translation term from the homogeneous transform to each
        % frame)
        %
        % frames = robot.fk(thetas); % Find the frames
        % frames_x = base_frames(1,3,:); % Get x values of the origins
        % frames_y = base_frames(2,3,:); % Get y values of the origins

% --------------- BEGIN STUDENT SECTION ----------------------------------
        for joint = 1 : robot.dof

            % TODO perturb the FK by 'epsilon' in this joint, and find the
            % origin of each frame.
            % Create epsilon vectors
            epsilon_vector = zeros(robot.dof, 1);
            epsilon_vector(joint) = epsilon;
            % Create base frames
            base_frames = robot.fk(thetas);
            % Create g(x+dx), g(x-dx)
            frames_high = robot.fk(thetas + epsilon_vector); % Find frames
            frames_low = robot.fk(thetas - epsilon_vector); % Find frames
            % Take numerical derivative
            frames_deriv_x = (frames_high(1,3,:)-frames_low(1,3,:))/(2*epsilon);
            frames_deriv_y = (frames_high(2,3,:)-frames_low(2,3,:))/(2*epsilon);
            
            for frame = 1 : size(base_frames,3)
                
                % TODO Fill in dx/dtheta_j for this frame
                jacobians(1, joint, frame) = frames_deriv_x(:, :, frame);

                % TODO Fill in dy/dtheta_j for this frame
                jacobians(2, joint, frame) = frames_deriv_y(:, :, frame);

                % TODO Fill in dtheta_end_effector/dtheta_j for this frame
                if frame > joint 
                    jacobians(3, joint, frame) = 1;
                else 
                    jacobians(3, joint, frame) = 0;
                end
                
            end
        end
% --------------- END STUDENT SECTION ------------------------------------
        end
        
        function thetas = inverse_kinematics(robot, initial_thetas, goal_position)
        % Returns the joint angles which minimize a simple squared-distance
        % cost function.

        % Make sure that all the parameters are what we're expecting.
        % This helps catch typos and other lovely bugs.
        if size(initial_thetas, 1) ~= robot.dof || size(initial_thetas, 2) ~= 1
            error('Invalid initial_thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(initial_thetas, 1), size(initial_thetas, 2));
        end

        if (size(goal_position, 1) ~= 2 && size(goal_position, 1) ~= 3) || ...
           size(goal_position, 2) ~= 1
            error('Invalid goal_position: Should be a 2 or 3 length column vector, is %dx%d.', size(goal_position, 1), size(goal_position, 2));
        end

        % Allocate a variable for the joint angles during the optimization;
        % begin with the initial condition
        thetas = initial_thetas;

        % Step size for gradient update
        step_size = 0.1;

        % Once the norm (magnitude) of the computed gradient is smaller than
        % this value, we stop the optimization
        stopping_condition = 0.00005;

        % Also, limit to a maximum number of iterations.
        max_iter = 1000;
        num_iter = 0;

% --------------- BEGIN STUDENT SECTION ----------------------------------     
        % Run gradient descent optimization
        alpha = 0.2;
        x_d = goal_position(1);
        y_d = goal_position(2);
        if (size(goal_position, 1) == 3)
            theta_d = goal_position(3);
        end
        while (num_iter < max_iter)

            % Compute the gradient for either an [x;y] goal or an
            % [x;y;theta] goal, using the current value of 'thetas'.
            % TODO fill in the gradient of the squared distance cost function
            % HINT use the answer for theory question 2, the
            % 'robot.end_effector' function, and the 'robot.jacobians'
            % function to help solve this problem
            jacobian = robot.jacobians(thetas);
            ee_positions = robot.ee(thetas);
            x_c = ee_positions(1);
            y_c = ee_positions(2);
            if (size(goal_position, 1) == 2) % [x;y] goal              
                dist_vector = [x_c-x_d; y_c-y_d];
                jacobian_ee_frame = jacobian(1:2,:,robot.dof+1);
                cost_gradient = -1*transpose(jacobian_ee_frame)*dist_vector;
            else % [x;y;theta] goal
                theta_c = ee_positions(3);
                dist_vector = [x_c-x_d; y_c-y_d; theta_c-theta_d];    
                jacobian_ee_frame = jacobian(:,:,robot.dof+1);
                %transpose(jacobian_ee_frame);
                cost_gradient = -1*transpose(jacobian_ee_frame)*dist_vector;
            end           

            % Update 'thetas'
            thetas = thetas + alpha*cost_gradient; 
  
            % Check stopping condition, and return if it is met.
            
            if(norm(cost_gradient) < stopping_condition)
                break 
            end

            num_iter = num_iter + 1;
        end
        
% --------------- END STUDENT SECTION ------------------------------------
        end
    
    end
end
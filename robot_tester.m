robot = Robot([1;1],[0;0],[0;0],0);
visualizer = RobotVisualizer2D(4, .25, false);
theta = theta_arr;
for i = 1:length(theta)
    
    % Actual frames
    frames = zeros(3,3,4);
    frames(:,:,1) = eye(3);
    frames(:,:,2:end) = robot.fk(theta(:,i));
    visualizer.setFrames(frames);

    
    % play through as fast as MATLAB can plot
	drawnow;
    
    % If this is too fast, you can add a pause here
     pause(0.1);
    
end

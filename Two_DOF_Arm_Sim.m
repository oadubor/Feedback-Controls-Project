%% Feedback Controls Final Project 
clc;
close all;
clear;
%% Animation
amp = 10; % amplitude of sine wave 
path_length = 10; % length of path 
plot_height = 2;
x = linspace(1,path_length); 
% generate path
path_fn = @(x)amp*sin(x); % function to create path
path = path_fn(x); % path for robot ee to follow
tframe = 0.15; % how long to wait before updating frame 
screen_width = 2; % width of plot
%initialize hold variables
x_c = -10; y_c = 0; x_do = 0; y_do = 0; x_o=0; y_o=0; 
% Initialize robot
link_lengths = [1; 1];
link_masses = [0; 0];
joint_masses = [0; 0];
ee_mass = 0;
robot = RobotFramework(link_lengths, link_masses, joint_masses, ee_mass);

%% Create EE Path
figure;
hold on
% plot circle to indicate reach limits of robot arm
radius = link_lengths(1)+link_lengths(2);
theta = 0:pi/50:2*pi;
xunit = radius * cos(theta);
yunit = radius * sin(theta);
plot(xunit, yunit);
% Create axis limits on plot
xlim([-screen_width, screen_width]);
ylim([0, plot_height]);
[x_positions,y_positions] = ginput; % grab points from figure
hold off
x_trajectory = x_positions(1):.25:x_positions(end); 
y_trajectory = spline(x_positions, y_positions, x_trajectory); % create smooth curve
%% Prepare Simulation Plot
x_goal = x_trajectory(1); % x goal position
y_goal = y_trajectory(1); % y goal position
% Create initial plot 
figure;
xlim([-screen_width, screen_width]);
ylim([-plot_height, plot_height]);
path_plot = plot(x_goal, y_goal, 'o');
visualizer = FeedbackRobotVisualizer2D(4, 0.1, 0.1, false);
title('RR Robot');

%% Iterativelty Visualize Data 
for i = 2:length(x_trajectory) % loop through path      
%for i = 1:1  
    %----------------------System Inputs----------------------------------%
    % get robot ee position
    x_goal = x_trajectory(i); % x goal position
    y_goal = y_trajectory(i); % y goal position
    goal_pos = [x_goal; y_goal]; %goal position vector
    pos_c = [x_c; y_c]; %Current position vector
    % Get Robot and path velocities
    v_gx = abs(x_goal - x_do)/tframe; % x-velocity of the goal
    v_gy = abs(y_goal - y_do)/tframe; % y-velocity of the goal
    v_g  = [v_gx; v_gy]; %Goal velocity vector
    v_x = abs(x_c - x_o)/tframe;
    v_y = abs(y_c - y_o)/tframe;
    v_c = [v_x; v_y]; %Current velocity vector
    [x_actual, y_actual] = Controller(goal_pos, pos_c, v_g, v_c);    
    x_o = x_c; y_o = y_c; %old ee position
    x_c = x_actual; y_c = y_actual; %current ee position
    x_do = x_goal; y_do = y_goal; %current goal position
    
    %----------------Transfer position info to robot----------------------%
    thetas = robot.inverse_kinematics([0; pi/4], [x_actual; y_actual]);
    %---------------------------Start Plot--------------------------------%
    % Create Robot Frames 
    frames = zeros(3,3,4);
    frames(:,:,1) = eye(3);
    frames(:,:,2:end) = robot.fk(thetas);
    % update drawings
    visualizer.setFrames(frames);    
    set(path_plot,'XData',x_goal);
    set(path_plot,'YData',y_goal);
    drawnow;           
    pause(tframe); 
    %----------------------------End Plot---------------------------------%
end
%% Controls Code Section 
%-------------------------EDIT CONTROLS CODE HERE-------------------------%
function [x,y] = Controller(goal_pos, pos_c, v_g, v_c)
    % insert controls code
    % DISCLAIMER: I put the 0.5 below so you can see both points in the 
    % animation, actual goal is to have both the points oeverlap
    x = pos_c(1); y = pos_c(2);
    x_d = goal_pos(1); y_d = goal_pos(2);
    kp = 0.9; %Proportional Control coefficient
    ki = 0; %Integral control coefficient
    kd = 0.05; %Differential control coefficient
    %error
    e_x = x_d - x;
    e_y = y_d - y;
    %velocity error
    e_vx = v_g(1) - v_c(1);
    e_vy = v_g(2) - v_c(2);
    %output
    x = x + (kp*e_x + kd*e_vx); %New value x
    y = y + (kp*e_y + kd*e_vy); %New value y
end
%-------------------------END OF CONTROLS CODE----------------------------%
%}
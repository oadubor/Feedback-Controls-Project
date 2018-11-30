%% Feedback Controls Final Project 
clc;
close all;
clear;
%% Animation
amp = 10; % amplitude of sine wave 
path_length = 10; % length of path 
x = linspace(1,path_length); 
% generate path
path_fn = @(x)amp*sin(x); % function to create path
path = path_fn(x); % path for robot ee to follow
tframe = 0.15; % how long to wait before updating frame 
screen_width = 15; % width of plot 

for i = 1:(length(x)-screen_width) % loop through path  
    % get robot ee position
    x_goal = path(i+floor(screen_width/2)); % x goal position
    y_goal = x(i+floor(screen_width/2)); % y goal position
    [x_actual, y_actual] = Controller(x_goal, y_goal);
    plot(path(i:i+screen_width), x(i:i+screen_width), ...
        x_goal, y_goal, 'o', x_actual, y_actual, 'o');
    grid on
    x_axis_buffer = 5; % how much further to push out pos/neg x limits
    xlim([-amp-x_axis_buffer, amp+x_axis_buffer]);    
    pause(tframe);    
end
%% Controls Code Section 
%-------------------------EDIT CONTROLS CODE HERE-------------------------%
function [x,y] = Controller(x_d, y_d)
    % insert controls code
    % DISCLAIMER: I put the 0.5 below so you can see both points in the 
    % animation, actual goal is to have both the points oeverlap
    x = x_d - 0.5;
    y = y_d - 0.5;
end
%-------------------------END OF CONTROLS CODE----------------------------%
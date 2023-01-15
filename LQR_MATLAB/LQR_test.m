clc
close all
clear 
global v omega d pre_error_PID_r integral_r pre_error_PID_l integral_l ul ur
global n n1 n2 n3 n4 n5 n6
%% Initial Parameters
% geometry dimension declaration
% r           = 0.04;                                           % radius of active wheel
% d           = 0.2087;                                         % distance between the center's sensor and to the middle of 2 wheels
% b           = 0.197;                                          % distance between 2 wheels
% vr          = 0.4;                                            % desired velocity
r           = 0.2;
d           = 0;                                       
b           = 0.5;                                      
vr          = 0.5;  

omega_r_ref = 0;                                              % omega right reference
omega_l_ref = 0;                                              % omega left reference

v_r_ref = 0;                                                  % velocity right and left reference
v_l_ref = 0;

v_ref   = 0;                                                  %Reference velocity for stable standard Lyapunov
omega_ref = 0;                                                
phr_ref = 0;
tsamp   = 0.001;                                              % sampling time reference
tsamp_m = 0.01;                                               % sampling time motor 
theta       = 0;                                              % assign Initial theta value                                                
theta_value = 0;                                              % save theta value
path = reference_line_map(vr,tsamp);                          % reference parameters
vr_save = ones(n+1,1)*vr;                                     % for plotting desired speed
starting_position = [path(1,1) path(2,1) path(3,1)];          % assign initial position
position = starting_position;                                 % save recent position
error = [0;                                                   % assign initial error
         0;
         0];

tracking_error_position = [error(1,1) error(2,1)];            % save error
tracking_error_angle = error(3,1);
omega_r = 0;                                                  % initial v and omega
omega_l = 0;  
omega_value = [0 0];                                          % assign recent omega value
omega_value_ref = [omega_r_ref omega_l_ref];                  % assign recent omega reference value
v_value_ref = [v_ref];                                        % save v_ref
T       = 0;                                                  % runtime                                                  
v  = 0;
v_save = [0];
omega_save = [0];
omega = path(4,1);
initial_err = 35;

%% Intial Error
% For the right motor
ur                = 0;
pre_error_PID_r   = 0;
integral_r        = 0;
% For the left motor
ul                = 0;
pre_error_PID_l   = 0;
integral_l        = 0;

%% Set up pamameters Full State Feedback
w_ref = vr / r;
A = [0 w_ref 0;
     -w_ref 0 vr;
     0 0 0];
B = [1 0;
     0 0;
     0 1];
C = [1 1 1];
D = 0;
Q = [1 0 0;
     0 40 0;
     0 0 28];
R = 0.01*[1 0;
       0 1];
K = lqr(A,B,Q,R)
k11 = K(1,1);        k21 = K(2,1);
k12 = K(1,2);        k22 = K(2,2);
k13 = K(1,3);        k23 = K(2,3);



%% Set up parameters PID for left motor
kp_l = 0.081;
ki_l = 215.46;
kd_l = 0;


%% Set up parameters PID for right motor
kp_r = 0.081;
ki_r = 215.46;
kd_r = 0;

%%
point = [];
% value = [];
theta_dot = 0;
%n is calculated in advance.
% for i = 1 : (n1+n2+n3+n4+n5+n6)
for i = 1 : n
    omega_r = motor_PID_r(omega_r_ref,tsamp_m,kp_l,ki_l,kd_l,omega_r);              % compute recent omega right
    omega_l = motor_PID_l(omega_l_ref,tsamp_m,kp_r,ki_r,kd_r,omega_l);              % compute recent omega left
    
    omega_value = [omega_value; omega_r*60/(2*pi) omega_l*60/(2*pi)];
    % rad/s = > rpm
    omega_value_ref = [omega_value_ref; omega_r_ref*60/(2*pi) omega_l_ref*60/(2*pi)];
    %M point
    v = 0.5*(omega_r+omega_l)*r;                                                  % compute recent velocity
    omega = r*(omega_r-omega_l)/b;                                                % compute recent omega
    v_save = [v_save; v];
    theta_dot = omega;
%% Finding errrors

    theta = starting_position(3);

    R = [cos(theta)  sin(theta) 0;    
         -sin(theta) cos(theta) 0;
         0           0          1];
    P = [path(1,i)-starting_position(1);
         path(2,i)-starting_position(2);
         path(3,i)-starting_position(3)];   
   
    error = R*P; 
    % generate initial errors 
    random_error = -initial_err+(initial_err-(-initial_err))*rand(1);
    random_error = random_error/100000;
    error(2) = error(2) + random_error;
    error(3) = error(3) + random_error;
    %% sensor error
    
    omegar = path(4,i);
    %%LQR
    v_cl = k11*error(1) + k12*error(2) + k13*error(3);
    w_cl = k21*error(1) + k22*error(3) + k23*error(3);
    v_ref = v_cl;                                                           % compute reference velocity
    omega_ref= w_cl;                                                        % compute reference omega           
    
    omega_r_ref = (2*v_ref+b*omega_ref)/(2*r);
    omega_l_ref = (2*v_ref-b*omega_ref)/(2*r);
    v_value_ref = [v_value_ref; v_ref];
    % rpm => rad/s
    if(omega_r_ref > 200*2*pi/60)
        omega_r_ref = 200*2*pi/60;
    end
    if(omega_r_ref < 0)
        omega_r_ref = 0;
    end
    if(omega_l_ref > 200*2*pi/60)
        omega_l_ref = 200*2*pi/60;
    end
    if(omega_l_ref < 0)
        omega_l_ref = 0;
    end

    if(abs(error(2)) > 0.02)
        point = [point; starting_position(1) starting_position(2)];
    end
    tracking_error_position = [tracking_error_position; error(1)*1000 error(2)*1000];
    tracking_error_angle    = [tracking_error_angle error(3)*180/pi];   
    
    %% Compute recent position and save value 
    xc_dot = cos(theta)*v - d*sin(theta)*theta_dot;
    yc_dot = sin(theta)*v + d*cos(theta)*theta_dot;
    thetac_dot = omega;
    
    x = starting_position(1) + xc_dot*tsamp;
    y = starting_position(2) + yc_dot*tsamp;
    theta = starting_position(3) + thetac_dot*tsamp;
   
    starting_position = [x y theta];
    T = [T; i*tsamp];
    position = [position; starting_position];
end

%%
figure('Name','Reference Omega Value From LQR','NumberTitle','off');
plot(T, omega_value_ref(:, 1), 'b', 'LineWidth', 0.5); grid on
xlabel('Time (s)'); ylabel('Wheel speed (rpm)');
hold on
plot(T, omega_value_ref(:, 2), 'r', 'LineWidth', 0.5);
xlim([0 5])
ylim([0 500])
hold off
legend('Right Wheel Ref','Left Wheel Ref');

%%
figure('Name','Response Omega Value','NumberTitle','off');
plot(T, omega_value(:, 1), 'b', 'LineWidth', 0.5);grid on
xlabel('Time (s)'); ylabel('Wheel velocity (rpm)');
hold on
plot(T, omega_value(:, 2), 'r', 'LineWidth', 0.5);
xlim([0.02 5])
ylim([0.02 500])
hold off
legend ('Right Wheel Simulated','Left Wheel Simulated');

%%
figure('Name','Error 2 tracking','NumberTitle','off');
plot(tracking_error_position(:, 2), 'g', 'LineWidth', 0.5);
legend('Error 2');
xlabel('Time (ms)');
ylabel('Error 2 Simulated(mm)');


%%
figure('Name','Velocity of the system','NumberTitle','off');
plot(T,vr_save , 'g', 'LineWidth', 0.5);grid on; hold on;
plot(T, v_value_ref, 'r', 'LineWidth', 0.5);
plot(T, v_save, 'b', 'LineWidth', 0.5);
xlabel('Time (s)'); ylabel('System velocity (m/s)');
xlim([0.02 5])
ylim([0.02 3])
hold on
legend('Desired velocity','State Feedback Reference Velocity','Simulated Velocity');


%%
figure('Name','Error 3 tracking','NumberTitle','off');
plot(T,tracking_error_angle, 'g', 'LineWidth', 0.5);
legend('Error 3');
xlabel('Time (s)');
ylabel('Error 3 Simulated (deg)');



%%
figure('Name','Path Tracking','NumberTitle','off');
plot(path(1,:),path(2,: ),'LineWidth',3);hold on; grid on;
plot(position(:,1),position(:,2 ),'r','LineWidth',1);
xlim([-2 2]);
ylim([-2 2]);
xlabel('X Coordinate (m)'); ylabel('Y Coordinate (m)');
legend('Ref Path','Simulated position');
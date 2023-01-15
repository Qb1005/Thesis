%function omega = motor_PID_l(ref,t_samp,kp_l,ki_l,kd_l,starting)
function omega = motor_PID_l(ref,t_samp,kp, ki, kd,starting)

global pre_error_PID_l  integral_l ul 
tsamp = t_samp;
t = t_samp;
x0 = starting;
X = [x0];
T = 0;
tspan= [0 tsamp];
running_time = t/tsamp;
u = ul;
PWM(1) = 0.4783*ul -15.443;
%PWM(1) = 0.3756*ul* + 5.5485;

e  = ref - x0;
integral_l = integral_l + ki*e *tsamp;
derivative = (e  - pre_error_PID_l)/tsamp;
u  = u + (kp*e  + integral_l + (kd*derivative));

PWM  = 0.449*u + 61.97;
%PWM = 0.3756*u* + 5.5485;
if(PWM  >100)                                                                       %Limit pwm
   PWM  = 100;
end
if(PWM  < 0)
   PWM  = 0;
end

[t,y] = ode45(@(t,y)1/0.032*(2.645*PWM -y),tspan,x0);
%[t,y] = ode45(@(t,y)1/0.03*(6.788*PWM - y),tspan,x0);
x0 = y(end);
omega = x0;
end
    
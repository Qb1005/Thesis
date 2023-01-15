%function omega = motor_PID_r(ref,t_samp,kp_r,ki_r,kd_r,starting)
function omega = motor_PID_r(ref,t_samp,kp, ki, kd,starting)

global pre_error_PID_r  integral_r ur 
tsamp = t_samp;
x0 = starting;
X = [x0];
T = 0;
tspan= [0 tsamp];
u = ur;

PWM(1) = 0.4951*ur -20.269;
%PWM(1) = 0.38718*ur + 6.14075;
e = ref - x0;
integral_r = integral_r + ki*e *tsamp;
derivative = (e  - pre_error_PID_r)/tsamp;
u  = u + (kp*e  + integral_r + (kd*derivative));
PWM  = 0.3859*u + 58.5878;
%PWM = 0.38718*u + 6.14075;
if(PWM  >100)                                                                       %Limit pwm
   PWM  = 100;
end
if(PWM  < 0)
   PWM  = 0;
end

[t,y] = ode45(@(t,y)1/0.03*(2.97 *PWM -y),tspan,x0);
%[t,y] = ode45(@(t,y)1/0.03*(6.586*PWM - y),tspan,x0);
x0 = y(end);
omega = x0;
end
    
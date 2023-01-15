% cảm biến 5 led

clear all
clc

duration = 5;      % th�?i gian mô ph�?ng
tc = 0.9;          % controller response time
ts = tc/3;          % sample time
runtime = duration/tc;

%% thông số

% a1 = 40;        % k/cách C -> C1
% a2 = 20;        % k/cách C -> C2
% b = 100;        % 1/2 chi�?u ngang xe
% d = 130;        % k/cách M tới C
% R = 32.5;         % b/kính bánh xe

a1 = 28;        % kho?ng cách C->C1
a2 = 14;        % kho?ng cách C -> C2
b = 175;         % 1/2 chi?u ngang xe
d = 200;        % kho?ng cách M t?i C
R = 100;         % bán kính bánh xe

robot = differentialDriveKinematics('WheelRadius',R/1000,'WheelSpeedRange',[-inf inf],'TrackWidth',2*b/1000);

vtb = 1.5;            % v trung bình, m/s
wheelspeed = vtb/(R/1000);  % rad/s
dw = 0.3*wheelspeed;        % độ chênh vận tốc tối đa giữa 2 bánh

%% giải pt vi phân

phiC1 = atan2(a1,d);
phiC2 = atan2(a2,d);

sensors = 0b00000;      % biến đ�?c sensor
last_position = 0;      % biến nhớ, 0=đúng line, -1=lệch trái, 1=lệch phải

initialState = [0 -13/1000 pi/5];
POS = [0 1000*initialState(1:2) initialState(3)];     %time x y phi

for i=1:runtime
    
    time = (i-1)*tc;
    
    %===== CHECK CẢM BIẾN =====%
    x = 1000*initialState(1);
    y = 1000*initialState(2);
    phi = initialState(3);
    
    C1 = [x+sqrt(d^2+a1^2)*cos(phi+phiC1)   y+sqrt(d^2+a1^2)*sin(phi+phiC1)];
    C4 = [x+sqrt(d^2+a1^2)*cos(phi-phiC1)   y+sqrt(d^2+a1^2)*sin(phi-phiC1)];
    C = [x+d*cos(phi)                       y+d*sin(phi)];
    C2 = [x+sqrt(d^2+a2^2)*cos(phi+phiC2)   y+sqrt(d^2+a2^2)*sin(phi+phiC2)];
    C3 = [x+sqrt(d^2+a2^2)*cos(phi-phiC2)   y+sqrt(d^2+a2^2)*sin(phi-phiC2)];
    
    s1 = check_state(C1(1),C1(2));  % trái
    s2 = check_state(C2(1),C2(2));
    s = check_state(C(1),C(2));     % giữa
    s3 = check_state(C3(1),C3(2));
    s4 = check_state(C4(1),C4(2));  % phải
    sensors = bitset(sensors,5,s1);
    sensors = bitset(sensors,4,s2);
    sensors = bitset(sensors,3,s);
    sensors = bitset(sensors,2,s3);
    sensors = bitset(sensors,1,s4);
    
    
    %===== CONTROL =====%
    [inL,inR,last_position] = controller(sensors,last_position,wheelspeed,dw);
    inputs = [inL inR];
    
    
    %===== �?ỘNG HỌC =====%
    tspan = time:ts:time+tc;
    [t,pos] = ode45(@(t,pos)derivative(robot,pos,inputs),tspan,initialState);
    x = 1000*pos(end,1);
    y = 1000*pos(end,2);
    phi = pos(end,3);
    POS = [POS;t(end) x y phi];
    initialState = pos(end,:);
    
end

plot(POS(:,2),POS(:,3),'k')
axis equal
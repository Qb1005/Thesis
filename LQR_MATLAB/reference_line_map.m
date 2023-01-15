%Line Following Robot Project New
function path = reference_line_map(vr, tsamp)

%xr,yr;                 reference coordinate
%phr;                   angle of reference section
%wr;                    angular velocity of reference section
                        
%global n n9 n8          %number of sampling times
dt = tsamp;             %sampling time, (s)

global n1 n2 n3 n4 n5 n6 n
%coordinate of section points
x0 = 1.5        ;   y0 = 0;     %starting point
x1 = x0 - 0.5   ;   y1 = y0 + 0.5;
x2 = x1 - 2     ;   y2 = y1;
x3 = x2 - 0.5   ;   y3 = y2 - 0.5;
x4 = x3 + 0.5   ;   y4 = y3 - 0.5;
x5 = x4 + 2     ;   y5 = y4;
x6 = x0         ;   y6 = y0;

Rr = 0.5; %Rr is line radius
%----1st segment-----A->B---------------------------------------
xr1(1) = x0;
yr1(1) = y0;
phr1(1) = pi/2;
wr1(1) = vr/Rr;
n1 = round(0.5 * pi * Rr / (vr * dt)) + 1;
for i = 2 : n1
   xr1(i)  =    xr1(1) - Rr * (1 - cos(vr * (i-2) * dt / Rr));
   yr1(i)  =    yr1(1) + Rr * (sin(vr * (i-2) * dt / Rr));
   phr1(i) =    phr1(i - 1) +  vr * dt / Rr;
   wr1(i)  =    wr1(i - 1);
end

%----2nd segment-----B->C---------------------------------------
xr2(1) = x1;                
yr2(1) = y1;
phr2(1) = pi;
wr2(1) = 0;
n2 = round(2 / (vr * dt)) + 1;
for i = 2 : n2
   xr2(i)  =    xr2(i - 1) - vr*dt;
   yr2(i)  =    yr2(i - 1);
   phr2(i) =    phr2(i - 1);
   wr2(i)  =    wr2(i - 1); 
end

%---3rd segment----C->D-----------------------------------------
xr3(1) = x2;                 
yr3(1) = y2;
phr3(1) = phr2(n2);               
wr3(1) = vr/Rr;
n3 = round(0.5 * pi * Rr / (vr * dt)) + 1;
for i = 2 : n3
   xr3(i)  =    xr3(1) - Rr * (sin(vr * (i-2) * dt / Rr));
   yr3(i)  =    yr3(1) - Rr * (1 - cos(vr * (i-2) * dt / Rr));
   phr3(i) =    phr3(i - 1) +  vr * dt / Rr;  
   wr3(i)  =    wr3(i - 1);
end

%---4th segment----D->E-----------------------------------------
xr4(1)  = x3;                
yr4(1) = y3;
phr4(1) = phr3(n3);      
wr4(1) = vr/Rr;
n4 = round(0.5 * pi * Rr / (vr * dt)) + 1;
for i = 2 : n4

   xr4(i)  =    xr4(1) + Rr * (1 - cos(vr * (i-2) * dt / Rr));
   yr4(i)  =    yr4(1) - Rr * (sin(vr * (i-2) * dt / Rr));
   phr4(i) =    phr4(i-1) + vr * dt / Rr;
   wr4(i)  =    wr4(i - 1);

end

%---5th segment----E->F-----------------------------------------
xr5(1) = x4;
yr5(1) = y4;
phr5(1) = phr4(n4);
wr5(1) = 0;

n5 = round(2 / (vr * dt)) + 1;
for i = 2 : n5
   xr5(i)  =    xr5(i - 1) + vr*dt;
   yr5(i)  =    yr5(i - 1);
   phr5(i) =    phr5(i - 1);
   wr5(i)  =    wr5(i - 1); 
end

%---6th segment----F->A-----------------------------------------
xr6(1) = x5;                 
yr6(1) = y5;
phr6(1) = phr5(n5);     
wr6(1) = vr/Rr;
n6 = round(0.5 * pi * Rr / (vr * dt)) + 1;
for i = 2 : n6
     xr6(i)  =    xr6(1) + Rr * (sin(vr * (i-2) * dt / Rr));
     yr6(i)  =    yr6(1) + Rr * (1-cos(vr * (i-2) * dt / Rr));
     phr6(i) =    phr6(i-1) + vr * dt / Rr;
     wr6(i)  =    wr6(i - 1); 
end

%-------------------------------------------------
xr  = [xr1,xr2,xr3,xr4,xr5,xr6];
yr  = [yr1,yr2,yr3,yr4,yr5,yr6];
phr = [phr1,phr2,phr3,phr4,phr5,phr6];
wr  = [wr1,wr2,wr3,wr4,wr5,wr6];
X0  = [x0,x1,x2,x3,x4,x5,x6];
Y0  = [y0,y1,y2,y3,y4,y5,y6];
n   = n1 + n2 + n3 + n4 + n5 + n6;
path = [xr; yr; phr; wr];
end

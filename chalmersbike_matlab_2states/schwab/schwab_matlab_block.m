% MASS

%Bike geometry
w = 1.02; % wheel base
c = 0.08; % Trail
l = pi/10; % Steer axis tilt lambda
g = 9.81; % gravity

% rear wheel
rr = 0.3;
mr = 2;
Irxx = 0.0603;
Iryy = 0.12;

% Rear Body and Frame Assembly
xb = 0.3;
zb = -0.9;
mb = 85;
Ibxx = 9.2;
Ibxz = 2.4;
Ibyy = 11;
Ibzz = 2.8;

% Front handlebar and fork assembly
xh = 0.9;
zh = -0.7;
mh = 4;
Ihxx = 0.05892; 
Ihxz = -0.00756;
Ihyy = 0.06;
Ihzz = 0.00708;

% Front Wheel
rf = 0.35;
mf = 3;
Ifxx = 0.1405;
Ifyy = 0.28;





% Åström paper parameters

a_astrm = (mb*xb + mh*xh + mf*w)/(mb + mh + mf);
b_astrm = w;
h_astrm = -(mb*zb + mh*zh + mf*rr)/(mb + mh + mf);
l_astrm = (pi/2 - l);






%% Equation Coefficients

mt = mr + mb + mh + mf;
xt = (xb * mb + xh * mh + w * mf) / mt;
zt = (-rr * mr + zb * mb + zh * mh - rf * mf) / mt;

Itxx = Irxx + Ibxx + Ihxx + Ifxx +  mr*rr^2  +  mb*zb^2  + mh*zh^2  + mf*rf^2;
Itxz = Ibxz + Ihxz - mb*xb*zb - mh*xh*zh + mf*w*rf;

Irzz = Irxx;
Ifzz = Ifxx;

Itzz = Irzz + Ibzz + Ihzz + Ifzz + mb*xb^2 + mh*xh^2 + mf*w^2;

ma = mh + mf;
xa = (xh * mh + w * mf) / ma;
za = (zh*mh - rf*mf) / ma;

Iaxx = Ihxx + Ifxx + mh*(zh - za)^2 + mf*(rf + za)^2;
Iaxz = Ihxz - mh*(xh-xa)*(zh-za) + mf*(w-xa)*(rf+za);
Iazz = Ihzz + Ifzz + mh*(xh-xa)^2 + mf*(w-xa)^2;

% l = deg2rad(70);
ua = (xa - w - c)*cos(l) - za * sin(l);

Iall = ma*ua^2 + Iaxx*sin(l)^2 + 2*Iaxz*sin(l)*cos(l) + Iazz*cos(l)^2;
Ialx = -ma*ua*za + Iaxx*sin(l) + Iaxz*cos(l);
Ialz = ma*ua*xa + Iaxz * sin(l) + Iazz * cos(l);

miu = (c/w) * cos(l);
Sr = Iryy / rr;
Sf = Ifyy / rf;
St = Sr + Sf;

Sa = ma * ua + miu * mt * xt;

M11 = Itxx;
M12 = Ialx + miu * Itxz;
M21 = M12;
M22 = Iall + 2*miu*Ialz + miu^2 * Itzz;
M = [M11 M12
     M21 M22];

K011 = mt * zt;
K012 = -Sa;
K021 = K012;
K022 = -Sa * sin(l);
K0 = [K011 K012
      K021 K022];

K211 = 0;
K212 = ( (St - mt*zt)/w )*cos(l);
K221 = 0;
K222 = ( (Sa + Sf*sin(l))/w )*cos(l);
K2 = [K211 K212
      K221 K222];
  
C111 = 0;
C112 = miu*St + Sf*cos(l) + (Itxz/w)*cos(l) - miu*mt*zt;
C121 = -(miu*St + Sf*cos(l));
C122 = (Ialz/w) * cos(l) + miu*(Sa + (Itzz/w)*cos(l));
C1 = [C111 C112
      C121 C122];
  

% ddPhi + C1(1,2)*v*ddelta + g.*K0(1,1)*phi + g.*K0(1,2)*delta + K2(1,2)*v^2 \delta - a_astrm/(b_astrm * h_astrm)*sin(l_astrm)*delta*dv ...
%     - sin(l_astrm)^2/b^2*tan(phi)*delta^2*v^2 - a_astrm * sin(l_astrm)/(b_astrm * h_astrm)*tan(phi)*v*delta*dphi = 0;


%% var needed:
% SHOULD BE REMOVED LATER！
v = 0;
dv = 0;
dphi = 0;
delta = 0;
ddelta = 0;
phi = 0;


% 
ddPhi = -(-C1(1,2)*v*ddelta + g.*K0(1,1)*phi - g.*K0(1,2)*delta - K2(1,2)*v^2*delta...
 - a_astrm/(b_astrm * h_astrm)*sin(l_astrm)*delta*dv - sin(l_astrm)^2/b_astrm^2*tan(phi)*delta^2*v^2 - a_astrm * sin(l_astrm)/(b_astrm * h_astrm)*tan(phi)*v*delta*dphi);




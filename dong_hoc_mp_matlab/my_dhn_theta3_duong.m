a2 = 32; a3 = 27; d1 = 12.6;

x = 35.7;
y = 20.6;
z = 52;

theta1 = atan2(y,x);


r = sqrt(x*x + y*y);
s = z - d1;
m = sqrt(s*s + r*r);

alpha = atan2(s,r);
gama = acos((m^2 + a2^2 - a3^2) / (2*m*a2));

theta2 = alpha - gama;
beta = acos((a2^2 +a3^2 - m^2) / (2*a2*a3));
theta3 = pi - beta;

theta1_do = theta1*180/pi
theta2_do = theta2*180/pi
theta3_do = theta3*180/pi





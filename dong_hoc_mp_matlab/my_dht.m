a2 = 0.32; a3 = 0.27; a4 = 0.0755; d1 = 0.126; d2=0.039; d3=0.669;
t0 = 0;
t1 = 0;
t2 = 0;

t0 = deg2rad(t0);
t1 = deg2rad(t1);
t2 = deg2rad(t2);

x = a2*cos(t0)*cos(t1) + a3*cos(t0)*cos(t1)*cos(t2) - a3*cos(t0)*sin(t1)*sin(t2)
y = a2*cos(t1)*sin(t0) + a3*cos(t1)*cos(t2)*sin(t0) - a3*sin(t0)*sin(t1)*sin(t2)
z = d1 + a2*sin(t1) + a3*cos(t1)*sin(t2) + a3*cos(t2)*sin(t1)
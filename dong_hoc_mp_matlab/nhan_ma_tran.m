syms t1 v1 d1;
T1 = [1 0 0 0;
      0 cos(t1) -sin(t1) v1;
      0 sin(t1) cos(t1) d1;
      0 0 0 1]

syms t2 a2 d2;
T2 = [cos(t2) -sin(t2) 0 a2;
      sin(t2) cos(t2) 0 0;
      0 0 1 -d2;
      0 0 0 1]

syms t3 a3 d3;
T3 = [cos(t3) -sin(t3) 0 a3;
      sin(t3) cos(t3) 0 0;
      0 0 1 -d3;
      0 0 0 1]

T13 = T1*T2*T3

x = T13(1, 4)
y = T13(2, 4)
z = T13(3, 4)

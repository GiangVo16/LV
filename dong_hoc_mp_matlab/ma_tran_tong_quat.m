syms a al d t
T = [cos(t), -sin(t)*cos(al), sin(t)*sin(al) , a*cos(t);
    sin(t) , cos(t)*cos(al) , -cos(t)*sin(al), a*sin(t);
    0      , sin(al)         , cos(al)        , d;
    0      , 0              , 0              , 1;];
 
a=0; al=pi/2; syms d1; d=d1; syms t1; t=t1;
T1 = subs(T);
disp('T1:'); disp(T1);

syms a2; a=a2; al=0; d=0; syms t2; t=t2;
T2 = subs(T);
disp('T2:'); disp(T2);

syms a3; a=a3; al=0; d=0; syms t3; t=t3;
T3 = subs(T);
disp('T3:'); disp(T3);

T13 = T1*T2*T3;
disp('T13:'); disp(T13);

x = T13(1, 4)
y = T13(2, 4)
z = T13(3, 4)



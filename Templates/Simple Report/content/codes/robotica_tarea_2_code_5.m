Identidad = eye(4);
syms alpha beta gamma real
BTH = rotY(gamma)*rotX(alpha)*Identidad*rotZ(beta)
alpha = deg2rad(45);
beta  = deg2rad(30);
gamma = deg2rad(60);
BTH = rotY(gamma)*rotX(alpha)*Identidad*rotZ(beta);
[O,A,T] = tr2OAT(BTH) 
[O,A,T] = tr2OAT(BTH,-1) 
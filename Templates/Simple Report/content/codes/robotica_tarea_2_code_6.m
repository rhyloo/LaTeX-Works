BTH = rotZ(deg2rad(-90))*Identidad*rotX(deg2rad(-90));
BTH_2 = rotY(gamma)*rotX(alpha)*BTH*rotZ(beta)
[O,A,T] = tr2OAT(BTH_2) 
[O,A,T] = tr2OAT(BTH_2,-1) 
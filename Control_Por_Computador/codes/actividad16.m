a = 0.00544; b = 3.456e-5;c = 0.002386;d=0.01207;
Avelocidad = [0 1;-d/b -c/b]; Bvelocidad = [0;a/b]; Dvelocidad_discreto = 0; Cvelocidad_discreto = [1 0];
A_hat = [Avelocidad zeros(2,1);-Cvelocidad_discreto 0];
B_hat = [Bvelocidad;0];
C_hat = [Cvelocidad_discreto 0];
P = [Avelocidad Bvelocidad;-Cvelocidad_discreto 0]
rank(P)
polos = [-10 -10 -40]
k_hat = acker(A_hat,B_hat,polos)
A_buclecerrado = A_hat-B_hat*k_hat
B_buclecerrado = [0 0 1]'
Gvelocidad_controlado = ss(A_buclecerrado,B_buclecerrado,C_hat,0)
eig(A_buclecerrado)
step(Gvelocidad_controlado)
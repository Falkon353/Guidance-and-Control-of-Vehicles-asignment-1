k_d = 40;
k_p = 2;
A = [zeros(3,3) eye(3);
     -k_p*eye(3) -k_d*eye(3)];
 
 
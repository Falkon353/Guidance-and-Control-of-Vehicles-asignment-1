function q_p =  quatproduct(q_2,q_1)

n_0 = (q_1(1)*q_2(1) -q_1(2)*q_2(2) -q_1(3)*q_2(3) -q_1(4)*q_2(4));
r_1 = (q_1(1)*q_2(2) +q_1(2)*q_2(1) -q_1(3)*q_2(4) +q_1(4)*q_2(3));
r_2 = (q_1(1)*q_2(3) +q_1(2)*q_2(4) +q_1(3)*q_2(1) -q_1(4)*q_2(2));
r_3 = (q_1(1)*q_2(4) -q_1(2)*q_2(3) +q_1(3)*q_2(2) +q_1(4)*q_2(1));


q_p = [n_0;r_1;r_2;r_3];
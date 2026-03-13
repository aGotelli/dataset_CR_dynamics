function M=Ad_g_m1_(R,r)


M=[R' zeros(3,3);-R'*hat_(r) R'];

function [res] = hat_SE3_(eta)
W=eta(1:3,1);
U=eta(4:end,1);
res=[[hat_(W),U];zeros(1,3),0];
end


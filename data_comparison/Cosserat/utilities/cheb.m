function [DX,X] = cheb(N,L)

if N==0 
    D=0;
    y=1;
end 
y= -cos(pi*(0:N)/N)';
c=[2;ones(N-1,1);2].*(-1).^(0:N)';
Y=repmat(y,1,N+1);
dY=Y-Y';
D = (c*(1./c)')./(dY+eye(N+1));
D = D-diag(sum(D'));

DX=(2/L)*D;
X=L*(y+1)/2;
end 
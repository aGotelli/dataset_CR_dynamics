function [DX,X] = cheb(N,L)
% cheb  Chebyshev differentiation matrix and node grid on [0,L].
%
% Standard construction (Chebyshev-Gauss-Lobatto nodes and the
% corresponding spectral differentiation matrix, e.g. Trefethen,
% "Spectral Methods in MATLAB"), rescaled from the canonical [-1,1]
% interval to [0,L]. Used in this codebase purely as a convenient way to
% generate a well-conditioned node grid (internalActuation.m,
% getCablesLength.m); DX is only used where a spectral solve is still
% performed, and is never called with N=0 in this codebase.
%
% NOTE: the N==0 branch below does not actually short-circuit (y and D
% are unconditionally overwritten immediately after it), so N=0 is not
% genuinely supported. Left exactly as in the original implementation
% since it is never exercised (N is always 29 here) -- flagged rather
% than silently changed.

if N == 0
    D = 0; %#ok<NASGU>
    y = 1; %#ok<NASGU>
end
y = -cos(pi*(0:N)/N)';
c = [2;ones(N-1,1);2].*(-1).^(0:N)';
Y = repmat(y,1,N+1);
dY = Y-Y';
D = (c*(1./c)')./(dY+eye(N+1));
D = D-diag(sum(D'));

DX = (2/L)*D;
X  = L*(y+1)/2;

end

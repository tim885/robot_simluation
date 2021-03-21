function dq = IVK_RX90(dX,T,q,dh,mode)

% dX : operational velocity set-point
% T : EE current pose
% q : current configuration
% mode : 0 means do it normally

J = getJac(q,dh,T);

% ?????%

% temporary
dq = zeros(6,1);
%

dq = (pinv(J)) * dX;

end
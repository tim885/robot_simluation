function dX = OpCorr(init_dP,init_dR,dP,dR,Kp,t);

% Return the operational velocity set point
% init_dP : initial EE position error
% init_dR : initial EE angular error
% dp : EE position error
% dA : EE angular error
% Kp : Proportional gain
% t : nb. of steps to travel from the home pose to the goal pose

dX = diag(sign([dP;dR]))*diag(sign([init_dP;init_dR]))*[init_dP;init_dR]/t + Kp*[dP;dR];

end
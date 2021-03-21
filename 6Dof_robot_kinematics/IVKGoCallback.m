function IKGoCallback(hObject, eventdata)

global STOP_IT

EEoGoalLoc = evalin('base','EEoGoal');
EEpGoalLoc = evalin('base','EEpGoal');

qLoc = evalin('base','q');
dhLoc = evalin('base','dh');
hLoc = evalin('base','h');
h_qLoc = evalin('base','h_q');
h_TLoc = evalin('base','h_T');

% Set precision reuqirements 
epsp = 0.001; % 1 mm precision for the position
epso = 0.1; % 0.1 rad precision for the angle error (see Axis/Angle representation)

% Set the Correction Gains and parameters
t =  100; % nb. of steps to travel from here to goal
Kp = 0.00 * eye(6);



% Initial conditions for the errors
dP = inf;
dR = inf;

% inverse velocity kinematics param
mode = 0; % No secondary constraint : 0

% Some trick
count = 0;
STOP_IT = 0;

n = 0;
while( (norm(dP) > epsp || norm(dR) > epso) && ~STOP_IT)
    

    [dP,dR,T] = OpError(EEpGoalLoc,EEoGoalLoc,qLoc,dhLoc);
    
    n=n+1;
    
    figure (2);
    
    subplot (3,1,1), plot (n, dP(1));
    title('dP(x)');
    hold on;
    subplot (3,1,2), plot (n, dP(2));
    title('dP(y)');
    hold on;
    subplot (3,1,3), plot (n, dP(3));
    title('dP(z)');
    hold on;
    
    
    figure (3);
    
    subplot (3,1,1), plot (n, dR(1));
    title('dR(x)');
    hold on;
    subplot (3,1,2), plot (n, dR(2));
    title('dR(y)');
    hold on;
    subplot (3,1,3), plot (n, dR(3));
    title('dR(z)');
    hold on;

    if (count == 0)
        init_dP = dP
        init_dR = dR
        count = 1;
    end
    

    dX = OpCorr(init_dP,init_dR,dP,dR,Kp,t);


    dq = IVK_RX90(dX,T,qLoc,dhLoc,mode);
    

    qLoc = simul(qLoc,dq);
    
    MvtRobot(dhLoc,hLoc,qLoc);
    
    % Update the display of the current config and current EE pose
    TLoc = modele_geom(dhLoc,qLoc);    
    updateVal(qLoc,h_qLoc,TLoc,h_TLoc);

    assignin('base','q',qLoc);
    pause(0.01);
end

end

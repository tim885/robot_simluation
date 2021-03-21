function J = getJac(q,dh,T)

%%% Temporary %%%%

J = zeros(6,6);
Je = zeros(6,6);

% ?????????????????? %
for i = 0:5;
    if i == 0 ;  
        T = modele_geom(dh(1:6,:),q(1:6));      
    else;        
        T = inv(modele_geom(dh(1:i,:),q(1:i))) * modele_geom(dh(1:6,:),q(1:6));       
    end  
    Je(:,i+1) = [(-T(2,4)*T(1,1)+T(1,4)*T(2,1));(-T(2,4)*T(1,2)+T(1,4)*T(2,2));(-T(2,4)*T(1,3)+T(1,4)*T(2,3));T(3,1);T(3,2);T(3,3)];
end


R = zeros(6,6);
Re = zeros(3,3);
T06 = modele_geom(dh(1:6,:),q(1:6));
Re = T06(1:3,1:3);

for j = 1:3;
    for k = 1:3;
        R(j,k) = Re(j,k);
        R(j+3,k+3) = Re(j,k);
    end
end

J = R * Je;


end
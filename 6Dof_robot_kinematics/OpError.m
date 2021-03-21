function [dP,dR,T] = OpError(EEp,EEo,q,dh)

% Returns the position error vector and the angular error
% also returns the EE current pose
% EEp : EE goal position
% EEo : EE goal orientation
% q : current configuration
% dh : DH parameters table


% Computes the current EE pose
T = modele_geom(dh,q);

% Position error
dP = EEp - T(1:3,4);

% Rotation matrix from current orientation to goal orientation
dR = EEo*inv(T(1:3,1:3));

% Orientation error using axis/angle representation
dA = acos((dR(1,1)+dR(2,2)+dR(3,3)-1)/2);

if (dA==0)
    uR(1) = 1.0; 
	uR(2) = 0.0;
	uR(3) = 0.0;	
elseif ((dR(1,1)+dR(2,2)+dR(3,3)-1)/2==-1) % dA = pi  
    uR(1) = sqrt((dR(1,1)+1)/2);
	uR(2) = sqrt((dR(2,2)+1)/2);
	uR(3) = sqrt((dR(3,3)+1)/2);
    % if (x is zero and y,z are not zero) invert y or z
	% else if (y is zero and z is not zero) invert x or z
	% else if (z is zero) invert x or y
	% else if (x*y , x*z and y*z are all positive) return positive values
	% else if (y*z is positive) invert x
	% else if (x*z is positive) invert y
	% else if (x*y is positive) invert z
	if (uR(1)==0 && uR(2)~=0 && uR(3)~=0) 
    	uR(2)=-uR(2);
    elseif (uR(2)==0 && uR(3)~=0)
        uR(3)=-uR(3);
    elseif (uR(3)==0)
		uR(1)=-uR(1);
    elseif (dR(1,2)>0 && dR(1,3)>0 && dR(2,3)>0)
		uR(1)=abs(uR(1));
		uR(2)=abs(uR(2));
		uR(3)=abs(uR(3));
    elseif (dR(2,3)>0)
		uR(1)=-uR(1);
    elseif (dR(1,3)>0)
		uR(2)=-uR(2);
    elseif (dR(1,2)>0)
		uR(3)=-uR(3);
    end
else
	uR(1) = 1/(2*sin(dA))*(dR(3,2)-dR(2,3)); 
	uR(2) = 1/(2*sin(dA))*(dR(1,3)-dR(3,1));
	uR(3) = 1/(2*sin(dA))*(dR(2,1)-dR(1,2));
end

dR = dA*uR';

end


%% Omni-Wheel to Spherical Wheel Speed Conversion: 
%Build the relationship of spherical wheel's velocities
%from omni-directional wheels' one.


%Just invert the allocation matrix derived in the direct mapping. 

% omega_W = r_S/r_W * M * omega_S
%  where M = [(e_Wz_1 x e_Wy_1)^T
%             (e_Wz_2 x e_Wy_2)^T  
%             (e_Wz_3 x e_Wy_3)^T];

% and it's easy to retireve omega_S as:   omega_S = r_W/r_S * (M^-1*omega_W)

function omega_S = V_3D (alpha, beta, r_S, r_W, omega_W)
%alpha is the angular displacement of the contact point with the vertical axis of the sphere wheel. 
%beta is the vector of the angual displacements of each omni-wheel qith
%respect to the x axis of the sphere wheel and those are [0, 2*pi/3,4*pi/3]
    M = zeros(3,3);
    for i=1:length(beta)
        Rot_mat = Rot_z(beta(i))*Rot_y(alpha);
        
        e_Wz = Rot_mat(:,3);
        e_Wy = Rot_mat(:,2);
        e_Wx = Rot_mat(:,1);
        
        M(i,:) = (skew(e_Wz)*e_Wy)^T;       
    end
    omega_S = r_W/r_S *(inv(M)*omega_W);
end

function Rz = Rot_z(ang)
    Rz = [cos(ang) -sen(ang) 0;
          sin(ang)  cos(ang) 0;
          0 0 1];
end

function Rx = Rot_x(ang)
    Rx = [1 0 0;
          0 cos(ang) -sen(ang);
          0 sin(ang)  cos(ang);];
end

function Ry = Rot_y(ang)
    Ry = [cos(ang),  0, sin(ang);
          0,         1, 0;
         -sin(ang), 0, cos(ang)];
end
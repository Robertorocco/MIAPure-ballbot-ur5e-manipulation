%% %% Spherical Wheel to Omni-Wheel Torque Conversion: 
%Build the relationship of omni-directional wheel's torques
%with spherical wheel's.

%Just invert the allocation matrix derived in the direct mapping: tau_S = r_S/r_W* G * tau_W 
% where each column of G is the resulting cross product u_i = e_Wz_i x e_Wy_i:
%So it is possible to invert G and put it in the form: tau_W = r_W/r_S * G^-1 * tau_S 

function tau_W = T_3D_inv (alpha, beta, r_S, r_W, tau_S)
%alpha is the angular displacement of the contact point with the vertical axis of the sphere wheel. 
%beta is the vector of the angual displacements of each omni-wheel qith
%respect to the x axis of the sphere wheel and those are [0, 2*pi/3,4*pi/3]
    G = zeros(3,3);
    
        for i=1:length(beta)
        Rot_mat = Rot_z(beta(i))*Rot_x(alpha);
        
        e_Wz = Rot_mat(:,3);
        e_Wy = Rot_mat(:,2);
        e_Wx = Rot_mat(:,1);
        
        G(:,i) = skew(e_Wz) * e_Wy;
        end
    tau_W = r_W/r_S * inv(G)*tau_S;
end

function Rz = Rot_z(ang)
    Rz = [cos(ang) -sin(ang) 0;
          sin(ang)  cos(ang) 0;
          0 0 1];
end

function Rx = Rot_x(ang)
    Rx = [1 0 0;
          0 cos(ang) -sin(ang);
          0 sin(ang)  cos(ang)];
end
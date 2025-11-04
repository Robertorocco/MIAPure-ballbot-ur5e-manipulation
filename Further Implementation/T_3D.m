%% Omni-Wheel to Spherical Wheel Torque Conversion: 
%Build the relationship of spherical wheel's torque
%from omni-directional wheels' one.

%Variables:
% - [e_Wx_i,e_Wy_i, e_Wz_i]: direction of local coordinate frame for each
% omni-wheel where e_Wy_i is the unit vector direction of the traction
% force and e_Wz_i is the unit vector normal to the sphere surface
% - r_S is the radius of the sphere 
% - r_W is the radius of the wheel
%Step-By-Step Derivation:
% 1. Traction force produced by the Omni-Wheels
%A wheel torque tau_i applied to the wheel/roller gives a tangential
%traction force Ft_i at the contact point equal to: Ft_i = tau_i/r_W * e_Wy_i

%2. Torque on the sphere about its center from the contact force
% The moment (torque) produced on the sphere by Ft_i is: tau_t_i = r_S*e_Wz_i x Ft_i
% To obtain the total torque all torque contribution from contact force
% must be summed

%By putting all together one obtains: tau_S = r_S/r_W* (tau_1*u1 + tau_2*u2 + tau_3*u3) 
% where u1,u2 and u3 are each resulting cross product u_i = e_Wz_i x e_Wy_i
%So it is possible to put it in the form: tau_S = r_S/r_W* G * tau_W 
% with:  G = [u1 u2, u3]

function tau_S = T_3D (alpha, beta, r_S, r_W, tau_W)
%alpha is the angular displacement of the contact point with the vertical axis of the sphere wheel. 
%beta is the vector of the angual displacements of each omni-wheel qith
%respect to the x axis of the sphere wheel and those are [0, 2*pi/3, 4*pi/3]
    G = zeros(3,3);
    
        for i=1:length(beta)
        Rot_mat = Rot_z(beta(i))*Rot_x(alpha);
        
        e_Wz = Rot_mat(:,3);
        e_Wy = Rot_mat(:,2);
        e_Wx = Rot_mat(:,1);
        
        G(:,i) = cross(e_Wz, e_Wy);
           
        end
    tau_S = r_S/r_W *G*tau_W;
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
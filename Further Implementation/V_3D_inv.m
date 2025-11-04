%% %% Spherical Wheel to Omni-Wheel Speed Conversion: 
%Build the relationship of omni-directional wheel's velocities
%with spherical wheel's.

%Variables:
% - [e_Wx_i,e_Wy_i, e_Wz_i]: direction of local coordinate frame for each
%omni-wheel
% - [e_Sx,e_Sy, e_Sz]: direcrion of the sphere's body frame
% - omega_S: also dot_phi angular velocity of sphere's body frame (expressed in sphere frame)

%Step-By-Step Derivation:
% 1. Contact Constraint Between Sphere and Omni-Wheel
% At the point of contact, the velocity of the sphere must match the 
% tangential velocity imparted by the omni-wheel to avoid slipping.
% To derive the speed conversion from the spherical
% wheel to the individual omni-wheels.
% 
% - e_Wz_i is the radial direction that joints the sphere center with the i-th omni-wheel origin frame. 
% - r_S*e_Wz_i is the coordinates of such vector in sphere frame 

% Then the velocity at the contact point on the sphere is:
% - v_S = omega_S x r_S*e_Wz_i, linear speed of the sphere = angular speed x radius
% 
% This velocity, projected onto the rolling direction of the omni-wheel,
% must be equal to the tangent velocity of the omi-wheel.
% 
% Each omni-wheel has a rotation which is defined by:
% 
% * Rolling direction: e_Wy_i
% * Wheel angular velocity: omega_Wi (scalar)
% * Radius of omni-wheel: r_W (scalar)
% 
% Then, the tangential velocity of the i-th omni-wheel is:
% 
% v_Wi = (omega_Wi * r_W) * e_Wy_i
% 
% Imposing that: |v_Wi| == v_S*e_Wy_i (scalar product: projection of the v_S onto the rolling
% wheel) and solving for omega_Wi we can obatin angular speed of i-th
% wheel.
% Note: |v_Wi| = omega_Wi * r_W  
%Substituting the previous equalty (==) and solving for omega_Wi one obtains:  omega_Wi = r_S/r_W * (omega_S x e_Wz_i)*e_Wy_i
%For each omni-wheel.

%Recalling the scalar triple vector product identity: 
%(a x b) * c = (b x c) * a = (c x a) *b, then:
%(omega_S x e_Wz_i)*e_Wy_i -> (e_Wz_i x e_Wy_i)*omega_S
% (here * is used in the sense of scalar product between two vectors)
% In particular one can simply write: (e_Wz_i x e_Wy_i)^T * omega_S
% (here * is used in the sense of row-columns product). 
% So the omega_W, vector containing all omni-wheel angular velocities expression becomes:
% omega_W = r_S/r_W * M * omega_S
%  where M = [(e_Wz_1 x e_Wy_1)^T
%             (e_Wz_2 x e_Wy_2)^T  
%             (e_Wz_3 x e_Wy_3)^T];

function omega_W = V_3D_inv (alpha, beta, r_S, r_W, x)
%alpha is the angular displacement of the contact point with the vertical axis of the sphere wheel. 
%beta is the vector of the angual displacements of each omni-wheel qith
%respect to the x axis of the sphere wheel and those are [0, 2*pi/3,4*pi/3]
    phi_x = x(9);
    phi_y = x(10);
    
    omega_S = [phi_x phi_y  0]';
    M = zeros(3,3);
    
        for i=1:length(beta)
        Rot_mat = Rot_z(beta(i))*Rot_y(alpha);
        
        e_Wz = Rot_mat(:,3);
        e_Wy = Rot_mat(:,2);
        e_Wx = Rot_mat(:,1);
        
        M(i,:) = (skew(e_Wz)*e_Wy)^T;    
           
        end
    omega_W = r_S/r_W * M * omega_S;
end

function Rz = Rot_z(ang)
 Rz = [cos(ang) -sen(ang) 0;
      sin(ang)  cos(ang) 0;
      0 0 1];
end

function Rx = Rot_x(ang)
 Rx = [1 0 0;
      0 cos(ang) -sen(ang);
      0 sin(ang)  cos(ang)];
end

function Ry = Rot_y(ang)
    Ry = [cos(ang),  0, sin(ang);
          0,         1, 0;
         -sin(ang), 0, cos(ang)];
end
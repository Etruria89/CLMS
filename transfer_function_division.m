% Discrete transfer function divisions
% updated:  25/07/2021
% Author: Federico Danzi
clc; clear; close all;

%--------------------------------------------------------------------------
% Input
%--------------------------------------------------------------------------

% Symbolic variable for transfer function
syms z

% Numerator and denominator of the transfer function
NTz = z-2;
DTz = z^2*(z-1)^2;


% Remember 
% Y(z)/U(z)=T(z) = Transfer function

%--------------------------------------------------------------------------
% Core
%--------------------------------------------------------------------------

% Estimate the degree of numerator and denominator
[C_DTz, T_DTz] = coeffs(DTz, z);
[C_NTz, T_NTz] = coeffs(NTz, z);
if length(T_NTz) == 1
    NTz_deg = 0
else
    NTz_deg = feval(symengine, 'degree', T_NTz(1))
end
if length(T_DTz) == 1
    DTz_deg = 0
else
    DTz_deg = feval(symengine, 'degree', T_DTz(1))
end

if DTz_deg > NTz_deg
    
    % Devide doth the terms by z at the power of Dz_deg
    Nz_divide = expand(NTz/z^DTz_deg)
    Dz_divide = expand(DTz/z^DTz_deg)
    
else
    disp("Degree of numerator higher than numerator the system is not consistent!")
end
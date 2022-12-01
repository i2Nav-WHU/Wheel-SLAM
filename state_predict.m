function particle = state_predict(particle, odom, sigmaDis_scale, sigmaPhi)

% INPUTS:
%   xv   - vehicle pose sample
%   Pv   - vehicle pose predict covariance
%   odom - dead reckoning information

% Note: Pv must be zeroed after each observation. It accumulates the
% vehicle pose uncertainty between measurements.
% particle = m_particle;
% particle.prepos = particle.xv(1:2);

% particle.pre_t = particle.t;

particle.t     = odom(1);
Dis            = odom(2);
xv             = particle.xv;

dphi           = odom(3);
phi            = dphi/2 + xv(3);
phi            = pi_to_pi(phi);


% predict state

%determine the distance uncertainty as a linear function of the distance

Q = [(sigmaDis_scale*Dis)*(sigmaDis_scale*Dis) 0;
    0 (sigmaPhi)*(sigmaPhi)];

%S    = multivariate_gauss([Dis;dphi],Q,1, particle.randstream);
S    = multivariate_gauss([Dis;dphi],Q,1);
Dis  = S(1); 
dphi = S(2);
particle.xv = [xv(1) + Dis*cos(phi); 
               xv(2) + Dis*sin(phi);
               pi_to_pi(xv(3) + dphi)];
           

function x = pi_to_pi(x)
if x > pi
    x= x - 2*pi;
elseif x < -pi
    x= x + 2*pi;
end

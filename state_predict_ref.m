function ref_state = state_predict_ref(odom, pre_state)
    t         = odom(1);
    Dis       = odom(2);
    dphi      = odom(3);
    phi       = dphi/2 + pre_state(4);
    phi       = pi_to_pi(phi);
    ref_state = [t pre_state(2) + Dis*cos(phi) pre_state(3) + Dis*sin(phi) pi_to_pi(pre_state(4) + dphi)];


function x = pi_to_pi(x)
if x > pi
    x= x - 2*pi;
elseif x < -pi
    x= x + 2*pi;
end
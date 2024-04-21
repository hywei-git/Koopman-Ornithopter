function F_tail = forces_tail(U, gamma, delta, theta, rho, St, ARt, bt) % formula from reference paper[4]
    C2 = 0.181+0.772/ARt;
    F = 1/(C2^2);
    AOA = delta - gamma - theta;
    Cl = 2*pi*F*sind(AOA);
    L = 1/2*rho*U^2*Cl*St;
    Re = U*St/bt/1.5111/10^-5;
    Cdp = 4.4*0.445/(log10(Re))^2.58;
    Cdi = Cl^2/0.8/pi/ARt;
    Cd = Cdp+Cdi;
    D = 1/2*rho*U^2*Cd*St;
    Vertical_tail = sind(delta - gamma)*D + cosd(delta - gamma)*L;
    Horizontal_tail = - cosd(delta - gamma)*D + sind(delta - gamma)*L;
    F_tail = [Vertical_tail Horizontal_tail];
end


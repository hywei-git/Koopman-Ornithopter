function F = force(f, b, c, U, phi, beta_max, rho)
    Re = U*c/1.5111/10^-5;
    Vertical = integral2(@(r, t) vertical(r, t, f, b, c, U, phi, beta_max, rho, Re), 0, b, 0, 1/f);
    Horizontal = integral2(@(r, t) horizontal(r, t, f, b, c, U, phi, beta_max, rho, Re), 0, b, 0, 1/f);
    Vertical = Vertical*2*f;
    Horizontal = Horizontal*2*f;
    F = [Vertical Horizontal];
end

% b: half of wing span (單翅長)
% c: wing chord
% f: flapping frequency
% U: forward speed
% phi: pitch angle
% beta_max: maximum flapping angle
% rho: air density

% F: [vertical force, horizontal force]

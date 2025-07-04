% 1) Load your CSV:
T = readtable('drag_curve.csv');   
v = T.speed;    % signed speeds [m/s]
D = T.drag_x;   % drag forces in X [N]

% 2) Fit a quadratic: D(v) ≈ a*v^2 + b*v + c
coeffs = polyfit(v, D, 2);   % coeffs = [a, b, c]

% 3) Your thruster max force (e.g. T500 in X)
Tmax = 336.2061;

% 4) Form the quadratic equation a*v^2 + b*v + (c – Tmax) = 0
a = coeffs(1);  b = coeffs(2);  c = coeffs(3) - Tmax;

% 5) Compute its roots:
r = roots([a, b, c]);

% 6) Keep only the real, positive root:
v_max_forward = r(imag(r)==0 & r>0);

% 7) Similarly for reverse (solve a*v^2 + b*v + (c + Tmax) = 0 for v<0):
c_rev = coeffs(3) + Tmax;
r_rev = roots([a, b, c_rev]);
v_max_reverse = r_rev(imag(r_rev)==0 & r_rev<0);

fprintf('X‐axis max: +%.2f m/s, %.2f m/s\n', v_max_forward, v_max_reverse);

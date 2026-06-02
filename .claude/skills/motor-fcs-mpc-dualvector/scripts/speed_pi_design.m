function pi = speed_pi_design(J, Pn, psif, Tsc, varargin)
%SPEED_PI_DESIGN  Symmetric-Optimum speed-PI gains for a dual-vector FCS-MPC loop.
%
%   pi = speed_pi_design(J, Pn, psif, Tsc) returns a struct with Kp, Ki and the
%   intermediate quantities, using the Symmetric Optimum (SO) tuning for a B=0
%   plant G(s) = Kt/(J*s*(T_eq*s+1)) where the inner dual-vector current loop is
%   modelled as a small lag T_eq ~ a few Tsc (near-deadbeat-fast).
%
%   Name/value options:
%     'a'      SO factor              (default 4   ; ~7% overshoot, zeta~0.71)
%     'nTeq'   T_eq = nTeq * Tsc      (default 5   ; 2..5 reasonable)
%
%   WHY SO (not pole-placement / PZC): with B=0 the plant is integrator+lag, so
%   SO places all closed-loop poles near wc = 1/(a*T_eq); there is no
%   bandwidth-independent slow pole (the -B/J trap does NOT arise here).
%   => tau_max ~ a*T_eq, settle(2%) ~ 4*a*T_eq.
%
%   Gains are in the rad/s domain. Convert BOTH speed reference and measurement
%   to rad/s before the PI (do not feed rpm), or the effective gain is off by
%   60/(2*pi) = 9.55x.
%
%   Saturate the PI output (iq_ref) to +/- iq_max with back-calculation
%   anti-windup, sized so 1.5*Pn*psif*iq_max >= 1.3*TL_max.
%
%   Example (paper SPMSM Table 1):
%     pi = speed_pi_design(0.0012, 4, 0.24, 100e-6)
%       -> Kt=1.44, T_eq=5e-4, Kp=0.4167, Ki=52.08, tau_max~2ms

p = inputParser;
p.addParameter('a', 4, @(x) isscalar(x) && x > 0);
p.addParameter('nTeq', 5, @(x) isscalar(x) && x > 0);
p.parse(varargin{:});
a    = p.Results.a;
nTeq = p.Results.nTeq;

Kt   = 1.5 * Pn * psif;            % torque constant [N*m/A]
T_eq = nTeq * Tsc;                 % inner-loop equivalent lag [s]
Kp   = J / (a   * Kt * T_eq);      % [A*s/rad]
Ki   = J / (a^3 * Kt * T_eq^2);    % [A/rad]

tau_max  = a * T_eq;               % dominant closed-loop time constant [s]
settle2  = 4 * a * T_eq;           % ~2% settling time [s]

pi = struct('Kt', Kt, 'T_eq', T_eq, 'a', a, ...
            'Kp', Kp, 'Ki', Ki, ...
            'tau_max', tau_max, 'settle2pct', settle2);

if nargout == 0
    fprintf('Kt=%.4g  T_eq=%.4g s (a=%g, nTeq=%g)\n', Kt, T_eq, a, nTeq);
    fprintf('Kp=%.4g [A*s/rad]   Ki=%.4g [A/rad]\n', Kp, Ki);
    fprintf('tau_max~%.4g s   settle(2%%)~%.4g s\n', tau_max, settle2);
    fprintf('Reminder: PI in rad/s domain; saturate iq_ref to +/- iq_max (back-calc).\n');
    clear pi
end
end

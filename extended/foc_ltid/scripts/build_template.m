function build_template()
% BUILD_TEMPLATE  One-click FOC + load-torque estimator + feed-forward PMSM model.
%
% Reusable build template for the motor-foc-ltid-pmsm skill. Builds a field-oriented
% SPEED controller for a 3-phase PMSM, augmented with a linear load-torque state estimator
% (7 selectable variants on one shared structure) whose estimate is fed forward into iq_ref.
%
% Structural modules (Rule 3): plant | outer speed PI | inner dq current PI + decoupling |
% transforms | load-torque estimator (F2) | feed-forward (F8).
%
% Estimator bank (F4-F10), selected by est_sel = 1..7:
%   1 Luenberger (pole-placed, Discrete State-Space, prediction form)
%   2 Steady-state KF      (F7, dlqe constant gain, SAME DSS realization as #1 -> F7.b)
%   3 Steady colored KF    (F7 + F9.b case B, n=3 augmented, dlqe)
%   4 Recursive KF         (F6, KF_Estimator_EML atom, n=2)            <- default
%   5 Recursive colored KF (F6 + F9.b case B, KF_Estimator_EML atom, n=3)
%   6 Correlated-noise KF  (F9.a, inline MATLAB Function, cross-term M)
%   7 Continuous Kalman-Bucy (F10, inline MATLAB Function + continuous Integrator)
%
% Plant, selected by plant_sel:
%   1 equation-linear dq integrator chain        (clean mechanical dynamics for estimator study)
%   2 Simscape switched plant (base atoms by pointer: PMSM, Universal_Bridge, SVPWM, DC, powergui)
%
% ONE-CLICK: every numeric originates in the model InitFcn below (incl. plant_sel/est_sel/
% ff_enable guarded defaults), so the saved .slx runs from a double-click in a fresh session.
% Idempotent: tears down + rebuilds + runs a FF-on/off demo. Uses ONLY shared/* assets.
%
% >>> TO RETARGET FOR YOUR MACHINE: edit the InitFcn parameter block (marked PARAMETER SURFACE).
%     All defaults are generic round-number SPMSM values and illustrative estimator tunings;
%     replace them with your machine + your design choices. Topology stays the same.

mdl  = 'foc_ltid_template';
here = fileparts(mfilename('fullpath'));
% repo root = nearest ancestor holding shared/building_blocks/pmsm_blocks.slx (base atoms, by pointer)
root = here;
while ~isfile(fullfile(root,'shared','building_blocks','pmsm_blocks.slx'))
    p = fileparts(root);
    if strcmp(p,root), error('repo root (shared/building_blocks/pmsm_blocks.slx) not found above %s', here); end
    root = p;
end
% method root = nearest ancestor holding building_blocks/foc_ltid_blocks.slx (method-local KF atom)
mroot = here;
while ~isfile(fullfile(mroot,'building_blocks','foc_ltid_blocks.slx'))
    p = fileparts(mroot);
    if strcmp(p,mroot), error('building_blocks/foc_ltid_blocks.slx not found above %s', here); end
    mroot = p;
end
pmsm_lib = fullfile(root,'shared','building_blocks','pmsm_blocks.slx');
kf_lib   = fullfile(mroot,'building_blocks','foc_ltid_blocks.slx');
assert(isfile(pmsm_lib), 'missing %s', pmsm_lib);
assert(isfile(kf_lib),   'missing %s', kf_lib);

if bdIsLoaded(mdl), close_system(mdl,0); end
new_system(mdl);
load_system(pmsm_lib); load_system(kf_lib);

L = @(a,b) add_line(mdl,a,b,'autorouting','on');

% =================================================================== InitFcn = PARAMETER SURFACE
% Edit this block for your machine / design choices. Everything downstream is topology, not numbers.
initfcn = strjoin({ ...
 '% ===== machine (generic SPMSM defaults; REPLACE for your motor) ====='
 'Rs=0.5; Ld=5e-3; Lq=5e-3; psif=0.15; pn=4; J=1e-3; B=5e-4;'
 'Kt=1.5*pn*psif;                       % torque constant, consistent with EQN plant Te=1.5*pn*psif*iq'
 'ts=1e-4;                              % controller/estimator discrete period'
 '% ===== inner dq current PI (F1.b IMC: Kp=ac*L, Ki=ac*Rs) + back-EMF decoupling (F1.c) ====='
 'alpha_c=2000;                         % current-loop bandwidth (keep << 2*pi/ts)'
 'Kp_d=alpha_c*Ld; Ki_d=alpha_c*Rs; Kp_q=alpha_c*Lq; Ki_q=alpha_c*Rs;'
 'Vmax=300;                             % dq voltage saturation'
 '% ===== outer speed PI (Symmetric Optimum; see Rule 8 / Phase-4.5 trap) + back-calc anti-windup ='
 'a_so=4; Teq=1/alpha_c;'
 'Kp_w=J/(a_so*Teq*Kt); Ki_w=J/(a_so^3*Teq^2*Kt);'
 'tau_i_w=Kp_w/Ki_w; Kb=1/tau_i_w;'
 'iq_max=10;                            % iq_ref / iq_fb saturation'
 '% ===== scenario ====='
 'wref_val=100; t_ramp=0.01;            % speed reference (rad/s mech) + ramp time'
 'tL_on=0.15; tL_off=0.30; TL_val=2; StopTime=0.45;   % load pulse 0->TL_val->0'
 '% ===== speed-sensor noise (fixed seed; the KF''s reason to exist) ====='
 'noise_var=0.25; noise_seed=12345; ts_noise=ts/2;'
 '% ===== estimator design (all computed here -> one-click) ====='
 '% F2 augmented model x=[wm;TL], u=Te, y=wm (n=2), random-walk TL (TLdot~0)'
 'Ac=[-B/J -1/J; 0 0]; Bc=[1/J;0]; Cc=[1 0]; Dc=0; Gc=[0;1];'
 'Ob=[Cc; Cc*Ac]; assert(rank(Ob)==2, ''F3 observability rank-deficient'');'
 'sysd=c2d(ss(Ac,Bc,Cc,Dc),ts,''zoh''); Ad=sysd.A; Bd=sysd.B; Cd=sysd.C; Dd=0;'
 '% --- variant 1: pole-placed Luenberger (F4/F5), prediction-form gain (obs 3-5x loop) ---'
 'po_c=[-1500 -2500]; zo=exp(po_c*ts); Lp1=place(Ad'',Cd'',zo)'';'
 'Aobs1=Ad-Lp1*Cd; Bobs1=[Bd Lp1]; Cobs1=eye(2); Dobs1=zeros(2,2); X01=[0;0];'
 '% --- variant 4: recursive KF (F6) tuning knobs ---'
 'Rd=noise_var; Qd=diag([0.01 0.1]);   % R=honest injected variance; Q(2,2)=TL random-walk knob'
 '% --- variant 2: steady-state KF (F7, DARE via dlqe) in the SAME prediction-form DSS (F7.b) ---'
 'M2=dlqe(Ad,eye(2),Cd,Qd,Rd); Lp2=Ad*M2;'
 'Aobs2=Ad-Lp2*Cd; Bobs2=[Bd Lp2]; Cobs2=eye(2); Dobs2=zeros(2,2); X02=[0;0];'
 '% --- colored model (F9.b case B): append 1st-order Gauss-Markov MEASUREMENT-noise state n ---'
 '%     keeps TL=state 2 (the atom hardcodes est_load=xhat(2)); a_col->0 recovers F2.'
 'a_col=2000;'
 'Ac3=[-B/J -1/J 0; 0 0 0; 0 0 -a_col]; Bc3=[1/J;0;0]; Cc3=[1 0 1]; Dc3=0;'
 'sysd3=c2d(ss(Ac3,Bc3,Cc3,Dc3),ts,''zoh''); Ad3=sysd3.A; Bd3=sysd3.B; Cd3=sysd3.C; Dd3=0;'
 'Q3=diag([0.01 0.1 0.05]); R3=noise_var*0.1;'
 '% --- variant 3: steady colored KF (F7 on the n=3 model) ---'
 'M3=dlqe(Ad3,eye(3),Cd3,Q3,R3); Lp3=Ad3*M3;'
 'Aobs3=Ad3-Lp3*Cd3; Bobs3=[Bd3 Lp3]; Cobs3=eye(3); Dobs3=zeros(3,2); X03=[0;0;0];'
 '% --- variant 6: correlated process/measurement-noise KF (F9.a) cross-term M (illustrative) ---'
 'Mcorr=[0; 0.005];                     % reduce toward 0 to recover #4-class behaviour'
 '% --- variant 7: continuous Kalman-Bucy (F10) intensities via F10.a discrete<->cont bridge ---'
 'Rc=Rd*ts; Qc=Qd(2,2)/ts;'
 '% ===== Simscape switched-plant numerics (used only when plant_sel==2) ====='
 'flux=psif; Pn=pn; F=B;'
 'Vdc_val=400; Tpwm=1e-4; Ts=1e-6;'
 '% ===== run-time selectors (guarded so external overrides survive InitFcn) ====='
 'if ~exist(''ff_enable'',''var''); ff_enable=1; end'
 'if ~exist(''est_sel'',''var''); est_sel=4; end'
 'if ~exist(''plant_sel'',''var''); plant_sel=1; end'
 }, newline);
set_param(mdl,'InitFcn',initfcn);

set_param(mdl,'SolverType','Variable-step','Solver','ode23tb', ...
    'StopTime','StopTime','MaxStep','ts/2','SaveOutput','on','SaveFormat','Dataset');

% =================================================================== scenario sources (top)
add('simulink/Sources/From Workspace','wref_src', G(0,0));
set_param([mdl '/wref_src'], ...
    'VariableName','[0 0; t_ramp wref_val; StopTime wref_val]', ...
    'Interpolate','on','SampleTime','0','OutputAfterFinalValue','Holding final value');
add('simulink/Sources/From Workspace','TL_src', G(0,7));
set_param([mdl '/TL_src'], ...
    'VariableName','[0 0; tL_on 0; tL_on TL_val; tL_off TL_val; tL_off 0; StopTime 0]', ...
    'Interpolate','on','SampleTime','0','OutputAfterFinalValue','Holding final value');

% =================================================================== PLANT (Variant Subsystem)
build_plant_variant_subsystem(mdl, pmsm_lib);
% Plant external interface:  In 1=Vd 2=Vq 3=TL  |  Out 1=id 2=iq 3=wm 4=Te 5=ia 6=ib 7=ic

% =================================================================== speed-sensor noise
add('simulink/Sources/Random Number','Noise_w', G(2,3));
set_param([mdl '/Noise_w'],'Mean','0','Variance','noise_var','Seed','noise_seed','SampleTime','ts_noise');
add('simulink/Math Operations/Add','Sum_wmeas', G(3,3,30,40),'Inputs','++');
L('Plant/3','Sum_wmeas/1'); L('Noise_w/1','Sum_wmeas/2');      % wm_meas = wm + noise

% =================================================================== outer speed PI (SO) + anti-windup
add('simulink/Math Operations/Add','Sum_ew', G(2,4,30,40),'Inputs','+-');
L('wref_src/1','Sum_ew/1'); L('Sum_wmeas/1','Sum_ew/2');
add('simulink/Math Operations/Gain','Gain_Kpw', G(3,4),'Gain','Kp_w');
L('Sum_ew/1','Gain_Kpw/1');
add('simulink/Math Operations/Gain','Gain_Kiw', G(3,5),'Gain','Ki_w');
L('Sum_ew/1','Gain_Kiw/1');
add('simulink/Discrete/Memory','Memory_aw', G(3,6),'X0','0');
add('simulink/Math Operations/Gain','Gain_Kb', G(2,6),'Gain','Kb');
L('Memory_aw/1','Gain_Kb/1');
add('simulink/Math Operations/Add','Sum_iin', G(4,5,30,40),'Inputs','++');
L('Gain_Kiw/1','Sum_iin/1'); L('Gain_Kb/1','Sum_iin/2');
add('simulink/Discrete/Discrete-Time Integrator','DInt_w', G(5,5), ...
    'gainval','1','SampleTime','ts','InitialCondition','0','IntegratorMethod','Integration: Forward Euler');
L('Sum_iin/1','DInt_w/1');
add('simulink/Math Operations/Add','Sum_upre', G(6,4,30,40),'Inputs','++');
L('Gain_Kpw/1','Sum_upre/1'); L('DInt_w/1','Sum_upre/2');
add('simulink/Discontinuities/Saturation','Sat_iqfb', G(7,4),'UpperLimit','iq_max','LowerLimit','-iq_max');
L('Sum_upre/1','Sat_iqfb/1');
add('simulink/Math Operations/Add','Sum_aw', G(7,5,30,40),'Inputs','+-');
L('Sat_iqfb/1','Sum_aw/1'); L('Sum_upre/1','Sum_aw/2');
L('Sum_aw/1','Memory_aw/1');

% =================================================================== estimator bank (7 variants)
add('simulink/Signal Routing/Mux','Mux_est_in', G(9,7,30,50),'Inputs','2');
L('Plant/4','Mux_est_in/1'); L('Sum_wmeas/1','Mux_est_in/2');   % DSS input order [u(Te); y(wm_meas)]
add('simulink/Discrete/Zero-Order Hold','ZOH_y', G(8,9,40,30),'SampleTime','ts');
add('simulink/Discrete/Zero-Order Hold','ZOH_u', G(8,10,40,30),'SampleTime','ts');
L('Sum_wmeas/1','ZOH_y/1'); L('Plant/4','ZOH_u/1');

% ---- variant 1: pole-placed Luenberger (Discrete State-Space) ----
add('simulink/Discrete/Discrete State-Space','Lobs1', G(10,7,120,60), ...
    'A','Aobs1','B','Bobs1','C','Cobs1','D','Dobs1','X0','X01','SampleTime','ts');
L('Mux_est_in/1','Lobs1/1');
add('simulink/Signal Routing/Demux','Demux1', G(12,7,30,50),'Outputs','2');
L('Lobs1/1','Demux1/1');                                         % Demux1/2 = T_hat_L (var1)

% ---- variant 2: steady-state KF (Discrete State-Space, dlqe gain) ----
add('simulink/Discrete/Discrete State-Space','Lobs2', G(10,8,120,60), ...
    'A','Aobs2','B','Bobs2','C','Cobs2','D','Dobs2','X0','X02','SampleTime','ts');
L('Mux_est_in/1','Lobs2/1');
add('simulink/Signal Routing/Demux','Demux2', G(12,8,30,50),'Outputs','2');
L('Lobs2/1','Demux2/1');                                         % Demux2/2 = T_hat_L (var2)

% ---- variant 3: steady colored KF (n=3 Discrete State-Space) ----
add('simulink/Discrete/Discrete State-Space','Lobs3', G(10,9,120,70), ...
    'A','Aobs3','B','Bobs3','C','Cobs3','D','Dobs3','X0','X03','SampleTime','ts');
L('Mux_est_in/1','Lobs3/1');
add('simulink/Signal Routing/Demux','Demux3', G(12,9,30,70),'Outputs','3');
L('Lobs3/1','Demux3/1');                                         % Demux3/2 = T_hat_L (var3)

% ---- variant 4: recursive KF atom (n=2) ----
add_block('foc_ltid_blocks/KF_Estimator_EML', [mdl '/KF4'], 'Position', G(10,10,140,120));
pin_kf_atom([mdl '/KF4'], 2);
add('simulink/Sources/Constant','C4_Ad', G(9,11,40,24),'Value','Ad','SampleTime','ts');
add('simulink/Sources/Constant','C4_Bd', G(9,12,40,24),'Value','Bd','SampleTime','ts');
add('simulink/Sources/Constant','C4_Cd', G(9,13,40,24),'Value','Cd','SampleTime','ts');
add('simulink/Sources/Constant','C4_Dd', G(9,14,40,24),'Value','Dd','SampleTime','ts');
add('simulink/Sources/Constant','C4_Qd', G(9,15,40,24),'Value','Qd','SampleTime','ts');
add('simulink/Sources/Constant','C4_Rd', G(9,16,40,24),'Value','Rd','SampleTime','ts');
L('ZOH_y/1','KF4/1'); L('ZOH_u/1','KF4/2');
L('C4_Ad/1','KF4/3'); L('C4_Bd/1','KF4/4'); L('C4_Cd/1','KF4/5');
L('C4_Dd/1','KF4/6'); L('C4_Qd/1','KF4/7'); L('C4_Rd/1','KF4/8');   % KF4/1 = est_load (var4)

% ---- variant 5: recursive colored KF atom (n=3) ----
add_block('foc_ltid_blocks/KF_Estimator_EML', [mdl '/KF5'], 'Position', G(13,10,140,120));
pin_kf_atom([mdl '/KF5'], 3);
add('simulink/Sources/Constant','C5_Ad', G(12,11,40,24),'Value','Ad3','SampleTime','ts');
add('simulink/Sources/Constant','C5_Bd', G(12,12,40,24),'Value','Bd3','SampleTime','ts');
add('simulink/Sources/Constant','C5_Cd', G(12,13,40,24),'Value','Cd3','SampleTime','ts');
add('simulink/Sources/Constant','C5_Dd', G(12,14,40,24),'Value','Dd3','SampleTime','ts');
add('simulink/Sources/Constant','C5_Qd', G(12,15,40,24),'Value','Q3','SampleTime','ts');
add('simulink/Sources/Constant','C5_Rd', G(12,16,40,24),'Value','R3','SampleTime','ts');
L('ZOH_y/1','KF5/1'); L('ZOH_u/1','KF5/2');
L('C5_Ad/1','KF5/3'); L('C5_Bd/1','KF5/4'); L('C5_Cd/1','KF5/5');
L('C5_Dd/1','KF5/6'); L('C5_Qd/1','KF5/7'); L('C5_Rd/1','KF5/8');   % KF5/1 = est_load (var5)

% ---- variant 6: correlated-noise KF (F9.a) inline MATLAB Function ----
add('simulink/User-Defined Functions/MATLAB Function','KF6', G(16,7,150,110));
set_kf6_script([mdl '/KF6']);
add('simulink/Sources/Constant','C6_Ad', G(15,9,40,24),'Value','Ad','SampleTime','ts');
add('simulink/Sources/Constant','C6_Bd', G(15,10,40,24),'Value','Bd','SampleTime','ts');
add('simulink/Sources/Constant','C6_Cd', G(15,11,40,24),'Value','Cd','SampleTime','ts');
add('simulink/Sources/Constant','C6_Dd', G(15,12,40,24),'Value','Dd','SampleTime','ts');
add('simulink/Sources/Constant','C6_Qd', G(15,13,40,24),'Value','Qd','SampleTime','ts');
add('simulink/Sources/Constant','C6_Rd', G(15,14,40,24),'Value','Rd','SampleTime','ts');
add('simulink/Sources/Constant','C6_M',  G(15,15,40,24),'Value','Mcorr','SampleTime','ts');
L('ZOH_y/1','KF6/1'); L('ZOH_u/1','KF6/2');
L('C6_Ad/1','KF6/3'); L('C6_Bd/1','KF6/4'); L('C6_Cd/1','KF6/5');
L('C6_Dd/1','KF6/6'); L('C6_Qd/1','KF6/7'); L('C6_Rd/1','KF6/8'); L('C6_M/1','KF6/9'); % KF6/1 = est_load

% ---- variant 7: continuous Kalman-Bucy (F10) inline MATLAB Function + continuous Integrator ----
add('simulink/User-Defined Functions/MATLAB Function','KF7', G(16,12,150,110));
set_kf7_script([mdl '/KF7']);
add('simulink/Continuous/Integrator','Int_KB', G(18,12,40,60), ...
    'InitialCondition','[0;0;1;0;1]');                          % z0 = [xhat1;xhat2;P11;P12;P22]
add('simulink/Sources/Constant','C7_Ac', G(15,14,40,24),'Value','Ac','SampleTime','0');
add('simulink/Sources/Constant','C7_Bc', G(15,15,40,24),'Value','Bc','SampleTime','0');
add('simulink/Sources/Constant','C7_Cc', G(15,16,40,24),'Value','Cc','SampleTime','0');
add('simulink/Sources/Constant','C7_Gc', G(15,17,40,24),'Value','Gc','SampleTime','0');
add('simulink/Sources/Constant','C7_Qc', G(15,18,40,24),'Value','Qc','SampleTime','0');
add('simulink/Sources/Constant','C7_Rc', G(15,19,40,24),'Value','Rc','SampleTime','0');
L('Sum_wmeas/1','KF7/1'); L('Plant/4','KF7/2');                 % continuous y, u (no ZOH)
L('C7_Ac/1','KF7/3'); L('C7_Bc/1','KF7/4'); L('C7_Cc/1','KF7/5');
L('C7_Gc/1','KF7/6'); L('C7_Qc/1','KF7/7'); L('C7_Rc/1','KF7/8');
L('Int_KB/1','KF7/9');                                          % z fed back
L('KF7/1','Int_KB/1');                                          % zdot -> integrator
add('simulink/User-Defined Functions/MATLAB Function','KB_pick', G(20,12,90,40));
set_kbpick_script([mdl '/KB_pick']);
L('Int_KB/1','KB_pick/1');                                      % KB_pick/1 = T_hat_L (var7)

% =================================================================== estimator selector + FF (F8)
add('simulink/Sources/Constant','Const_estsel', G(14,4,60,24),'Value','est_sel','SampleTime','inf');
add('simulink/Signal Routing/Multiport Switch','MSwitch_est', G(15,4,30,200),'Inputs','7');
L('Const_estsel/1','MSwitch_est/1');
L('Demux1/2','MSwitch_est/2');     % var1
L('Demux2/2','MSwitch_est/3');     % var2
L('Demux3/2','MSwitch_est/4');     % var3
L('KF4/1','MSwitch_est/5');        % var4
L('KF5/1','MSwitch_est/6');        % var5
L('KF6/1','MSwitch_est/7');        % var6
L('KB_pick/1','MSwitch_est/8');    % var7
add('simulink/Math Operations/Gain','Gain_invKt', G(16,4),'Gain','1/Kt');
L('MSwitch_est/1','Gain_invKt/1');
add('simulink/Sources/Constant','Const_ffen', G(16,5,60,24),'Value','ff_enable','SampleTime','inf');
add('simulink/Math Operations/Product','Prod_ff', G(17,4,30,40),'Inputs','2');
L('Gain_invKt/1','Prod_ff/1'); L('Const_ffen/1','Prod_ff/2');
add('simulink/Math Operations/Add','Sum_iqref', G(8,4,30,40),'Inputs','++');
L('Sat_iqfb/1','Sum_iqref/1'); L('Prod_ff/1','Sum_iqref/2');
add('simulink/Discontinuities/Saturation','Sat_iqref', G(9,4),'UpperLimit','iq_max','LowerLimit','-iq_max');
L('Sum_iqref/1','Sat_iqref/1');

% =================================================================== inner dq current PI + decoupling
add('simulink/Math Operations/Gain','Gain_we', G(2,2),'Gain','pn');    % we = pn*wm
L('Plant/3','Gain_we/1');
% d-axis (id_ref = 0)
add('simulink/Sources/Constant','Const_idref', G(0,1,60,24),'Value','0','SampleTime','inf');
add('simulink/Math Operations/Add','Sum_ed', G(1,1,30,40),'Inputs','+-');
L('Const_idref/1','Sum_ed/1'); L('Plant/1','Sum_ed/2');
add('simulink/Math Operations/Gain','Gain_Kpd', G(0,2),'Gain','Kp_d');
L('Sum_ed/1','Gain_Kpd/1');
add('simulink/Discrete/Discrete-Time Integrator','DInt_d', G(1,2), ...
    'gainval','Ki_d','SampleTime','ts','InitialCondition','0','IntegratorMethod','Integration: Forward Euler', ...
    'LimitOutput','on','UpperSaturationLimit','Vmax','LowerSaturationLimit','-Vmax');
L('Sum_ed/1','DInt_d/1');
add('simulink/Math Operations/Add','Sum_VdPI', G(2,1,30,40),'Inputs','++');
L('Gain_Kpd/1','Sum_VdPI/1'); L('DInt_d/1','Sum_VdPI/2');
add('simulink/Signal Routing/Mux','Mux_decd', G(0,3,30,50),'Inputs','2');
add('simulink/User-Defined Functions/Fcn','Fcn_decd', G(1,3),'Expr','-Lq*u(1)*u(2)');
L('Gain_we/1','Mux_decd/1'); L('Plant/2','Mux_decd/2'); L('Mux_decd/1','Fcn_decd/1');
add('simulink/Math Operations/Add','Sum_Vd', G(3,1,30,40),'Inputs','++');
L('Sum_VdPI/1','Sum_Vd/1'); L('Fcn_decd/1','Sum_Vd/2');
add('simulink/Discontinuities/Saturation','Sat_Vd', G(4,1),'UpperLimit','Vmax','LowerLimit','-Vmax');
L('Sum_Vd/1','Sat_Vd/1');
L('Sat_Vd/1','Plant/1');                                        % -> plant Vd
% q-axis
add('simulink/Math Operations/Add','Sum_eq', G(10,4,30,40),'Inputs','+-');
L('Sat_iqref/1','Sum_eq/1'); L('Plant/2','Sum_eq/2');
add('simulink/Math Operations/Gain','Gain_Kpq', G(10,3),'Gain','Kp_q');
L('Sum_eq/1','Gain_Kpq/1');
add('simulink/Discrete/Discrete-Time Integrator','DInt_q', G(11,4), ...
    'gainval','Ki_q','SampleTime','ts','InitialCondition','0','IntegratorMethod','Integration: Forward Euler', ...
    'LimitOutput','on','UpperSaturationLimit','Vmax','LowerSaturationLimit','-Vmax');
L('Sum_eq/1','DInt_q/1');
add('simulink/Math Operations/Add','Sum_VqPI', G(12,4,30,40),'Inputs','++');
L('Gain_Kpq/1','Sum_VqPI/1'); L('DInt_q/1','Sum_VqPI/2');
add('simulink/Signal Routing/Mux','Mux_decq', G(10,2,30,50),'Inputs','2');
add('simulink/User-Defined Functions/Fcn','Fcn_decq', G(11,2),'Expr','Ld*u(1)*u(2)+psif*u(1)');
L('Gain_we/1','Mux_decq/1'); L('Plant/1','Mux_decq/2'); L('Mux_decq/1','Fcn_decq/1');
add('simulink/Math Operations/Add','Sum_Vq', G(13,3,30,40),'Inputs','++');
L('Sum_VqPI/1','Sum_Vq/1'); L('Fcn_decq/1','Sum_Vq/2');
add('simulink/Discontinuities/Saturation','Sat_Vq', G(14,3),'UpperLimit','Vmax','LowerLimit','-Vmax');
L('Sum_Vq/1','Sat_Vq/1');
L('Sat_Vq/1','Plant/2');                                        % -> plant Vq
L('TL_src/1','Plant/3');                                        % -> plant TL

% =================================================================== logging (To Workspace, Array @ ts)
add('simulink/Signal Routing/Mux','MxSpeed',[2200 40 2230 160],'Inputs','3');
L('wref_src/1','MxSpeed/1'); L('Plant/3','MxSpeed/2'); L('Sum_wmeas/1','MxSpeed/3');
add('simulink/Sinks/To Workspace','tw_speed',[2260 80 2340 120],'VariableName','log_speed','SaveFormat','Array','SampleTime','ts');
L('MxSpeed/1','tw_speed/1');
add('simulink/Signal Routing/Mux','MxTL',[2200 220 2230 540],'Inputs','8');
L('TL_src/1','MxTL/1');
L('Demux1/2','MxTL/2'); L('Demux2/2','MxTL/3'); L('Demux3/2','MxTL/4');
L('KF4/1','MxTL/5'); L('KF5/1','MxTL/6'); L('KF6/1','MxTL/7'); L('KB_pick/1','MxTL/8');
add('simulink/Sinks/To Workspace','tw_TL',[2260 360 2340 400],'VariableName','log_TLhat','SaveFormat','Array','SampleTime','ts');
L('MxTL/1','tw_TL/1');
add('simulink/Signal Routing/Mux','MxCur',[2200 580 2230 680],'Inputs','3');
L('Sat_iqref/1','MxCur/1'); L('Plant/2','MxCur/2'); L('Plant/1','MxCur/3');
add('simulink/Sinks/To Workspace','tw_cur',[2260 600 2340 640],'VariableName','log_cur','SaveFormat','Array','SampleTime','ts');
L('MxCur/1','tw_cur/1');
add('simulink/Sinks/To Workspace','tw_trq',[2260 700 2340 740],'VariableName','log_trq','SaveFormat','Array','SampleTime','ts');
L('Plant/4','tw_trq/1');
add('simulink/Signal Routing/Mux','MxIabc',[2200 760 2230 860],'Inputs','3');
L('Plant/5','MxIabc/1'); L('Plant/6','MxIabc/2'); L('Plant/7','MxIabc/3');
add('simulink/Sinks/To Workspace','tw_iabc',[2260 780 2340 820],'VariableName','log_iabc','SaveFormat','Array','SampleTime','ts');
L('MxIabc/1','tw_iabc/1');

% =================================================================== theta_e Goto self-test (Rule 4)
gblks = find_system(mdl,'LookUnderMasks','all','FollowLinks','on','BlockType','Goto');
for k=1:numel(gblks)
    if strcmp(get_param(gblks{k},'GotoTag'),'The')
        assert(strcmp(get_param(gblks{k},'TagVisibility'),'global'), ...
            'theta_e Goto "The" must be TagVisibility=global (%s)', gblks{k});
    end
end

save_system(mdl, fullfile(here,[mdl '.slx']));
fprintf('built + saved %s.slx\n', mdl);

% =================================================================== compile self-test (both plants)
evalin('base','clear ff_enable est_sel plant_sel');
set_param(mdl,'StopTime','0.002');
for ps=[1 2]
    assignin('base','plant_sel',ps);
    set_param(mdl,'SimulationCommand','update');
    fprintf('update_diagram OK (plant_sel=%d)\n', ps);
end
evalin('base','clear plant_sel');
set_param(mdl,'StopTime','StopTime');

% =================================================================== FF-on / FF-off demo (the headline effect)
% Default estimator (est_sel=4, recursive KF), equation-linear plant (plant_sel=1).
% To sweep all 7 estimators: loop est_sel=1:7. To use the switched plant: plant_sel=2.
evalin('base','clear est_sel'); assignin('base','plant_sel',1); assignin('base','est_sel',4);
dips = zeros(1,2);
for ff=[0 1]
    assignin('base','ff_enable',ff);
    so = sim(mdl,'ReturnWorkspaceOutputs','on');
    sp = so.log_speed;                       % cols [wref, wm, wm_meas], Array @ ts
    N  = size(sp,1); tt = (0:N-1)'*1e-4;      % ts=1e-4 default
    win = (tt>=0.15 & tt<=0.25);             % load-step transient window
    wref_hi = sp(end,1);
    dips(ff+1) = wref_hi - min(sp(win,2));   % speed dip below setpoint at the load step
    fprintf('  ff_enable=%d : load-step speed dip = %.4f rad/s\n', ff, dips(ff+1));
end
evalin('base','clear ff_enable est_sel plant_sel');
if dips(1) > 0
    fprintf('FF dip reduction: %.1f%% (FF-off %.3f -> FF-on %.3f rad/s)\n', ...
        100*(1-dips(2)/dips(1)), dips(1), dips(2));
end
fprintf('demo done. Edit the InitFcn PARAMETER SURFACE to retarget; loop est_sel=1:7 to compare variants.\n');

% ---- nested helpers ----
    function add(src,dst,pos,varargin)
        add_block(src,[mdl '/' dst],'Position',pos,varargin{:});
    end
    function pos = G(c,r,w,h)
        if nargin<3, w=120; end
        if nargin<4, h=40; end
        pos = [60+c*200, 40+r*70, 60+c*200+w, 40+r*70+h];
    end
end

% ============================================================================================
% ============================== PLANT VARIANT SUBSYSTEM ======================================
% ============================================================================================
function build_plant_variant_subsystem(mdl, pmsm_lib) %#ok<INUSD>
vss = [mdl '/Plant'];
add_block('simulink/Ports & Subsystems/Variant Subsystem', vss, 'Position', [620 300 760 560]);
for nm = {'In1','Out1','Subsystem','Subsystem1'}
    try, delete_block([vss '/' nm{1}]); catch, end
end
% Plant-level external ports: In 1=Vd 2=Vq 3=TL ; Out 1=id 2=iq 3=wm 4=Te 5=ia 6=ib 7=ic
inNames  = {'Vd','Vq','TL'};
outNames = {'id','iq','wm','Te','ia','ib','ic'};
for i=1:numel(inNames)
    add_block('built-in/Inport',[vss '/' inNames{i}],'Position',[40 40+40*i 70 54+40*i]);
end
for i=1:numel(outNames)
    add_block('built-in/Outport',[vss '/' outNames{i}],'Position',[900 40+40*i 930 54+40*i]);
end
% A Variant Subsystem auto-maps each member's ports to the VSS-level ports BY NUMBER — do NOT
% add_line between them (Simulink rejects lines at the VSS level). Just match the port counts.
build_eqn_variant([vss '/EQN']);
build_sim_variant([vss '/SIM'], pmsm_lib);
set_param(vss,'VariantControlMode','expression');
set_param([vss '/EQN'],'VariantControl','plant_sel==1');
set_param([vss '/SIM'],'VariantControl','plant_sel==2');
end

% --------------------------------------------------------------------- EQN variant interior
function build_eqn_variant(sub)
add_block('built-in/Subsystem', sub, 'Position',[300 200 420 360]);
add_block('built-in/Inport', [sub '/Vd'],'Position',[20 40 50 54]);
add_block('built-in/Inport', [sub '/Vq'],'Position',[20 100 50 114]);
add_block('built-in/Inport', [sub '/TL'],'Position',[20 160 50 174]);
on = {'id','iq','wm','Te','ia','ib','ic'};
for i=1:numel(on), add_block('built-in/Outport',[sub '/' on{i}],'Position',[900 40+40*i 930 54+40*i]); end
Le = @(a,b) add_line(sub,a,b,'autorouting','on');
% d-axis current: did = (Vd - Rs*id + pn*wm*Lq*iq)/Ld
add_block('simulink/Signal Routing/Mux',[sub '/Mx_did'],'Inputs','4','Position',[120 40 150 130]);
add_block('simulink/User-Defined Functions/Fcn',[sub '/F_did'],'Expr','(u(1)-Rs*u(2)+pn*u(3)*Lq*u(4))/Ld','Position',[180 60 280 100]);
add_block('simulink/Continuous/Integrator',[sub '/I_id'],'InitialCondition','0','Position',[320 60 350 90]);
Le('Mx_did/1','F_did/1'); Le('F_did/1','I_id/1');
% q-axis current: diq = (Vq - Rs*iq - pn*wm*Ld*id - pn*wm*psif)/Lq
add_block('simulink/Signal Routing/Mux',[sub '/Mx_diq'],'Inputs','4','Position',[120 160 150 250]);
add_block('simulink/User-Defined Functions/Fcn',[sub '/F_diq'],'Expr','(u(1)-Rs*u(2)-pn*u(3)*Ld*u(4)-pn*u(3)*psif)/Lq','Position',[180 180 280 220]);
add_block('simulink/Continuous/Integrator',[sub '/I_iq'],'InitialCondition','0','Position',[320 180 350 210]);
Le('Mx_diq/1','F_diq/1'); Le('F_diq/1','I_iq/1');
% torque Te = 1.5*pn*psif*iq
add_block('simulink/User-Defined Functions/Fcn',[sub '/F_Te'],'Expr','1.5*pn*psif*u(1)','Position',[400 180 480 220]);
Le('I_iq/1','F_Te/1');
% mechanical: dwm = (Te - B*wm - TL)/J
add_block('simulink/Signal Routing/Mux',[sub '/Mx_dwm'],'Inputs','3','Position',[520 260 550 340]);
add_block('simulink/User-Defined Functions/Fcn',[sub '/F_dwm'],'Expr','(u(1)-B*u(2)-u(3))/J','Position',[580 280 660 320]);
add_block('simulink/Continuous/Integrator',[sub '/I_wm'],'InitialCondition','0','Position',[700 280 730 310]);
Le('F_Te/1','Mx_dwm/1'); Le('I_wm/1','Mx_dwm/2'); Le('TL/1','Mx_dwm/3');
Le('Mx_dwm/1','F_dwm/1'); Le('F_dwm/1','I_wm/1');
% theta_e = integral(pn*wm) ; publish Goto "The" (global)
add_block('simulink/User-Defined Functions/Fcn',[sub '/F_we'],'Expr','pn*u(1)','Position',[760 280 820 310]);
add_block('simulink/Continuous/Integrator',[sub '/I_th'],'InitialCondition','0','Position',[840 280 870 310]);
add_block('simulink/Signal Routing/Goto',[sub '/Goto_The'],'Position',[900 282 960 308]);
set_param([sub '/Goto_The'],'GotoTag','The','TagVisibility','global');
Le('I_wm/1','F_we/1'); Le('F_we/1','I_th/1'); Le('I_th/1','Goto_The/1');
% voltage-loop closure (states feed back into dq derivative Fcns)
Le('Vd/1','Mx_did/1'); Le('I_id/1','Mx_did/2'); Le('I_wm/1','Mx_did/3'); Le('I_iq/1','Mx_did/4');
Le('Vq/1','Mx_diq/1'); Le('I_iq/1','Mx_diq/2'); Le('I_wm/1','Mx_diq/3'); Le('I_id/1','Mx_diq/4');
% iabc reconstruction via base Anti_Park (currents): port1=iq, port2=id -> ialpha,ibeta
add_block('pmsm_blocks/Anti_Park',[sub '/iAntiPark'],'Position',[400 400 520 460]);
Le('I_iq/1','iAntiPark/1'); Le('I_id/1','iAntiPark/2');
add_block('simulink/Signal Routing/Mux',[sub '/Mx_ab'],'Inputs','2','Position',[560 400 590 460]);
Le('iAntiPark/1','Mx_ab/1'); Le('iAntiPark/2','Mx_ab/2');
add_block('simulink/User-Defined Functions/Fcn',[sub '/F_ia'],'Expr','u(1)','Position',[640 380 720 410]);
add_block('simulink/User-Defined Functions/Fcn',[sub '/F_ib'],'Expr','-0.5*u(1)+(sqrt(3)/2)*u(2)','Position',[640 430 720 460]);
add_block('simulink/User-Defined Functions/Fcn',[sub '/F_ic'],'Expr','-0.5*u(1)-(sqrt(3)/2)*u(2)','Position',[640 480 720 510]);
Le('Mx_ab/1','F_ia/1'); Le('Mx_ab/1','F_ib/1'); Le('Mx_ab/1','F_ic/1');
% outputs
Le('I_id/1','id/1'); Le('I_iq/1','iq/1'); Le('I_wm/1','wm/1'); Le('F_Te/1','Te/1');
Le('F_ia/1','ia/1'); Le('F_ib/1','ib/1'); Le('F_ic/1','ic/1');
end

% --------------------------------------------------------------------- SIM variant interior
function build_sim_variant(sub, pmsm_lib) %#ok<INUSD>
add_block('built-in/Subsystem', sub, 'Position',[300 380 420 540]);
add_block('built-in/Inport', [sub '/Vd'],'Position',[20 40 50 54]);
add_block('built-in/Inport', [sub '/Vq'],'Position',[20 100 50 114]);
add_block('built-in/Inport', [sub '/TL'],'Position',[20 160 50 174]);
on = {'id','iq','wm','Te','ia','ib','ic'};
for i=1:numel(on), add_block('built-in/Outport',[sub '/' on{i}],'Position',[1100 40+40*i 1130 54+40*i]); end
Ls = @(a,b) add_line(sub,a,b,'autorouting','on');
% powergui (one per model region)
add_block('pmsm_blocks/powergui',[sub '/powergui'],'Position',[40 600 120 660]);
set_param([sub '/powergui'],'SimulationMode','Discrete','SampleTime','Ts','SolverType','Tustin/Backward Euler (TBE)');
% Vdq + theta_e -> Anti_Park (voltage): port1=Vq, port2=Vd
add_block('pmsm_blocks/Anti_Park',[sub '/vAntiPark'],'Position',[120 60 240 120]);
Ls('Vq/1','vAntiPark/1'); Ls('Vd/1','vAntiPark/2');
% SVPWM (Valpha,Vbeta) -> pulse(6)
add_block('pmsm_blocks/SVPWM',[sub '/SVPWM'],'Position',[300 60 420 120]);
Ls('vAntiPark/1','SVPWM/1'); Ls('vAntiPark/2','SVPWM/2');
% sector-7 startup fix (api §15.3): localize + DiagnosticForDefault=None on internal MultiPortSwitch
set_param([sub '/SVPWM'],'LinkStatus','inactive');
ms = find_system([sub '/SVPWM'],'LookUnderMasks','all','FollowLinks','on','BlockType','MultiPortSwitch');
for k=1:numel(ms), set_param(ms{k},'DiagnosticForDefault','None'); end
% Universal Bridge + DC source
add_block('pmsm_blocks/Universal_Bridge',[sub '/UB'],'Position',[480 60 560 180]);
Ls('SVPWM/1','UB/1');
add_block('pmsm_blocks/DC_Voltage_Source',[sub '/Vdc'],'Position',[480 240 540 300]);
add_line(sub,'Vdc/RConn1','UB/RConn1','autorouting','on');
add_line(sub,'Vdc/LConn1','UB/RConn2','autorouting','on');
% PMSM
add_block('pmsm_blocks/PMSM',[sub '/PMSM'],'Position',[640 60 760 200]);
add_line(sub,'UB/LConn1','PMSM/LConn1','autorouting','on');
add_line(sub,'UB/LConn2','PMSM/LConn2','autorouting','on');
add_line(sub,'UB/LConn3','PMSM/LConn3','autorouting','on');
Ls('TL/1','PMSM/1');                                   % Tm = TL
% measurement bus -> selector (ias,ibs,ics,w,theta,Te)
add_block('pmsm_blocks/BusSelector_PMSM',[sub '/BS'],'Position',[820 60 880 200]);
set_param([sub '/BS'],'OutputSignals','ias,ibs,ics,w,theta,Te');
Ls('PMSM/1','BS/1');
% theta_e = pn*theta -> Goto "The" (global)
add_block('simulink/Math Operations/Gain',[sub '/Gain_pnTheta'],'Gain','pn','Position',[920 360 980 390]);
Ls('BS/5','Gain_pnTheta/1');                            % BS/5 = theta
add_block('simulink/Signal Routing/Goto',[sub '/Goto_The'],'Position',[1010 362 1070 388]);
set_param([sub '/Goto_The'],'GotoTag','The','TagVisibility','global');
Ls('Gain_pnTheta/1','Goto_The/1');
% id,iq via Clark + Plark on ias,ibs,ics (Plark takes theta_e as explicit inport)
add_block('pmsm_blocks/Clark',[sub '/Clark'],'Position',[920 60 1000 140]);
Ls('BS/1','Clark/1'); Ls('BS/2','Clark/2'); Ls('BS/3','Clark/3');
add_block('pmsm_blocks/Plark',[sub '/Plark'],'Position',[1040 60 1120 160]);
Ls('Clark/1','Plark/1'); Ls('Clark/2','Plark/2'); Ls('Gain_pnTheta/1','Plark/3');
% outputs
Ls('Plark/1','id/1'); Ls('Plark/2','iq/1');
Ls('BS/4','wm/1');                                      % w
Ls('BS/6','Te/1');                                      % Te
Ls('BS/1','ia/1'); Ls('BS/2','ib/1'); Ls('BS/3','ic/1');
end

% ============================================================================================
% ============================== ESTIMATOR ATOM SIZING / INLINE CHARTS ========================
% ============================================================================================
function pin_kf_atom(blk, n)
% R2024b SIZING recipe (foc_ltid_api_notes §E.1): localize + pin port Size, DataType stays Inherit.
set_param(blk,'LinkStatus','inactive');
ch = sfroot().find('-isa','Stateflow.EMChart','Path',blk); ch = ch(1);
d  = ch.find('-isa','Stateflow.Data');
szmap = struct('y','1','u','1', ...
    'A',sprintf('[%d %d]',n,n),'B',sprintf('[%d 1]',n),'C',sprintf('[1 %d]',n),'D','1', ...
    'Q',sprintf('[%d %d]',n,n),'R','1','est_load','1','omega_hat','1','yhat','1');
for ii=1:numel(d)
    nm = d(ii).Name;
    if isfield(szmap,nm), d(ii).Props.Array.Size = szmap.(nm); end
end
ch.SampleTime = '-1'; ch.ChartUpdate = 'INHERITED';
end

function set_kf6_script(blk)
% F9.a correlated process/measurement-noise KF (n=2), modified gain with cross-term M.
body = strjoin({ ...
 'function est_load = kf6(y, u, A, B, C, D, Q, R, M)'
 '%#codegen'
 '% F9.a: K = (P*C''+M)/(C*P*C''+C*M+M''*C''+R); P = P - K*(C*P+M'').'
 'persistent xhat P'
 'if isempty(xhat)'
 '    xhat = zeros(2,1);'
 '    P    = eye(2);'
 'end'
 'xhat = A*xhat + B*u;'
 'P    = A*P*A.'' + Q;'
 'S    = C*P*C.'' + C*M + M.''*C.'' + R;'
 'K    = (P*C.'' + M) / S;'
 'resid = y - (C*xhat + D*u);'
 'xhat = xhat + K*resid;'
 'P    = P - K*(C*P + M.'');'
 'est_load = xhat(2);'
 'end'
 }, newline);
ch = sfroot().find('-isa','Stateflow.EMChart','Path',blk); ch = ch(1);
ch.Script = body; ch.SampleTime = '-1'; ch.ChartUpdate = 'INHERITED';
pin_chart_sizes(ch, struct('y','1','u','1','A','[2 2]','B','[2 1]','C','[1 2]', ...
                           'D','1','Q','[2 2]','R','1','M','[2 1]','est_load','1'));
end

function set_kf7_script(blk)
% F10 continuous Kalman-Bucy: zdot = f(z,y,u). z=[xhat1;xhat2;P11;P12;P22], P symmetric.
body = strjoin({ ...
 'function zdot = kf7(y, u, A, B, C, G, Qc, Rc, z)'
 '%#codegen'
 '% Continuous Riccati flow (F10.a): K=P*C''/Rc; xdot=A*x+B*u+K*(y-C*x);'
 '% Pdot=A*P+P*A''-P*C''*(1/Rc)*C*P+G*Qc*G''.'
 'xhat = [z(1); z(2)];'
 'P    = [z(3) z(4); z(4) z(5)];'
 'K    = P*C.'' / Rc;'
 'xd   = A*xhat + B*u + K*(y - C*xhat);'
 'Pd   = A*P + P*A.'' - P*C.''*(1/Rc)*C*P + G*Qc*G.'';'
 'zdot = [xd(1); xd(2); Pd(1,1); Pd(1,2); Pd(2,2)];'
 'end'
 }, newline);
ch = sfroot().find('-isa','Stateflow.EMChart','Path',blk); ch = ch(1);
ch.Script = body; ch.SampleTime = '-1'; ch.ChartUpdate = 'INHERITED';
pin_chart_sizes(ch, struct('y','1','u','1','A','[2 2]','B','[2 1]','C','[1 2]', ...
                           'G','[2 1]','Qc','1','Rc','1','z','[5 1]','zdot','[5 1]'));
end

function set_kbpick_script(blk)
body = strjoin({ ...
 'function TLhat = kbpick(z)'
 '%#codegen'
 'TLhat = z(2);'   % continuous load-torque estimate (state 2)
 'end'
 }, newline);
ch = sfroot().find('-isa','Stateflow.EMChart','Path',blk); ch = ch(1);
ch.Script = body; ch.SampleTime = '-1'; ch.ChartUpdate = 'INHERITED';
pin_chart_sizes(ch, struct('z','[5 1]','TLhat','1'));
end

function pin_chart_sizes(ch, szmap)
% Pin Size on inline-chart data ports (DataType stays Inherit) — R2024b output-size
% inference gap (same as the KF atom, foc_ltid_api_notes §E.1).
d = ch.find('-isa','Stateflow.Data');
for ii=1:numel(d)
    nm = d(ii).Name;
    if isfield(szmap,nm), d(ii).Props.Array.Size = szmap.(nm); end
end
end

function tf = bdIsLoaded(name)
    tf = any(strcmp(find_system('SearchDepth',0,'type','block_diagram'),name));
end

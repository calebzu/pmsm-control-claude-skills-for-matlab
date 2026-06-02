function run_phase8()
% RUN_PHASE8  Phase-8 generalization verification harness for motor-foc-ltid-pmsm.
% New machine: Rs=1.2, Ld=Lq=8.5e-3, psif=0.175, pn=3, J=3e-4, B=8e-5.
% Scenario: const wref=80 rad/s, sustained load step +1.5 N.m @ 0.2s held to 0.40s, EQN plant.
% Runs {est_sel 1,4} x {ff 0,1}; computes Visual-4-check + Mode-B S1..S5 + T_hat_L
% convergence + FF dip reduction. SCALAR console output only; figures raised for G4.

mdl  = 'foc_ltid_run1';
d    = fileparts(mfilename('fullpath'));
% repo root = nearest ancestor holding shared/building_blocks/pmsm_blocks.slx (base atoms, by pointer)
root = d;
while ~isfile(fullfile(root,'shared','building_blocks','pmsm_blocks.slx'))
    pp = fileparts(root);
    if strcmp(pp,root), error('repo root (shared/building_blocks/pmsm_blocks.slx) not found above %s', d); end
    root = pp;
end
% method root = nearest ancestor holding building_blocks/foc_ltid_blocks.slx (method-local KF atom)
mroot = d;
while ~isfile(fullfile(mroot,'building_blocks','foc_ltid_blocks.slx'))
    pp = fileparts(mroot);
    if strcmp(pp,mroot), error('building_blocks/foc_ltid_blocks.slx not found above %s', d); end
    mroot = pp;
end
% load libs (EQN plant copies atoms but Anti_Park is a library link) + the saved model
load_system(fullfile(root,'shared','building_blocks','pmsm_blocks.slx'));
load_system(fullfile(mroot,'building_blocks','foc_ltid_blocks.slx'));
slx = fullfile(d,[mdl '.slx']);
assert(isfile(slx),'missing %s -- run build_foc_ltid_run1 first',slx);
if any(strcmp(find_system('SearchDepth',0,'type','block_diagram'),mdl)); close_system(mdl,0); end
load_system(slx);

ts=1e-4; wref=80; TLtrue=1.5; Bv=8e-5; tL_on=0.2; StopTime=0.40; iq_max=10;
w_dip = @(tt) (tt>=tL_on & tt<=tL_on+0.10);   % dip search window after step
w_post= @(tt) (tt>=0.30 & tt<=StopTime);      % post-load steady (>5*tau_max after step)

combos = [1 0;1 1;4 0;4 1];   % rows: es1ff0, es1ff1, es4ff0, es4ff1
clear ff_enable est_sel plant_sel
assignin('base','plant_sel',1);
R = struct([]);
for i=1:size(combos,1)
  es=combos(i,1); ff=combos(i,2);
  assignin('base','est_sel',es); assignin('base','ff_enable',ff);
  so = sim(mdl,'ReturnWorkspaceOutputs','on');
  sp=so.log_speed; tl=so.log_TLhat; cu=so.log_cur; tq=so.log_trq; ic=so.log_iabc;
  N=size(sp,1); tt=(0:N-1)'*ts;
  R(i).es=es; R(i).ff=ff; R(i).tt=tt;
  R(i).wref=sp(:,1); R(i).wm=sp(:,2); R(i).wmeas=sp(:,3);
  R(i).TLtrue=tl(:,1); R(i).TL1=tl(:,2); R(i).TL4=tl(:,5);   % col2=var1 Luenberger, col5=var4 recursive KF
  R(i).iqref=cu(:,1); R(i).iq=cu(:,2); R(i).id=cu(:,3);
  R(i).Te=tq(:,1); R(i).ia=ic(:,1); R(i).ib=ic(:,2); R(i).ic=ic(:,3);
  wd=w_dip(tt); wpo=w_post(tt);
  R(i).dip      = wref - min(R(i).wm(wd));
  R(i).wm_post  = mean(R(i).wm(wpo));
  R(i).iq_post  = mean(R(i).iq(wpo));
  R(i).id_post  = mean(R(i).id(wpo));
  R(i).Te_post  = mean(R(i).Te(wpo));
  R(i).bal_post = mean(R(i).Te(wpo)) - (TLtrue + Bv*mean(R(i).wm(wpo)));
  R(i).TL1_post = mean(R(i).TL1(wpo));
  R(i).TL4_post = mean(R(i).TL4(wpo));
  R(i).iq_absmax= max(abs(R(i).iq));
  R(i).anyNaN   = any(~isfinite([sp(:);tl(:);cu(:);tq(:);ic(:)]));
  ax=R(i).ia(wpo); R(i).ia_amp=(max(ax)-min(ax))/2; R(i).ia_mean=mean(ax);
  R(i).ia_zc = sum(abs(diff(sign(ax-mean(ax))))>0);   % zero-crossings post-load (AC signature)
end

% ---------------- scalar summary ----------------
fprintf('\n==== Phase-8 G1 summary | new machine Rs1.2 Ld=Lq8.5m psif.175 pn3 J3e-4 B8e-5 ====\n');
fprintf('wref=%.0f rad/s, load +%.1f N.m @ %.2fs held to %.2fs, EQN plant, ts=%.0e\n',wref,TLtrue,tL_on,StopTime,ts);
fprintf('%-2s %-2s | %7s | %8s %7s %7s | %7s %8s | %6s %6s | %3s\n',...
 'es','ff','dip','wm_post','iq_pst','id_pst','Te_pst','bal','TL1','TL4','NaN');
for i=1:numel(R)
 fprintf('%-2d %-2d | %7.3f | %8.3f %7.3f %7.3f | %7.4f %8.4f | %6.3f %6.3f | %3d\n',...
  R(i).es,R(i).ff,R(i).dip,R(i).wm_post,R(i).iq_post,R(i).id_post,R(i).Te_post,R(i).bal_post,R(i).TL1_post,R(i).TL4_post,R(i).anyNaN);
end
red1=100*(1-R(2).dip/R(1).dip); red4=100*(1-R(4).dip/R(3).dip);
fprintf('\nFF dip reduction: est_sel=1 Luenberger %.1f%% (%.3f->%.3f) | est_sel=4 recKF %.1f%% (%.3f->%.3f)\n',...
 red1,R(1).dip,R(2).dip,red4,R(3).dip,R(4).dip);
fprintf('T_hatL steady (post-load, ->1.5): var1=%.3f var4=%.3f (true=%.3f)\n',R(4).TL1_post,R(4).TL4_post,TLtrue);

% ---------------- Visual-4-check + Mode-B (representative: est_sel=4, ff=1 = row 4) ----------------
k=4;
c1 = abs(R(k).wm_post-wref)/wref < 0.05;
c2 = R(k).iq_absmax < 0.95*iq_max;
c3 = (R(k).ia_amp>0.3) && (abs(R(k).ia_mean)<0.2*R(k).ia_amp) && (R(k).ia_zc>=4);
c4 = abs(R(k).bal_post) < 0.05;
fprintf('\nVISUAL-4-CHECK (es4,ff1): rotate=%d  iq_track(notBangBang)=%d  abc_AC=%d  Te_balance=%d\n',c1,c2,c3,c4);
fprintf('  iabc post-load: amp=%.3f A, mean=%.4f A, zerocross=%d ; iq_absmax=%.3f (<iq_max=%.0f)\n',...
 R(k).ia_amp,R(k).ia_mean,R(k).ia_zc,R(k).iq_absmax,iq_max);
trough=wref-R(k).dip;
S1=abs(R(k).wm_post-wref)/wref<0.05;
S2=(trough>=0.5*wref)&&(R(k).wm_post>=0.95*wref);
S3=(R(k).iq_absmax<iq_max)&&(R(k).iq_post>0.1);
S4=abs(R(k).id_post)<0.5;
S5=(~R(k).anyNaN)&&(abs(R(k).ia_mean)<0.2*max(R(k).ia_amp,eps));
fprintf('MODE-B S1..S5 (es4,ff1): S1=%d S2=%d S3=%d S4=%d S5=%d => %d/5 (pass>=4)\n',S1,S2,S3,S4,S5,S1+S2+S3+S4+S5);

% ---------------- figures for G4 (rendered in MATLAB desktop) ----------------
% Fig1: speed wm FFoff vs FFon (both estimators), zoom around the load step
f1=figure('Name','G4-1 speed wm: FF off vs on','Color','w');
subplot(2,1,1); plot(R(3).tt,R(3).wref,'k--',R(3).tt,R(3).wm,'r',R(4).tt,R(4).wm,'b','LineWidth',1.1);
grid on; xlim([0.15 0.35]); ylim([wref-12 wref+3]); xlabel('t [s]'); ylabel('\omega_m [rad/s]');
title('est\_sel=4 recursive KF: FF off (red) vs FF on (blue), wref (k--)'); legend('wref','FF off','FF on','Location','southeast');
subplot(2,1,2); plot(R(1).tt,R(1).wref,'k--',R(1).tt,R(1).wm,'r',R(2).tt,R(2).wm,'b','LineWidth',1.1);
grid on; xlim([0.15 0.35]); ylim([wref-12 wref+3]); xlabel('t [s]'); ylabel('\omega_m [rad/s]');
title('est\_sel=1 Luenberger: FF off (red) vs FF on (blue)'); legend('wref','FF off','FF on','Location','southeast');

% Fig2: iq_ref vs iq (es4 ff1) -> tracks, not bang-bang
f2=figure('Name','G4-2 iq tracking','Color','w');
plot(R(4).tt,R(4).iqref,'r',R(4).tt,R(4).iq,'b','LineWidth',1.0); grid on;
yline(iq_max,'k:'); yline(-iq_max,'k:'); xlabel('t [s]'); ylabel('A');
title('es4,ff1: iq\_ref (red) vs iq (blue); dotted = \pm iq\_max'); legend('iq\_ref','iq','Location','northeast');

% Fig3: T_hat_L var1 & var4 vs true TL (es4 ff1) -> ->1.5
f3=figure('Name','G4-3 load-torque estimate','Color','w');
plot(R(4).tt,R(4).TLtrue,'k','LineWidth',1.4); hold on;
plot(R(4).tt,R(4).TL1,'r',R(4).tt,R(4).TL4,'b','LineWidth',1.0); grid on;
xlabel('t [s]'); ylabel('N\cdotm'); ylim([-0.5 2.2]);
title('TL true (k) vs estimate: var1 Luenberger (r), var4 recKF (b)'); legend('TL true','T\_hatL var1','T\_hatL var4','Location','southeast');

% Fig4: iabc post-load zoom -> AC sinusoid (not DC-locked)
f4=figure('Name','G4-4 abc currents (post-load)','Color','w');
plot(R(4).tt,R(4).ia,'r',R(4).tt,R(4).ib,'g',R(4).tt,R(4).ic,'b'); grid on;
xlim([0.30 0.36]); xlabel('t [s]'); ylabel('A'); title('es4,ff1: i_a/i_b/i_c post-load (AC sinusoid)'); legend('ia','ib','ic');

% Fig5: Te vs TL+B*wm (es4 ff1) -> energy balance
f5=figure('Name','G4-5 torque energy balance','Color','w');
plot(R(4).tt,R(4).Te,'b',R(4).tt,R(4).TLtrue+Bv*R(4).wm,'r--','LineWidth',1.0); grid on;
xlabel('t [s]'); ylabel('N\cdotm'); ylim([-0.5 3]);
title('es4,ff1: Te (blue) vs TL + B\cdot\omega_m (red--)'); legend('Te','TL+B\omega','Location','southeast');

save(fullfile(d,'results_phase8.mat'),'R');
try
  saveas(f1,fullfile(d,'g4_1_speed.png')); saveas(f2,fullfile(d,'g4_2_iq.png'));
  saveas(f3,fullfile(d,'g4_3_TLhat.png')); saveas(f4,fullfile(d,'g4_4_iabc.png'));
  saveas(f5,fullfile(d,'g4_5_balance.png'));
catch e; fprintf('savepng warn: %s\n',e.message); end
fprintf('\nsaved results_phase8.mat + g4_*.png in %s\n',d);
fprintf('run_phase8 done.\n');
end

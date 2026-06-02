function run_phase8_full()
% RUN_PHASE8_FULL  Full Phase-8 matrix: 7 estimators x 2 plants x {FF off,on} = 28 sims.
% Proves the skill reproduces ALL 7 estimator variants on BOTH plant realizations for the
% new machine. Per-cell physics judged on its own (no cross-plant compare): T_hatL -> 1.5
% and FF dip(ff1) < dip(ff0). Results accumulate into a NUMERIC matrix (no struct array).
% SCALAR output only; one dip-bar figure per plant for G4.

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
load_system(fullfile(root,'shared','building_blocks','pmsm_blocks.slx'));
load_system(fullfile(mroot,'building_blocks','foc_ltid_blocks.slx'));
slx = fullfile(d,[mdl '.slx']);
assert(isfile(slx),'missing %s -- run build_foc_ltid_run1 first',slx);
if any(strcmp(find_system('SearchDepth',0,'type','block_diagram'),mdl)); close_system(mdl,0); end
load_system(slx);

ts=1e-4; wref=80; TLtrue=1.5; Bv=8e-5; tL_on=0.2; StopTime=0.40;
vnames={'1 Luenb','2 ssKF','3 ssColor','4 recKF','5 recColor','6 corrKF','7 KBucy'};
% log_TLhat columns: 1=true, 2=v1,3=v2,4=v3,5=v4,6=v5,7=v6,8=v7  => est_sel es -> column es+1
pn_name={'EQN','SIM'};

% numeric results matrix: one row per (plant,es); 15 cols
% [plant es dipOff dipOn ffRed TLhat wm_post iq_post iqAbsMax id_post Te_bal anyNaN ia_amp ia_mean ia_zc]
M = zeros(14,15); r=0; T0=tic;
for ps=[1 2]
  assignin('base','plant_sel',ps);
  for es=1:7
    assignin('base','est_sel',es);
    dipoff=NaN; dipon=NaN;
    tl_post=NaN; wmp=NaN; iqp=NaN; iqmx=NaN; idp=NaN; tebal=NaN; nanflag=0; iaamp=NaN; iamean=NaN; iazc=NaN;
    for ff=[0 1]
      assignin('base','ff_enable',ff);
      so = sim(mdl,'ReturnWorkspaceOutputs','on');
      sp=so.log_speed; tl=so.log_TLhat; cu=so.log_cur; tq=so.log_trq; ic=so.log_iabc;
      N=size(sp,1); tt=(0:N-1)'*ts;
      wd =(tt>=tL_on & tt<=tL_on+0.10);
      wpo=(tt>=0.30 & tt<=StopTime);
      dval = wref - min(sp(wd,2));
      if ff==0
        dipoff=dval;
      else
        dipon=dval;
        wmp=mean(sp(wpo,2)); iqp=mean(cu(wpo,2)); iqmx=max(abs(cu(:,2)));
        idp=mean(cu(wpo,3)); tebal=mean(tq(wpo,1))-(TLtrue+Bv*mean(sp(wpo,2)));
        tl_post=mean(tl(wpo,es+1));
        nanflag=any(~isfinite([sp(:);tl(:);cu(:);tq(:);ic(:)]));
        ax=ic(wpo,1); iaamp=(max(ax)-min(ax))/2; iamean=mean(ax);
        iazc=sum(abs(diff(sign(ax-mean(ax))))>0);
      end
    end
    ffred=100*(1-dipon/dipoff);
    r=r+1;
    M(r,:)=[ps es dipoff dipon ffred tl_post wmp iqp iqmx idp tebal nanflag iaamp iamean iazc];
  end
end
fprintf('=== 28 sims done in %.1f s ===\n', toc(T0));

% ---------------- scalar table ----------------
fprintf('\n==== Phase-8 FULL matrix | new machine 80 rad/s +1.5N.m | 7 est x 2 plant ====\n');
fprintf('%-4s %-10s | %7s %7s %6s | %7s %7s %7s | %8s %6s %3s | %3s\n',...
 'plnt','estimator','dipOff','dipOn','red%','TLhat','wm_pst','iq_pst','Te_bal','iqmax','NaN','AC');
for i=1:size(M,1)
  ps=M(i,1); es=M(i,2);
  acOK=(M(i,13)>0.3)&&(abs(M(i,14))<0.3*max(M(i,13),eps))&&(M(i,15)>=4);
  fprintf('%-4s %-10s | %7.3f %7.3f %5.1f | %7.3f %7.2f %7.3f | %8.4f %6.2f %3d | %3d\n',...
   pn_name{ps}, vnames{es}, M(i,3),M(i,4),M(i,5),M(i,6),M(i,7),M(i,8),M(i,11),M(i,9),M(i,12),acOK);
end

% ---------------- per-cell verdicts ----------------
fprintf('\n---- per-cell PASS (TLhat in [1.275,1.665] & FFred>1 & |wm-80|/80<5%% & AC & noNaN) ----\n');
npass=0;
for i=1:size(M,1)
  ps=M(i,1); es=M(i,2);
  tlOK=abs(M(i,6)-TLtrue)<=0.15*TLtrue;
  ffOK=M(i,5)>1;
  wmOK=abs(M(i,7)-wref)/wref<0.05;
  acOK=(M(i,13)>0.3)&&(abs(M(i,14))<0.3*max(M(i,13),eps))&&(M(i,15)>=4);
  nzOK=~M(i,12);
  cell=tlOK&&ffOK&&wmOK&&acOK&&nzOK; npass=npass+cell;
  v='PASS'; if ~cell; v='FAIL'; end
  fprintf('%-4s %-10s : TLhat=%d FFred=%d wm=%d AC=%d noNaN=%d => %s\n',...
    pn_name{ps},vnames{es},tlOK,ffOK,wmOK,acOK,nzOK,v);
end
fprintf('\nTOTAL: %d/%d cells PASS (7 estimators x 2 plants)\n', npass, size(M,1));

save(fullfile(d,'results_phase8_full.mat'),'M','vnames','pn_name');

% ---------------- two G4 dip-bar figures ----------------
for ps=[1 2]
  idx=find(M(:,1)==ps);
  f=figure('Name',sprintf('G4-full %s dip off vs on',pn_name{ps}),'Color','w');
  bar([M(idx,3) M(idx,4)]); grid on; set(gca,'XTick',1:7,'XTickLabel',vnames,'XTickLabelRotation',20);
  ylabel('load-step speed dip [rad/s]'); legend('FF off','FF on','Location','northeast');
  title(sprintf('%s plant: dip FF-off vs FF-on, 7 estimators (new machine)',pn_name{ps}));
  saveas(f, fullfile(d,sprintf('g4_full_%s_dip.png',pn_name{ps})));
end
drawnow; figs=findall(0,'Type','figure'); for i=1:numel(figs); figure(figs(i)); end; drawnow;
fprintf('saved results_phase8_full.mat + g4_full_*.png ; live figures=%d\n', numel(findall(0,'Type','figure')));
end

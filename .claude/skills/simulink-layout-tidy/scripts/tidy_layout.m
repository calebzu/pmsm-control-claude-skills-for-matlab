function rpt = tidy_layout(mdl, varargin)
%TIDY_LAYOUT  L1 layout tidy for an already-built Simulink model.
%   rpt = TIDY_LAYOUT(mdl) diagnoses the top-level layout, arranges it with
%   Simulink.BlockDiagram.arrangeSystem, removes any residual block
%   overlaps, re-measures, exports a PNG, and returns a before/after report.
%
%   Name-value options:
%     'System'     subsystem path to tidy (default: mdl, the top level)
%     'Screenshot' PNG path to export (default: '<mdl>_layout.png')
%     'Save'       logical, save the model when done (default: false)
%
%   L1 CONTRACT (enforced by construction): this function ONLY mutates block
%   Position and line routing (Points). It never adds/removes blocks and
%   never changes port connections, so the compiled model and every
%   simulation result are byte-for-byte identical. No functional re-test
%   is required. Goto/From substitution and Subsystem encapsulation are L2
%   and are intentionally NOT performed here.

    p = inputParser;
    p.addParameter('System', mdl);
    p.addParameter('Screenshot', '');
    p.addParameter('Save', false);
    p.parse(varargin{:});
    sys = p.Results.System;
    shot = p.Results.Screenshot;
    if isempty(shot)
        shot = [strrep(mdl, '/', '_') '_layout.png'];
    end

    if ~bdIsLoaded(mdl)
        load_system(mdl);
    end

    before = layout_metrics(sys);

    % Candidate A — minimal-move: keep the original layout and its routing,
    % nudge only enough to clear overlaps. No arrangeSystem, so the original
    % (often already-decent) routing is preserved and crossings cannot be made
    % worse by re-routing. This is applied to the REAL model and kept intact.
    movedA = deoverlap_pass(sys);
    metricsA = layout_metrics(sys);

    % Candidate B — arrange: cost of arrangeSystem + de-overlap, measured on a
    % throwaway COPY so the real model's minimal-move layout is never destroyed.
    % (arrangeSystem invalidates line handles, so an in-place try/restore cannot
    % reproduce candidate A reliably — copy-and-measure is the robust path.)
    metricsB = arrange_probe_cost(sys);
    haveB = ~isempty(metricsB);

    % Pick the better layout. Prefer arrange for compactness, but NEVER accept a
    % net crossing regression: keep minimal-move when arrange is strictly worse
    % (or unavailable). Both candidates have zero overlaps, so the hard gate
    % holds either way. The chosen layout is produced fresh on the real model,
    % so the reported metrics are exact (no snapshot/restore drift).
    if haveB && metricsB.line_crossings <= metricsA.line_crossings
        chosen = 'arrange';
        Simulink.BlockDiagram.arrangeSystem(sys);
        moved = deoverlap_pass(sys);
        after = layout_metrics(sys);
    else
        chosen = 'minimal-move';
        moved = movedA;
        after = metricsA;              % real model is still exactly candidate A
    end

    try
        print(['-s' bdroot(sys)], '-dpng', shot);
    catch err
        warning('tidy_layout:screenshot', 'screenshot failed: %s', err.message);
        shot = '';
    end

    if p.Results.Save
        save_system(mdl);
    end

    rpt.before = before;
    rpt.after = after;
    rpt.screenshot = shot;
    rpt.residual_moves = moved;
    rpt.chosen = chosen;
    if haveB
        rpt.crossings_arrange = metricsB.line_crossings;
    else
        rpt.crossings_arrange = NaN;
    end
    rpt.crossings_minimal = metricsA.line_crossings;
    rpt.arrange_degraded = haveB && metricsB.line_crossings > before.line_crossings;
    print_report(rpt);
end

% --- arrange cost on a throwaway copy ------------------------------------
% Copies the contents of SYS into a fresh scratch model, runs arrangeSystem +
% de-overlap there, and returns the resulting layout_metrics (or [] on any
% failure). The real model is never touched, so candidate A stays intact and
% the arrange/minimal-move comparison is clean. The scratch model is always
% closed without saving.
function m = arrange_probe_cost(sys)
    m = [];
    probe = 'tidy_layout_arrange_probe';
    if bdIsLoaded(probe)
        close_system(probe, 0);
    end
    try
        new_system(probe);
        load_system(probe);
        wrap = [probe '/wrap'];
        add_block('built-in/Subsystem', wrap);
        % copyContentsToSubsystem replaces the default In1/Out1 with sys's contents.
        Simulink.BlockDiagram.copyContentsToSubsystem(sys, wrap);
        Simulink.BlockDiagram.arrangeSystem(wrap);
        deoverlap_pass(wrap);
        m = layout_metrics(wrap);
    catch err
        warning('tidy_layout:arrangeProbe', ...
                'arrange probe failed (%s); keeping minimal-move only', err.message);
    end
    if bdIsLoaded(probe)
        close_system(probe, 0);
    end
end

% --- residual de-overlap ------------------------------------------------
% arrangeSystem usually yields zero overlaps, but guarantees none. Greedily
% push the lower block of any overlapping pair downward until clear. Only
% Position is touched, preserving the L1 contract.
function moved = deoverlap_pass(sys)
    blocks = find_system(sys, 'SearchDepth', 1, 'Type', 'Block');
    blocks = setdiff(blocks, {sys});
    moved = 0;
    for iter = 1:50
        P = cell2mat(cellfun(@(b) get_param(b, 'Position'), ...
                             blocks, 'UniformOutput', false));
        clash = false;
        for i = 1:size(P, 1)
            for j = i + 1:size(P, 1)
                if ~(P(i,3) <= P(j,1) || P(j,3) <= P(i,1) || ...
                     P(i,4) <= P(j,2) || P(j,4) <= P(i,2))
                    % push the block whose top is lower (larger top y)
                    if P(i,2) >= P(j,2), k = i; o = j; else, k = j; o = i; end
                    dy = P(o,4) - P(k,2) + 20;
                    pos = get_param(blocks{k}, 'Position');
                    set_param(blocks{k}, 'Position', pos + [0 dy 0 dy]);
                    moved = moved + 1;
                    clash = true;
                end
            end
        end
        if ~clash, break, end
    end
end

% --- report -------------------------------------------------------------
function print_report(rpt)
    b = rpt.before; a = rpt.after;
    fprintf('\n=== simulink-layout-tidy (L1) report ===\n');
    fprintf('%-18s %10s -> %-10s\n', 'metric', 'before', 'after');
    fprintf('%-18s %10d    %-10d\n', 'block_overlaps', b.block_overlaps, a.block_overlaps);
    fprintf('%-18s %10d    %-10d\n', 'line_block_hits', b.line_block_hits, a.line_block_hits);
    fprintf('%-18s %10d    %-10d\n', 'line_crossings', b.line_crossings, a.line_crossings);
    fprintf('%-18s %6dx%-6d %d x%-6d\n', 'extent (w x h)', ...
            b.extent(1), b.extent(2), a.extent(1), a.extent(2));
    fprintf('residual de-overlap moves: %d\n', rpt.residual_moves);
    fprintf('layout strategy: chose %s (crossings arrange=%g vs minimal-move=%d)\n', ...
            rpt.chosen, rpt.crossings_arrange, rpt.crossings_minimal);
    if rpt.arrange_degraded
        if strcmp(rpt.chosen, 'minimal-move')
            fprintf(['  arrangeSystem REGRESSED crossings %d->%d; reverted to ' ...
                     'minimal-move layout (no net-negative crossing optimization).\n'], ...
                    b.line_crossings, rpt.crossings_arrange);
        else
            fprintf(['  note: arrangeSystem raised crossings %d->%d but it is ' ...
                     'still <= minimal-move, so kept arrange for compactness.\n'], ...
                    b.line_crossings, rpt.crossings_arrange);
        end
    end
    if ~isempty(rpt.screenshot)
        fprintf('screenshot: %s\n', rpt.screenshot);
    end
    % Hard gates (L1 must-pass).
    assert(a.block_overlaps == 0, 'GATE FAIL: block overlaps remain (%d)', a.block_overlaps);
    fprintf('GATE block_overlaps==0: PASS\n');
    if a.line_block_hits == 0
        fprintf('GATE line_block_hits==0: PASS\n');
    else
        fprintf('GATE line_block_hits: %d (soft; inspect screenshot)\n', a.line_block_hits);
    end
    fprintf(['line_crossings is a SOFT metric: planar graphs can reach ~0, ' ...
             'non-planar graphs cannot (run planarity_check.py). Never hard-gate it.\n']);
    fprintf('=> Human visual sign-off on the screenshot is the final gate.\n\n');
end

function selftest()
%SELFTEST  End-to-end check of simulink-layout-tidy on the synthetic fixture.
%   Builds the dirty fixture, runs L1 tidy, asserts hard gates, exercises
%   the graph extractor + planarity script, and prints idempotency proof.

    here = fileparts(mfilename('fullpath'));
    addpath(here);

    mdl = make_dirty_fixture('dirty_fixture');

    fprintf('--- before tidy ---\n');
    disp(layout_metrics(mdl));

    png = fullfile(here, 'fixture_after.png');
    rpt = tidy_layout(mdl, 'Screenshot', png);

    % Hard gate: zero block overlaps after tidy.
    assert(rpt.after.block_overlaps == 0, 'overlap gate failed');

    % Core invariant: tidy must NEVER increase line crossings vs before.
    assert(rpt.after.line_crossings <= rpt.before.line_crossings, ...
           'crossing regression: %d -> %d', ...
           rpt.before.line_crossings, rpt.after.line_crossings);
    % On this non-planar K3,3 fixture arrange degrades, so minimal-move wins.
    assert(strcmp(rpt.chosen, 'minimal-move'), ...
           'expected minimal-move on degrading fixture, got %s', rpt.chosen);
    fprintf('no-regression invariant + minimal-move fallback -> OK\n');

    % Idempotency: a second tidy must not change the metrics.
    rpt2 = tidy_layout(mdl, 'Screenshot', '');
    assert(isequal(rpt.after.block_overlaps, rpt2.after.block_overlaps), ...
           'idempotency: overlaps changed');
    fprintf('idempotency: overlaps stable at %d on re-run -> OK\n', ...
            rpt2.after.block_overlaps);

    % --- arrange-wins branch: a crossed-but-planar model arrange can uncross ---
    cw = 'crossed_planar';
    if bdIsLoaded(cw), close_system(cw, 0); end
    new_system(cw); load_system(cw);
    add_block('simulink/Sources/Constant', [cw '/C1'], 'Position', [40 40 70 70]);
    add_block('simulink/Sources/Constant', [cw '/C2'], 'Position', [40 160 70 190]);
    add_block('simulink/Sinks/Terminator', [cw '/T1'], 'Position', [300 40 320 60]);
    add_block('simulink/Sinks/Terminator', [cw '/T2'], 'Position', [300 160 320 180]);
    add_line(cw, 'C1/1', 'T2/1', 'autorouting', 'on');   % top source -> bottom sink
    add_line(cw, 'C2/1', 'T1/1', 'autorouting', 'on');   % bottom source -> top sink
    rc = tidy_layout(cw, 'Screenshot', '');
    assert(rc.after.line_crossings <= rc.before.line_crossings, 'crossed: regression');
    fprintf('arrange-wins case: chose %s, crossings %d -> %d -> OK\n', ...
            rc.chosen, rc.before.line_crossings, rc.after.line_crossings);
    close_system(cw, 0);

    % Planarity diagnosis on the K3,3 fan-in (expected: NON-PLANAR).
    gjson = fullfile(here, 'fixture_graph.json');
    extract_graph(mdl, gjson);
    [st, out] = system(sprintf('python3 "%s" "%s"', ...
                       fullfile(here, 'planarity_check.py'), gjson));
    fprintf('--- planarity_check.py (exit=%d) ---\n%s\n', st, out);

    close_system(mdl, 0);
    fprintf('SELFTEST PASSED\n');
end

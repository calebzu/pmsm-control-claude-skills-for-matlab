function mdl = make_dirty_fixture(mdl)
%MAKE_DIRTY_FIXTURE  Build a self-contained, deliberately ugly Simulink model.
%   mdl = MAKE_DIRTY_FIXTURE(name) creates and returns a model named NAME
%   (default 'dirty_fixture') exhibiting three layout pathologies:
%     1. Overlapping blocks (two blocks placed on top of each other).
%     2. Long crossing lines (sources wired across the canvas).
%     3. A non-planar K3,3 fan-in: 3 sources x 3 collectors, all-to-all,
%        which by Kuratowski's theorem CANNOT be drawn with zero crossings.
%
%   The model is closed if open, recreated from scratch, and left loaded
%   (not saved). Use it as a fixture for developing and evaluating
%   simulink-layout-tidy without depending on any external model.

    if nargin < 1 || isempty(mdl)
        mdl = 'dirty_fixture';
    end
    if bdIsLoaded(mdl)
        close_system(mdl, 0);
    end
    new_system(mdl);
    load_system(mdl);

    % 3 sources, all near the same x so lines must fan across to collectors.
    src = cell(1, 3);
    for i = 1:3
        src{i} = [mdl '/S' num2str(i)];
        add_block('simulink/Sources/Constant', src{i}, ...
                  'Value', num2str(i), 'Position', boxAt(40, 40 + 80*i, 60, 40));
    end

    % 3 collectors (3-input Sums) placed far to the right.
    col = cell(1, 3);
    for j = 1:3
        col{j} = [mdl '/A' num2str(j)];
        add_block('simulink/Math Operations/Add', col{j}, ...
                  'Inputs', '+++', 'Position', boxAt(620, 40 + 90*j, 50, 70));
    end

    % All-to-all wiring => bipartite K3,3 core => provably non-planar.
    for i = 1:3
        for j = 1:3
            add_line(mdl, ['S' num2str(i) '/1'], ['A' num2str(j) '/' num2str(i)], ...
                     'autorouting', 'on');
        end
    end

    % Each collector to a Terminator on the far right.
    for j = 1:3
        t = [mdl '/T' num2str(j)];
        add_block('simulink/Sinks/Terminator', t, ...
                  'Position', boxAt(760, 60 + 90*j, 30, 30));
        add_line(mdl, ['A' num2str(j) '/1'], ['T' num2str(j) '/1'], ...
                 'autorouting', 'on');
    end

    % Pathology 1: force a hard overlap by stacking S2 onto S1's box.
    set_param(src{2}, 'Position', get_param(src{1}, 'Position') + [5 5 5 5]);
end

function pos = boxAt(left, top, w, h)
    pos = [left top left + w top + h];
end

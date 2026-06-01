function out = extract_graph(sys, jsonPath)
%EXTRACT_GRAPH  Abstract SYS into a block/line graph and write it as JSON.
%   out = EXTRACT_GRAPH(sys, jsonPath) treats blocks as nodes and lines as
%   edges (by source/destination block) at the top level of SYS, then
%   writes {"nodes":[names], "edges":[[i,j],...]} to JSONPATH (0-based
%   indices) for consumption by planarity_check.py. Returns the same struct.
%
%   Branch lines (one source, many destinations) contribute one edge per
%   destination. Self/invalid endpoints are skipped.

    blocks = find_system(sys, 'SearchDepth', 1, 'Type', 'Block');
    blocks = setdiff(blocks, {sys});
    h = zeros(numel(blocks), 1);
    for i = 1:numel(blocks)
        h(i) = get_param(blocks{i}, 'Handle');
    end
    idx = containers.Map(num2cell(h), num2cell(0:numel(h)-1));

    lines = find_system(sys, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'Line');
    edges = zeros(0, 2);
    seen = containers.Map('KeyType', 'char', 'ValueType', 'logical');
    for k = 1:numel(lines)
        s = get_param(lines(k), 'SrcBlockHandle');
        d = get_param(lines(k), 'DstBlockHandle');
        if ~isscalar(s) || ~isscalar(d) || s < 0 || d < 0 ...
                || ~isKey(idx, s) || ~isKey(idx, d)
            continue   % trunk/branch with no single block endpoint
        end
        a = idx(s); b = idx(d);
        if a == b, continue, end
        key = sprintf('%d-%d', min(a, b), max(a, b));
        if isKey(seen, key), continue, end   % simple graph: dedupe parallels
        seen(key) = true;
        edges(end+1, :) = [a b];             %#ok<AGROW>
    end

    out.nodes = blocks(:)';
    out.edges = edges;
    if nargin >= 2 && ~isempty(jsonPath)
        fid = fopen(jsonPath, 'w');
        fwrite(fid, jsonencode(out));
        fclose(fid);
    end
end

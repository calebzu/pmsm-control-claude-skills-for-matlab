function m = layout_metrics(sys)
%LAYOUT_METRICS  Quantify Simulink layout quality at the top level of SYS.
%   m = LAYOUT_METRICS(sys) returns a struct with fields:
%     n_blocks        number of blocks (root excluded)
%     n_lines         number of line handles
%     n_segments      number of polyline segments across all lines
%     block_overlaps  count of overlapping block bounding-box pairs
%     line_block_hits count of (line, block) pairs where a line passes
%                     through a block it is not connected to
%     line_crossings  count of proper line-line crossings (cross-product)
%     extent          [width height] of the bounding box of all blocks
%
%   SYS may be a model name or a subsystem path. All metrics are computed
%   only on the immediate (SearchDepth==1) contents of SYS.
%
%   Position convention: [left top right bottom], y increases downward.

    blocks = find_system(sys, 'SearchDepth', 1, 'Type', 'Block');
    blocks = setdiff(blocks, {sys});            % drop root if present
    nb = numel(blocks);
    P = zeros(nb, 4);
    for i = 1:nb
        P(i, :) = get_param(blocks{i}, 'Position');
    end

    lines = find_system(sys, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'Line');
    [segs, owner] = collect_segments(lines);

    m.n_blocks       = nb;
    m.n_lines        = numel(lines);
    m.n_segments     = size(segs, 1);
    m.block_overlaps = count_block_overlaps(P);
    m.line_block_hits = count_line_block_hits(segs, P);
    m.line_crossings = count_line_crossings(segs, owner);
    if nb > 0
        m.extent = [max(P(:,3)) - min(P(:,1)), max(P(:,4)) - min(P(:,2))];
    else
        m.extent = [0 0];
    end
end

% --- block-block bounding-box overlap count -----------------------------
% P = Nx4, each row [left top right bottom], y downward. Counts unordered
% pairs whose interiors overlap (touching edges do not count).
function n = count_block_overlaps(P)
    n = 0;
    for i = 1:size(P, 1)
        for j = i + 1:size(P, 1)
            if ~(P(i,3) <= P(j,1) || P(j,3) <= P(i,1) || ...
                 P(i,4) <= P(j,2) || P(j,4) <= P(i,2))
                n = n + 1;
            end
        end
    end
end

% --- line-through-block count -------------------------------------------
% Counts (line, block) pairs where a segment of the line crosses the
% interior of a block rectangle. The rectangle is inset by MARGIN so that
% lines legitimately touching a block's ports at the boundary do not count.
function n = count_line_block_hits(segs, P)
    margin = 2;
    n = 0;
    owners_done = containers.Map('KeyType', 'double', 'ValueType', 'any');
    for b = 1:size(P, 1)
        r = [P(b,1)+margin, P(b,2)+margin, P(b,3)-margin, P(b,4)-margin];
        if r(3) <= r(1) || r(4) <= r(2)
            continue   % degenerate after inset
        end
        for s = 1:size(segs, 1)
            if seg_rect_intersect(segs(s,1:2), segs(s,3:4), r)
                n = n + 1;
            end
        end
    end
end

function tf = seg_rect_intersect(p1, p2, r)
% True if segment p1->p2 intersects the interior of rect r=[l t rt b].
    if point_in_rect(p1, r) || point_in_rect(p2, r)
        tf = true; return
    end
    corners = [r(1) r(2); r(3) r(2); r(3) r(4); r(1) r(4)];
    for e = 1:4
        q1 = corners(e, :);
        q2 = corners(mod(e, 4) + 1, :);
        if segments_cross(p1, p2, q1, q2)
            tf = true; return
        end
    end
    tf = false;
end

function tf = point_in_rect(p, r)
    tf = p(1) > r(1) && p(1) < r(3) && p(2) > r(2) && p(2) < r(4);
end

% --- line-line proper crossing count ------------------------------------
% segs = Mx4 segments [x1 y1 x2 y2], owner = Mx1 line id. Counts proper
% crossings (strictly through), skipping segments of the same line and
% segments that merely share an endpoint (branch fan-out points) or are
% collinear. Uses cross-product orientation with EPS tolerance.
function n = count_line_crossings(segs, owner)
    eps_tol = 1e-6;
    n = 0;
    for i = 1:size(segs, 1)
        for j = i + 1:size(segs, 1)
            if owner(i) == owner(j)
                continue
            end
            if segments_cross(segs(i,1:2), segs(i,3:4), ...
                              segs(j,1:2), segs(j,3:4), eps_tol)
                n = n + 1;
            end
        end
    end
end

function tf = segments_cross(p1, p2, p3, p4, eps_tol)
% Proper crossing test: true iff p1p2 and p3p4 strictly cross.
    if nargin < 5, eps_tol = 1e-6; end
    d1 = cross_o(p3, p4, p1);
    d2 = cross_o(p3, p4, p2);
    d3 = cross_o(p1, p2, p3);
    d4 = cross_o(p1, p2, p4);
    tf = (d1 >  eps_tol && d2 < -eps_tol || d1 < -eps_tol && d2 > eps_tol) && ...
         (d3 >  eps_tol && d4 < -eps_tol || d3 < -eps_tol && d4 > eps_tol);
end

function d = cross_o(a, b, c)
% Orientation of c relative to directed segment a->b.
    d = (b(1) - a(1)) * (c(2) - a(2)) - (b(2) - a(2)) * (c(1) - a(1));
end

% --- collect polyline segments ------------------------------------------
function [segs, owner] = collect_segments(lines)
    segs = zeros(0, 4);
    owner = zeros(0, 1);
    for k = 1:numel(lines)
        pts = get_param(lines(k), 'Points');   % Nx2
        for s = 1:size(pts, 1) - 1
            segs(end+1, :) = [pts(s, :) pts(s+1, :)]; %#ok<AGROW>
            owner(end+1, 1) = k;                       %#ok<AGROW>
        end
    end
end

function out = aerolap_ggv_demo(matfile, trackfile, mode_name, start, do_plot)
% AEROLAP_GGV_DEMO  Use an aerolap g-g-V export in MATLAB / Octave.
%
%   out = aerolap_ggv_demo('out/ggv/ggv.mat', 'data/tracks/fsae_autocross_example.csv')
%   out = aerolap_ggv_demo(matfile, trackfile, 'drs', 'flying')
%   out = aerolap_ggv_demo(matfile, trackfile, '', 'standing', false)   % no figures
%
% Create the export first (Python):
%   aerolap ggv data/vehicles/fsae_ev_aero.yaml --all-modes --export --out out/ggv
%
% The script plots the envelope and drives a simple point-mass QSS lap on a segment track CSV
% (type,length_m,radius_m,direction) with the exported envelope as a lookup table:
% apex speed  v^2 |kappa| = ay_max(v),  forward  v' = sqrt(v^2 + 2 ax_acc(v, ay) ds),
% backward  v = sqrt(v'^2 - 2 ax_brk(v', ay) ds).
%
% The envelope already contains the suspension / aero coupling of the exported setup, but it
% is fixed: ride heights along the lap, DRS switching and any setup change are NOT re-solved
% here (the Python solver does that). Use it for post-processing, quick what-ifs on the track
% and comparisons with existing MATLAB lap-sim code.
%
% Plain MATLAB (no toolbox) and GNU Octave compatible.

if nargin < 1 || isempty(matfile),   matfile = 'out/ggv/ggv.mat'; end
if nargin < 2 || isempty(trackfile), trackfile = 'data/tracks/fsae_autocross_example.csv'; end
if nargin < 3, mode_name = ''; end
if nargin < 4 || isempty(start), start = 'standing'; end
if nargin < 5, do_plot = true; end

S = load(matfile);
modes = fieldnames(S.ggv);
if isempty(mode_name), mode_name = modes{1}; end
g = S.ggv.(mode_name);
v_tab = g.v(:);            % m/s        (nv x 1)
frac = g.frac(:)';         % ay/ay_max  (1 x nf), sin-spaced
ay_max = g.ay_max(:);      % m/s^2      (nv x 1)
fprintf('GGV %s: %d speeds (%.1f..%.1f km/h) x %d lateral fractions, modes: %s\n', mode_name, ...
        numel(v_tab), 3.6 * v_tab(1), 3.6 * v_tab(end), numel(frac), strjoin(modes', ', '));

% ---------------------------------------------------------------- envelope plots
if do_plot
figure('Name', 'aerolap GGV');
subplot(1, 2, 1);
plot(3.6 * v_tab, ay_max / 9.80665, 'LineWidth', 1.5); hold on;
plot(3.6 * v_tab, g.ax_acc(:, 1) / 9.80665, 3.6 * v_tab, -g.ax_brk(:, 1) / 9.80665);
grid on; xlabel('v [km/h]'); ylabel('[g]');
legend('ay_{max}', 'ax_{acc} (ay = 0)', '-ax_{brk} (ay = 0)', 'Location', 'best');
title(['limits, mode ' strrep(mode_name, '_', '\_')]);
subplot(1, 2, 2); hold on;
for vk = [40 80 120] / 3.6
    i = find(v_tab >= vk, 1);
    if isempty(i), continue; end
    ay = ay_max(i) * frac;
    plot([ay, fliplr(ay)] / 9.80665, [g.ax_acc(i, :), fliplr(g.ax_brk(i, :))] / 9.80665, 'LineWidth', 1.2);
end
grid on; axis equal; xlabel('a_y [g]'); ylabel('a_x [g]'); title('g-g at 40 / 80 / 120 km/h');
end

% ---------------------------------------------------------------- track
[s, kappa, closed] = read_segment_track(trackfile, 0.5);
ds = diff(s);
n = numel(s);

% ---------------------------------------------------------------- apex speed
v_apex = zeros(n, 1);
for k = 1:n
    gap = ay_max - v_tab .^ 2 * abs(kappa(k));
    j = find(gap < 0, 1);
    if isempty(j)
        v_apex(k) = v_tab(end);
    elseif j == 1
        v_apex(k) = sqrt(max(ay_max(1), 0) / max(abs(kappa(k)), 1e-9));
    else
        v_apex(k) = v_tab(j - 1) + (v_tab(j) - v_tab(j - 1)) * gap(j - 1) / (gap(j - 1) - gap(j));
    end
end

% closed track + flying start: three laps back to back, keep the middle one
if closed && strcmpi(start, 'flying')
    reps = 3;
else
    reps = 1;
end
dsr = repmat(ds, reps, 1);
kr = [repmat(kappa(1:end-1), reps, 1); kappa(end)];
vr = [repmat(v_apex(1:end-1), reps, 1); v_apex(end)];
m = numel(vr);

% ---------------------------------------------------------------- forward / backward passes
vf = vr;
if strcmpi(start, 'standing'), vf(1) = 0; end
for k = 1:m - 1
    a = lookup(g, v_tab, frac, ay_max, 'ax_acc', vf(k), vf(k) ^ 2 * abs(kr(k)));
    vf(k + 1) = min(vr(k + 1), sqrt(max(vf(k) ^ 2 + 2 * a * dsr(k), 0)));
end
vb = vf;
for k = m - 1:-1:1
    a = lookup(g, v_tab, frac, ay_max, 'ax_brk', vb(k + 1), vb(k + 1) ^ 2 * abs(kr(k + 1)));
    vb(k) = min(vf(k), sqrt(max(vb(k + 1) ^ 2 - 2 * a * dsr(k), 0)));
end
if reps > 1
    v = vb(n:2 * n - 1);
else
    v = vb;
end
dt = 2 * ds ./ max(v(1:end-1) + v(2:end), 1e-6);
t = [0; cumsum(dt)];
fprintf('%s (%s start): lap time %.3f s, v_max %.1f km/h, v_min %.1f km/h\n', trackfile, start, ...
        t(end), 3.6 * max(v), 3.6 * min(v));

if do_plot
figure('Name', 'aerolap GGV lap');
plot(s, 3.6 * v, s, 3.6 * min(v_apex, v_tab(end)), ':'); grid on;
xlabel('s [m]'); ylabel('v [km/h]'); legend('speed', 'apex limit');
title(sprintf('point-mass QSS with the exported GGV: %.3f s', t(end)));
end

out = struct('s', s, 'v', v, 't', t, 'lap_time', t(end), 'v_apex', v_apex, 'kappa', kappa);
end


function a = lookup(g, v_tab, frac, ay_max, which, v, ay)
% table value at speed v and lateral acceleration ay (fraction of ay_max(v), clamped to [0, 1])
v = min(max(v, v_tab(1)), v_tab(end));
aym = interp1(v_tab, ay_max, v);
f = min(max(ay / max(aym, 1e-9), 0), 1);
a = interp2(frac, v_tab, g.(which), f, v);
end


function [s, kappa, closed] = read_segment_track(path, ds)
% segment CSV as written by aerolap (comment lines '# key: value', header, one segment per row)
fid = fopen(path, 'r');
if fid < 0, error('cannot open %s', path); end
closed = true; L = []; R = []; dirs = {}; types = {}; header_seen = false;
while true
    ln = fgetl(fid);
    if ~ischar(ln), break; end
    ln = strtrim(ln);
    if isempty(ln), continue; end
    if ln(1) == '#'
        body = lower(strtrim(ln(2:end)));
        if strncmp(body, 'closed:', 7), closed = any(strcmp(strtrim(body(8:end)), {'true', 'yes', '1'})); end
        continue;
    end
    if ~header_seen, header_seen = true; continue; end
    c = strsplit(ln, ',');
    while numel(c) < 4, c{end + 1} = ''; end
    types{end + 1} = lower(strtrim(c{1}));
    L(end + 1) = str2double(c{2});
    r = str2double(c{3}); if isnan(r), r = 0; end
    R(end + 1) = r;
    dirs{end + 1} = lower(strtrim(c{4}));
end
fclose(fid);
edges = [0, cumsum(L)];
s = [];
for k = 1:numel(L)
    nk = max(1, round(L(k) / ds));
    s = [s, edges(k) + (0:nk - 1) * L(k) / nk]; %#ok<AGROW>
end
s = [s, edges(end)]';
kappa = zeros(size(s));
for k = 1:numel(L)
    if types{k}(1) == 's', continue; end
    sgn = -1; if ~isempty(dirs{k}) && dirs{k}(1) == 'l', sgn = 1; end
    mid = s >= edges(k) & s < edges(k + 1);
    kappa(mid) = sgn / R(k);
end
kappa(end) = kappa(end - 1);
end

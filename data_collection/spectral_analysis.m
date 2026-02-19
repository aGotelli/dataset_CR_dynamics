function [freq, psd_db] = spectral_analysis(t, signals, varargin)
%SPECTRAL_ANALYSIS Compute (and plot) PSD for multiple signals.
%   [freq, psd_db] = spectral_analysis(t, signals, 'Name', value, ...)
%
%   Inputs:
%     t       : Nx1 time vector (seconds, sorted; can be non-uniform).
%     signals : NxM matrix, each column a signal.
%
%   Name-Value pairs (all optional):
%     'Labels'   : 1xM cellstr subplot titles.
%     'Title'    : figure super-title string.
%     'Fmax'     : x-axis limit [0 Fmax] Hz.
%     'Figure'   : existing figure handle to plot into.
%     'NperSeg'  : segment length for pwelch (default ~1 s).
%
%   Outputs:
%     freq   : Kx1 frequency vector (Hz).
%     psd_db : KxM PSD in dB/Hz.

% ---- Parse inputs (compatible with older MATLAB, no arguments block) ----
p = inputParser;
p.FunctionName = 'spectral_analysis';
addParameter(p, 'Labels', {}, @(c) iscell(c) || isempty(c));
addParameter(p, 'Title',  "", @(s) isstring(s) || ischar(s));
addParameter(p, 'Fmax',   [], @(x) isempty(x) || (isscalar(x) && x>0));
addParameter(p, 'Figure', [], @(h) isempty(h) || ishghandle(h));
addParameter(p, 'NperSeg', [], @(x) isempty(x) || (isscalar(x) && x>0));
addParameter(p, 'Plot', true, @(x) islogical(x) || isnumeric(x));
parse(p, varargin{:});

labels   = p.Results.Labels;
figTitle = string(p.Results.Title);
fmax     = p.Results.Fmax;
figH     = p.Results.Figure;
nperseg_user = p.Results.NperSeg;
doPlot = logical(p.Results.Plot);

% ---- Preprocess ----
t = t(:);
dt_med = median(diff(t));
fs = 1/dt_med;

sig = signals;
sig = fillmissing(sig, 'linear');
% sig = detrend(sig, 'constant');

% Welch window length: adaptive (~1 s, power-of-two), overrides pwelch default (256)
if isempty(nperseg_user)
    nperseg = 2^nextpow2(max(round(fs), 32)); % ~1 s window
else
    nperseg = nperseg_user;
end
nperseg = min(nperseg, size(sig,1)); % ensure segment length <= signal length
if nperseg < 4
    freq = [];
    psd_db = [];
    return;
end

% ---- PSD via Welch ----
num_signals = size(sig,2);
psd_db = [];
for k = 1:num_signals
    [pxx, freq] = pwelch(sig(:,k), hamming(nperseg), [], [], fs);
    if isempty(psd_db)
        psd_db = zeros(numel(freq), num_signals);
    end
    psd_db(:,k) = 10*log10(pxx);
end

% ---- Plot ----
if doPlot
    if isempty(figH)
        figH = figure('Name','Spectral analysis');
    else
        figure(figH); clf(figH);
    end

    rows = ceil(num_signals/2);
    cols = min(2, num_signals);
    for k = 1:num_signals
        subplot(rows, cols, k);
        plot(freq, psd_db(:,k), 'LineWidth', 1.6);
        grid on;
        xlabel('Frequency [Hz]', 'FontSize', 14);
        ylabel('PSD [dB/Hz]',   'FontSize', 14);
        if ~isempty(labels) && numel(labels) >= k
            title(labels{k}, 'FontSize', 14);
        end
        if ~isempty(fmax)
            xlim([0 fmax]);
        end
    end

    if strlength(figTitle) > 0
        sgtitle(figTitle, 'FontSize', 15, 'FontWeight', 'bold');
    end
    set(findall(figH,'-property','FontSize'),'FontSize',14);
end

end

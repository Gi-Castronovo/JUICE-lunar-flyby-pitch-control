function save_figure(fig, name, enabled)
%SAVE_FIGURE Exports a figure to the figures folder when enabled.

if enabled
    if ~exist('figures', 'dir'), mkdir('figures'); end
    exportgraphics(fig, fullfile('figures', [name '.png']), 'Resolution', 150);
end
end

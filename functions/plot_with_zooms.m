function fig = plot_with_zooms(t_min, y, y_label, plot_title, limits, zoom1, zoom2)
%PLOT_WITH_ZOOMS Full history on top, first 5 min and tCA +/- 10 min below.

fig = figure('Color', 'w', 'Position', [100 100 950 620]);
ranges = {[min(t_min), max(t_min)], zoom1, zoom2};
panels = {[1 2], 3, 4};
titles = {plot_title, 'First 5 minutes', 't_{CA} \pm 10 min'};

for k = 1:3
    subplot(2, 2, panels{k});
    idx = t_min >= ranges{k}(1) & t_min <= ranges{k}(2);
    plot(t_min(idx), y(idx), 'b', 'LineWidth', 1.5); hold on
    for j = 1:numel(limits)
        yline(limits(j), '--r', 'LineWidth', 1);
    end
    grid on
    xlim(ranges{k});
    xlabel('Time from closest approach [min]');
    ylabel(y_label);
    title(titles{k});
end
end

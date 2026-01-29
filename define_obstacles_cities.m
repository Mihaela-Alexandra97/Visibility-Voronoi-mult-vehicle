function [Obstacles, Cities, fig_handle] = define_obstacles_cities(env_bounds, n_obs, n_cities, e, l)
fig_handle = figure();
axis('equal');
axis(env_bounds);
box on
hold on

colors = ['k']; % obstacle fill color

%% Define obstacles
uiwait(msgbox(sprintf('\nFor defining an obstacle:\n\t - left-click = pick a vertex\n\t - right-click = pick last vertex\nObstacles should be convex and non-overlapping\n'), 'Obstacles', 'modal'));
Obstacles = cell(1, n_obs);

for i = 1:n_obs
    title(['Define obstacle #' num2str(i) ' from ' num2str(n_obs)]);
    counter = 1;
    button = 1;
    Obstacles{i}.vertices = [];
    indx_color = randi(length(colors), 1);
    Obstacles{i}.color = colors(indx_color);
    
    while button == 1
        [x, y, button] = ginput(1);
        if (x >= env_bounds(1) && x <= env_bounds(2) && y >= env_bounds(3) && y <= env_bounds(4))
            plot(x, y, '.k');
            Obstacles{i}.vertices = [Obstacles{i}.vertices [x; y]];
            label = sprintf('v%d%d', i, counter);
            text(x, y, label, 'FontSize', 9, 'Color', 'k', ...
                'VerticalAlignment', 'top', 'HorizontalAlignment', 'right');
            counter = counter + 1;
        end

        if button ~= 1
            try
                cv = convhull(Obstacles{i}.vertices(1,:), Obstacles{i}.vertices(2,:));
            catch
                uiwait(msgbox(sprintf('A convex hull cannot be obtained from the set of points.\nContinue with current obstacle')));
                button = 1;
            end
        end
    end

    cv(end) = [];
    Obstacles{i}.vertices = Obstacles{i}.vertices(:, cv);
    Obstacles{i}.handle = fill(Obstacles{i}.vertices(1,:), Obstacles{i}.vertices(2,:), Obstacles{i}.color);
    Obstacles{i}.center = [mean(Obstacles{i}.vertices(1,:)); mean(Obstacles{i}.vertices(2,:))];
    text(Obstacles{i}.center(1), Obstacles{i}.center(2), ...
         ['O_{' num2str(i) '}'], 'Color', 'w', 'FontWeight', 'bold', ...
         'HorizontalAlignment', 'center');
end

%% Define start and POIs
title('Define POIs and starting point');
uiwait(msgbox(sprintf('\nDefine starting point and POIs with mouse clicks:\n\t-points should be in the free space\n\t-See figure title for current point'), 'Cities', 'modal'));
Cities = NaN(2, n_cities);

for i = 1:n_cities
    if i == 1
        title('Define starting point p1');
    else
        title(['Define POI #' num2str(i) ' from ' num2str(n_cities)]);
    end
    
    good_point = 0;
    while good_point == 0
        [x, y] = ginput(1);
        good_point = 1;
        if ~(x >= env_bounds(1) && x <= env_bounds(2) && y >= env_bounds(3) && y <= env_bounds(4))
            good_point = 0;
            uiwait(msgbox(sprintf('\nWrong point - outside bounds.\nChoose again')));
        else
            for j = 1:n_obs
                if inpolygon(x, y, Obstacles{j}.vertices(1,[1:end 1]), Obstacles{j}.vertices(2,[1:end 1]))
                    good_point = 0;
                    uiwait(msgbox(sprintf('\nWrong point - inside obstacle %d.\nChoose again', j)));
                    break;
                end
            end
            if i >= 2 && ~isempty(intersect(Cities(:,1:i-1)', [x y], 'rows'))
                good_point = 0;
                uiwait(msgbox(sprintf('\nWrong point - identical to one previously defined.\nChoose again')));
            end
        end

        if good_point == 1
            Cities(:,i) = [x; y];
            if i == 1
                plot(x, y, 'ob');
                text(x, y, 'p_1', 'Color', 'k', 'FontWeight', 'bold', ...
                     'FontSize', 12, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
            else
                plot(x, y, 'pr');
                ei = e(i);
                li = l(i);
               label = sprintf('p_{%d}\n[%g,%g]', i, ei, li);
te = text(x, y, label, 'Color', 'k', 'FontWeight', 'bold', ...
         'FontSize', 14, 'HorizontalAlignment', 'center', ...
         'VerticalAlignment', 'top', 'Tag', 'TimeWindowLabel');
            end
        end
    end
end

title('');
end
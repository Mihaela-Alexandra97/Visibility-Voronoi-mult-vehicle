function [fig_handle] = extract_cities(env_bounds,n_cities,coord_data,e,l)
fig_handle = figure();
axis('equal');
% 
% max_x=max(coord_data(:,1))+10;max_y=max(coord_data(:,2))+10;
% env_bounds=[0 max_x 0 max_y];
axis(env_bounds);
box on
hold on
% n_obs=size(Obstacles,2);
% colors = ['k']; %if you want more random colors for obstacles, complete here

%% define starting point and cities
%left or right click picks a point
title('Define cities');
% uiwait(msgbox(sprintf('\nDefine starting point and cities with mouse clicks:\n\t-points should be in the free space\n\t-See fig''s title for current point'),'Cities','modal'));
Cities = NaN(2,n_cities); %matrix where each column contains the coordinates of a city.

for i=1:n_cities
    if i==1
        title('Define starting point');
    else
        title(['Define city #' num2str(i-1) ' from ' num2str(n_cities-1)]);
    end
    good_point = 0;
    while good_point == 0    %read point(s) until it is in free space
        x = coord_data(i,1);   
        y=coord_data(i,2);   
        good_point = 1;  %assume good point
        if ~(x >= env_bounds(1) && x <= env_bounds(2) && y >= env_bounds(3) && y <= env_bounds(4)) %inside bounds
            good_point = 0;
            uiwait(msgbox(sprintf('\nWrong point - outside bounds.\nChoose again')));
        else %inside boundaries
            % for j=1:n_obs
            %     if inpolygon(x,y,Obstacles{j}.vertices(1,[1:end 1]),Obstacles{j}.vertices(2,[1:end 1]))    %should close the polygon with first point as last
            %         good_point = 0;
            %         uiwait(msgbox(sprintf('\nWrong point - inside obstacle %d.\nChoose again',j)));
            %         break;
            %     end
            % end
            if i>=2 %test if it is a new point
                if ~isempty(intersect(Cities(:,1:i-1)',[x y],'rows'))
                    good_point = 0;
                    uiwait(msgbox(sprintf('\nWrong point - identical to one previously defined.\nChoose again')));
                end
            end
        end
        if good_point == 1   %good point
            if i==1
                plot(x,y,'ob'); %start point with blue circle
                text(x,y,'s','Color','k','FontWeight','bold','HorizontalAlignment','center','VerticalAlignment','top','FontSize',15);
            else
                plot(x,y,'dk','MarkerSize',15); %cities with red star
                text(x,y-1,['p_{' num2str(i-1) '}' '/' '[' num2str(e(i)),', ',num2str(l(i)) ']'],'Color','k','FontWeight','bold','HorizontalAlignment','center','VerticalAlignment','top','FontSize',15);
            end
             Cities(:,i) = [x;y];
        end
    end
end
% for i=1:n_cities
%     text(Cities(1,i),Cities(2,i)-3.15,['[' num2str(e(i)),', ',num2str(l(i)) ']'],'Color','k','FontWeight','normal','HorizontalAlignment','center','VerticalAlignment','top','FontSize',15);
% end

title('');
end

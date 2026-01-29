function [obstacles] = extract_obstacles(env_bounds,n_obs,fig_handle,Obstacles)
figure(fig_handle)
hold on
Extracted_Obstacles=cell(1,n_obs);
colors = ['k']; 
for i=1:n_obs
    Extracted_Obstacles{i}.vertices = [];
    indx_color = randi(length(colors),1);   %random obstacle color
    Extracted_Obstacles{i}.color = colors(indx_color);
    n=size(Obstacles{i}.vertices,2);
    j=1;
while j<=n
    x= Obstacles{i}.vertices(1,j);
    y=Obstacles{i}.vertices(2,j);
 if (x >= env_bounds(1) && x <= env_bounds(2) && y >= env_bounds(3) && y <= env_bounds(4)) %inside bounds
            plot(x,y,'.k');
           Extracted_Obstacles{i}.vertices = [Extracted_Obstacles{i}.vertices [x;y]];
        % cv = convhull(Extracted_Obstacles{i}.vertices(1,:),Extracted_Obstacles{i}.vertices(2,:));
 end
          % cv = convhull(Extracted_Obstacles{i}.vertices(1,:),Extracted_Obstacles{i}.vertices(2,:));
            j=j+1;
end
    cv = convhull(Extracted_Obstacles{i}.vertices(1,:),Extracted_Obstacles{i}.vertices(2,:));
  cv(end) = [];   %remove duplicated vertex (will close the polygon later
    Extracted_Obstacles{i}.vertices = Extracted_Obstacles{i}.vertices(:,cv);
   Extracted_Obstacles{i}.handle = fill(Extracted_Obstacles{i}.vertices(1,:),Extracted_Obstacles{i}.vertices(2,:),Extracted_Obstacles{i}.color);
   
    Extracted_Obstacles{i}.center = [mean(Extracted_Obstacles{i}.vertices(1,:)); mean(Extracted_Obstacles{i}.vertices(2,:))];
    text(Extracted_Obstacles{i}.center(1),Extracted_Obstacles{i}.center(2),['O_{' num2str(i) '}'],'Color','w','FontWeight','bold','HorizontalAlignment','center');
    
end
for i=1:n_obs
obstacles{i}=Extracted_Obstacles{i}.vertices;
end
end
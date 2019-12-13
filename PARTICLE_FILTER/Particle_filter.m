% Copyright © 2019 Giovanni Miraglia
% 
% Permission is hereby granted, free of charge, to any person obtaining a 
% copy of this software and associated documentation files (the “Software”), 
% to deal in the Software without restriction, including without limitation 
% the rights to use, copy, modify, merge, publish, distribute, sublicense, 
% and/or sell copies of the Software, and to permit persons to whom the 
% Software is furnished to do so, subject to the following conditions: 
% The above copyright notice and this permission notice shall be included in 
% all copies or substantial portions of the Software. 
% THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS 
% OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
% THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

classdef Particle_filter<handle
    properties
        rows
        cols
        grid_map
        N_samples
        cell_size
        angle_resolution
        n_angles
        resample_rate %from 0 to 1
        x_pos % in mm
        y_pos % in mm
        th_pos % 0->360 deg
        particles
        sigma_measure % in mm
        sigma_resample_pos % in mm
        sigma_resample_angle % in mm
        handle_pos = NaN;
        handle_particles = NaN;
    end
    
    methods
        function this_loc = Particle_filter(file_name,cell_size,n_angles,N_samples,...
                resample_rate,sigma_measure,sigma_resample_pos,sigma_resample_angle)
            
            if nargin == 8     
                % LOAD MAP
                this_loc.grid_map = csvread(file_name);
                
                this_loc.cell_size = cell_size;
                this_loc.n_angles = n_angles;
                this_loc.angle_resolution = 360/n_angles;
                this_loc.resample_rate = resample_rate;
                this_loc.N_samples = N_samples;
                this_loc.sigma_measure = sigma_measure;
                this_loc.sigma_resample_pos = sigma_resample_pos;
                this_loc.sigma_resample_angle = sigma_resample_angle;
                
                [this_loc.rows,this_loc.cols] = size(this_loc.grid_map);
                                
                % INITIALIZE PARTICLES
                this_loc = init_particles(this_loc);
                
                % INIT LOCATION    
                this_loc.update_estimation();            
            elseif nargin~=0
                error('wrong number of parameters');
            end
        end
        
        function [distance, end_point_x, end_point_y] = ray_tracing(this_loc,x_pos,y_pos,direction)

            max_row = this_loc.rows;
            max_col = this_loc.cols;
            
            start_row = floor(y_pos/this_loc.cell_size)+1;
            start_col = floor(x_pos/this_loc.cell_size)+1;
            
            % check for 90 270 180 0 360   
            direction = wrapTo360(direction);
            if direction == 90 || direction == 180 || direction == 0 ||...
                    direction == 360 || direction == 270  
                direction = wrapTo360(direction+eps(1000));
            end

            % position out of map
            if start_row<1 || start_row>max_row || start_col<1 || start_col>max_col
                distance = NaN;
                end_point_x = NaN;
                end_point_y = NaN;
                return;
            end

            if this_loc.grid_map(start_row,start_col)~=0 %inside obstacle
                distance = NaN;
                end_point_x = NaN;
                end_point_y = NaN;
                return;
            end

            inc_col = sign(cosd(direction));
            inc_row = sign(sind(direction));


            % horizontal scan

            if inc_row>0 %first delta y
                delta_y1 = this_loc.cell_size*start_row - y_pos;
            else
                delta_y1 = this_loc.cell_size*(start_row-1) - y_pos;
            end

            delta_x1 = delta_y1/tand(direction);  %first delta x  

            y_step = inc_row * this_loc.cell_size; 
            x_step = y_step/tand(direction); %delta x for each horizontal line crossing

            current_x = x_pos + delta_x1;
            current_y = y_pos + delta_y1;

            current_row = start_row + inc_row;
            current_col = floor(current_x/this_loc.cell_size)+1;

            while true
                if current_col<1 || current_col>max_col
                    x_intersection_h = NaN;
                    y_intersection_h = NaN;
                    break;
                elseif current_row<1 || current_row>max_row
                    x_intersection_h = current_x;
                    y_intersection_h = current_y;
                    break;
                elseif this_loc.grid_map(current_row,current_col)~=0
                    x_intersection_h = current_x;
                    y_intersection_h = current_y;
                    break;
                end

                current_x = current_x + x_step;
                current_y = current_y + y_step;
                current_col = floor(current_x/this_loc.cell_size)+1;
                current_row = current_row + inc_row;        
            end

            % vertical scan

            if inc_col>0 %first delta x
                delta_x1 = this_loc.cell_size*start_col - x_pos;
            else
                delta_x1 = this_loc.cell_size*(start_col-1) - x_pos;
            end

            delta_y1 = delta_x1*tand(direction);  %first delta y  

            x_step = inc_col * this_loc.cell_size; 
            y_step = x_step*tand(direction); %delta x for each horizontal line crossing

            current_x = x_pos + delta_x1;
            current_y = y_pos + delta_y1;

            current_col = start_col + inc_col;
            current_row = floor(current_y/this_loc.cell_size)+1;

            while true
                if current_row<1 || current_row>max_row
                    x_intersection_v = NaN;
                    y_intersection_v = NaN;
                    break;
                elseif current_col<1 || current_col>max_col
                    x_intersection_v = current_x;
                    y_intersection_v = current_y;
                    break;
                elseif this_loc.grid_map(current_row,current_col)~=0
                    x_intersection_v = current_x;
                    y_intersection_v = current_y;
                    break;
                end

                current_x = current_x + x_step;
                current_y = current_y + y_step;
                current_row = floor(current_y/this_loc.cell_size)+1;
                current_col = current_col + inc_col;        
            end

            %compute distances

            dist_h = norm([(x_pos-x_intersection_h) (y_pos-y_intersection_h)]); 
            dist_v = norm([(x_pos-x_intersection_v) (y_pos-y_intersection_v)]); 

            [~,index] = min([dist_h dist_v]);

            if index == 1
                distance = dist_h;
                end_point_x = x_intersection_h;
                end_point_y = y_intersection_h;
            else
                distance = dist_v;
                end_point_x = x_intersection_v;
                end_point_y = y_intersection_v;
            end
        end
        
        function plot_map(this_loc)
            boundaries = polyshape([0 this_loc.cell_size*this_loc.cols...
                this_loc.cell_size*this_loc.cols 0],...
                [0 0 this_loc.cell_size*this_loc.rows this_loc.cell_size*this_loc.rows]);

            figure(1);
            clf;
            plot(boundaries);
            axis equal;
            max_lim = max(this_loc.rows,this_loc.cols)*this_loc.cell_size;
            
            xlim([-10*this_loc.cell_size max_lim + 10*this_loc.cell_size]);
            ylim([-10*this_loc.cell_size max_lim + 10*this_loc.cell_size]);
            
            hold on;
            
            help_map = this_loc.grid_map;
            help_map(help_map ~= 0) = 1;
            help_map(help_map == 0) = NaN;
            
            xx = this_loc.cell_size/2: this_loc.cell_size:this_loc.cell_size*this_loc.cols - (this_loc.cell_size/2);
            yy = this_loc.cell_size/2: this_loc.cell_size:this_loc.cell_size*this_loc.rows - (this_loc.cell_size/2);
            [XX, YY] = meshgrid(xx, yy);
            XX = XX.*help_map;
            YY = YY.*help_map;
            scatter(XX(:),YY(:),'k','filled')
            %for i = 1:this_loc.rows
            %    for j = 1:this_loc.cols
            %        if this_loc.grid_map(i,j)~=0
            %            fill([this_loc.cell_size*(j-1) this_loc.cell_size*j...
            %                this_loc.cell_size*j this_loc.cell_size*(j-1)],...
            %            [this_loc.cell_size*(i-1) this_loc.cell_size*(i-1)...
            %            this_loc.cell_size*i this_loc.cell_size*i],'r');
            %        end
            %   end
            %end
        end
        
        function show_location(this_loc)        
            
            if ~ishandle(this_loc.handle_pos)
                figure(1);
                this_loc.handle_pos = quiver(this_loc.x_pos,this_loc.y_pos,this_loc.cell_size*cosd(this_loc.th_pos),...
                    this_loc.cell_size* sind(this_loc.x_pos),'AutoScale','off','LineWidth',5);  
                this_loc.handle_pos.Color = 'y';
            else
                this_loc.handle_pos.XData = this_loc.x_pos;
                this_loc.handle_pos.YData = this_loc.y_pos;
                this_loc.handle_pos.UData = this_loc.cell_size*cosd(this_loc.th_pos);
                this_loc.handle_pos.VData = this_loc.cell_size*sind(this_loc.th_pos);
            end
            
        end
                        
        function this_loc = init_particles(this_loc)
            this_loc.particles = NaN(this_loc.N_samples,4+this_loc.n_angles);
            weight = 1/this_loc.N_samples;
            max_x = 0.99998*(this_loc.cols*this_loc.cell_size);
            max_y = 0.99998*(this_loc.rows*this_loc.cell_size);
                        
            for i = 1:this_loc.N_samples
                
                this_loc.particles(i,3) = 359*rand();
                this_loc.particles(i,4) = weight;
                
                this_loc.particles(i,1) = max_x*rand()+0.00001;
                this_loc.particles(i,2) = max_y*rand()+0.00001;
                ind_row = floor(this_loc.particles(i,2)/this_loc.cell_size)+1;
                ind_col = floor(this_loc.particles(i,1)/this_loc.cell_size)+1;
                                
                while(this_loc.grid_map(ind_row,ind_col)~=0) %avoid obstacles
                    this_loc.particles(i,1) = max_x*rand()+0.00001;
                    this_loc.particles(i,2) = max_y*rand()+0.00001;
                    ind_row = floor(this_loc.particles(i,2)/this_loc.cell_size)+1;
                    ind_col = floor(this_loc.particles(i,1)/this_loc.cell_size)+1;                                
                end
                
                vector_scans = this_loc.get_scans(this_loc.particles(i,1),...
                    this_loc.particles(i,2),this_loc.particles(i,3));
                
                this_loc.particles(i,5:end) = vector_scans;
                
            end
        end
        
        function vector_scans = get_scans(this_loc,x,y,th)
            vector_scans = NaN(1,this_loc.n_angles);
            for i=1:this_loc.n_angles
                [vector_scans(i),~,~] = this_loc.ray_tracing(x,y,th);
                th = th + this_loc.angle_resolution;
                mod(th,360);
                
            end
        end
        
        function show_particles(this_loc)
                       
            if ~ishandle(this_loc.handle_particles)
                figure(1);
                this_loc.handle_particles = scatter(this_loc.particles(:,1), this_loc.particles(:,2),'.','r');        
            else
                this_loc.handle_particles.XData = this_loc.particles(:,1);
                this_loc.handle_particles.YData = this_loc.particles(:,2);
            end
        end
        
        function move_particles(this_loc,delta_x,delta_y,delta_theta)
            max_x = this_loc.cols*this_loc.cell_size;
            max_y = this_loc.rows*this_loc.cell_size;
            nan_vec = NaN(1,4+this_loc.n_angles);
            
            for i = 1:this_loc.N_samples
                this_loc.particles(i,1) = this_loc.particles(i,1) + delta_x;
                this_loc.particles(i,2) = this_loc.particles(i,2) + delta_y;
                this_loc.particles(i,3) = wrapTo360(this_loc.particles(i,3) + delta_theta);
                
                ind_row = floor(this_loc.particles(i,2)/this_loc.cell_size)+1;
                ind_col = floor(this_loc.particles(i,1)/this_loc.cell_size)+1;
                
                if(this_loc.particles(i,1) <= 0 ||...
                    this_loc.particles(i,1) >= max_x||...
                    this_loc.particles(i,2) <= 0 ||...
                    this_loc.particles(i,2) >= max_y)
                
                    this_loc.particles(i,:) = nan_vec; %outside map
                elseif this_loc.grid_map(ind_row,ind_col) ~=0
                    this_loc.particles(i,:) = nan_vec; %inside obstacle
                end
            end
        end
        
        function weight = weight_computation(this_loc,particle_vector,measurements_vector)
            
            if length(particle_vector)~= length(measurements_vector)
                error('measurements from Lidar must have same resolution of particle filter!');
            end
            
            weight = 1;

            for i = 1:length(particle_vector)
                if isnan(particle_vector(i))|| isnan(measurements_vector(i))
                    continue;
                end
                
                difference = abs(particle_vector(i)-measurements_vector(i)); %assume mm

                prob = normpdf(difference, 0, this_loc.sigma_measure);
                %correct way but very slow:
                %prob = 1-(cdf('Normal',difference,0,this_loc.sigma_measure)-cdf('Normal',-difference,0,this_loc.sigma_measure));

                weight = weight*prob;

            end
            
        end
                
        function re_sample(this_loc,measurements_vec)
            max_x_GEN = 0.99998*this_loc.cols*this_loc.cell_size;
            max_y_GEN = 0.99998*this_loc.rows*this_loc.cell_size;
            
            N_old_samples = round(this_loc.N_samples * (1-this_loc.resample_rate));
                        
            new_particles = NaN(size(this_loc.particles));
            
            this_loc.particles(isnan(this_loc.particles(:,4)),:)=[]; %remove NaN entries
                        
            %evaluate old particles
            for i = 1:length(this_loc.particles(:,1))
                %compute weight particle besed on distance measurments
                this_loc.particles(i,4) = this_loc.weight_computation(this_loc.particles(i,5:end),measurements_vec);
            end
            
            %normalize weights
            total_weight = sum(this_loc.particles(:,4));
            this_loc.particles(:,4) = round((this_loc.particles(:,4)/total_weight)*N_old_samples);
                        
            %resampling around old samples according to weight
            index = 1;
            weight = 1/this_loc.N_samples;
            
            for i = 1: length(this_loc.particles(:,4))
                for j = 1:this_loc.particles(i,4)
                    thth = wrapTo360(normrnd(this_loc.particles(i,3),this_loc.sigma_resample_angle));
                    
                    xx = abs(normrnd(this_loc.particles(i,1),this_loc.sigma_resample_pos));
                    yy = abs(normrnd(this_loc.particles(i,2),this_loc.sigma_resample_pos));
                                        
                    if xx >max_x_GEN
                        xx = max_x_GEN;
                    elseif xx<=0
                        xx = 0.00001;
                    end
                    
                    if yy >max_y_GEN
                        yy = max_y_GEN;
                    elseif yy<=0
                        yy = 0.00001;
                    end
                    
                    ind_row = floor(yy/this_loc.cell_size)+1;
                    ind_col = floor(xx/this_loc.cell_size)+1;   
                    
                    while(this_loc.grid_map(ind_row,ind_col)~=0) %avoid obstacles
                        xx = abs(normrnd(this_loc.particles(i,1),this_loc.sigma_resample_pos));
                        yy = abs(normrnd(this_loc.particles(i,2),this_loc.sigma_resample_pos));

                        if xx >max_x_GEN
                            xx = max_x_GEN;
                        elseif xx<=0
                            xx = 0.00001;
                        end

                        if yy >max_y_GEN
                            yy = max_y_GEN;
                        elseif yy<=0
                            yy = 0.00001;
                        end

                        ind_row = floor(yy/this_loc.cell_size)+1;
                        ind_col = floor(xx/this_loc.cell_size)+1;                                
                    end                  
                    new_particles(index,1) = xx;
                    new_particles(index,2) = yy;
                    new_particles(index,3) = thth;
                    
                    vector_scans = this_loc.get_scans(new_particles(index,1),...
                    new_particles(index,2),new_particles(index,3));
                
                    new_particles(index,5:end) = vector_scans;
                    new_particles(index,4) = weight; %not really necessary, just for future developments    
                    index = index+1;
                    
                    if index>N_old_samples
                        break;
                    end
                end
            end               
                                    
            for i = index: this_loc.N_samples
                
                new_particles(i,3) = 359*rand();
                new_particles(i,4) = weight;
                
                new_particles(i,1) = max_x_GEN*rand()+0.00001;
                new_particles(i,2) = max_y_GEN*rand()+0.00001;
                
                ind_row = floor(new_particles(i,2)/this_loc.cell_size)+1;
                ind_col = floor(new_particles(i,1)/this_loc.cell_size)+1;

                while(this_loc.grid_map(ind_row,ind_col)~=0) %avoid obstacles
                    new_particles(i,1) = max_x_GEN*rand()+0.00001;
                    new_particles(i,2) = max_y_GEN*rand()+0.00001;
                    ind_row = floor(new_particles(i,2)/this_loc.cell_size)+1;
                    ind_col = floor(new_particles(i,1)/this_loc.cell_size)+1;                                
                end

                vector_scans = this_loc.get_scans(new_particles(i,1),...
                    new_particles(i,2),new_particles(i,3));
                
                new_particles(i,5:end) = vector_scans;
            end
            
            this_loc.particles = new_particles;
        end
        
        function update_estimation(this_loc)

            %use k-means clustering to find position and orientation           
            [vec_pos,C_pos] = kmeans(this_loc.particles(:,1:2),2);
            [vec_th,C_th] = kmeans(this_loc.particles(:,3),2);

            ind_pos = mode(vec_pos);
            ind_th = mode(vec_th);
            
            this_loc.y_pos = C_pos(ind_pos,2);
            this_loc.x_pos =  C_pos(ind_pos,1);
            this_loc.th_pos = C_th(ind_th);
        end
        
        function update_state(this_loc,delta_x,delta_y,delta_theta,distance_vec)
            
            %move particles
            this_loc.move_particles(delta_x,delta_y,delta_theta);
       
            %resample
            this_loc.re_sample(distance_vec);
            
            %update estimation
            this_loc.update_estimation();
        end
            
    end
end


clear;

map_file_name = 'test_map3a.csv'; %csv occupancy map (0 is free region)
cell_resolution = 50; %mm
number_of_scans = 24; 
number_of_particles = 1000;
resampling = 0.2; %from 0 to 1

%this is used when assigning weights to particles
sigma_measurments = 50; %mm

%this are used when resampling around particles
sigma_angle = 8; %deg
sigma_pos = 20; %mm

% noise in lidar 
noise_sigma = 30; %mm

%delta_t is only used for plotting (it is not used in particle filter!)
delta_t = 0.1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

a = Particle_filter(map_file_name,cell_resolution,number_of_scans,number_of_particles,...
    resampling,sigma_measurments,sigma_pos,sigma_angle);

a.plot_map();
a.show_particles();

current_x = 0;%5*cell_resolution;
current_y = 0;%5*cell_resolution;
current_th = 0; %-90;

delta_x = 0;%.1*cell_resolution;
delta_y = 0;%0*cell_resolution;
delta_theta = 0;

h_pos = NaN;

t = tcpip('0.0.0.0', 42069, 'NetworkRole', 'server');
disp('here 1')
fopen(t);
disp('here 2')


flag = true;
while flag
        if t.BytesAvailable > 0
            t.BytesAvailable
            raw_data = fread(t, t.BytesAvailable);
            data = typecast(uint8(raw_data), 'double');
            current_x = data(1);
            current_y = data(2);
            current_th = data(3);
            flag = false;
        end
end


while true
    
    %try 
    current_x = current_x + delta_x;
    current_y = current_y + delta_y;
    current_th = current_th + delta_theta;
    
    %mesures = get_scans_vector(a.grid_map,a.n_angles,a.cell_size,current_x,current_y,current_th); %get scans
    %mesures = mesures+normrnd(0,noise_sigma,[1 a.n_angles]) %add noise
    mesures = [];
    angles= [];
    
    flag = true;
    while flag
        if t.BytesAvailable > 0
            t.BytesAvailable
            raw_data = fread(t, t.BytesAvailable);
            data = typecast(uint8(raw_data), 'double');
            
            delta_x = data(1)
            delta_y = data(2)
            delta_theta = data(3) %assumes incomeing data is counter clockwise
            
            mesures = data(end-(2*number_of_scans-1):2:end); %force it to give only last scan
            angles = data(end-(2*number_of_scans-2):2:end); % force it to give only last scan)
            
%             mesures = data(end-(2*number_of_scans-1):2:end); %force it to give only last scan
%             angles = data(end-(2*number_of_scans-2):2:end); % force it to give only last scan)
            flag = false;
        end
        pause(.02);
    end
    
    %mesures = circshift(mesures,2);
    
    
    
    a.update_state(delta_x,delta_y,delta_theta,mesures);
    
   a.show_particles(); 
   a.show_location();
    
    vert_x = current_x;
    vert_y = current_y;
    %current_th = a.th_pos;
    
    dataOut = typecast([a.x_pos,a.y_pos,a.th_pos],'uint8');
    fwrite(t,dataOut);
    
    for j =1:a.n_angles
        vert_x = [vert_x (current_x + mesures(j)*cosd(current_th+(j-1)*a.angle_resolution)) current_x];
        vert_y = [vert_y (current_y + mesures(j)*sind(current_th+(j-1)*a.angle_resolution)) current_y];
    end    

    if~ishandle(h_pos)
        h_pos = quiver(current_x,current_y,a.cell_size*cosd(current_th),a.cell_size*sind(current_th),'AutoScale','off','LineWidth',5,'Color','b');
        h_plot_ray = plot(vert_x,vert_y,'b');
    else
        h_pos.XData = current_x;
        h_pos.YData = current_y;
        h_pos.UData = a.cell_size*cosd(current_th);
        h_pos.VData = a.cell_size*sind(current_th);
        h_plot_ray.XData = vert_x;
        h_plot_ray.YData = vert_y;
    end
    
    %disp('Test')
    %disp(current_th)
    %disp(a.th_pos)
    fprintf("%f.1 %f.1 %f.1\n", a.x_pos, a.y_pos, a.th_pos);
    %a.x_pos
    %a.y_pos
    %a.th_pos

    
    %catch 
    %    disp('Bad read')
    %end
    
    %pause(delta_t);
end
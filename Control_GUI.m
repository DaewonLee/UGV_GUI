function varargout = Control_GUI(varargin)
% CONTROL_GUI MATLAB code for Control_GUI.fig
%      CONTROL_GUI, by itself, creates a new CONTROL_GUI or raises the existing
%      singleton*.
%
%      H = CONTROL_GUI returns the handle to a new CONTROL_GUI or the handle to
%      the existing singleton*.
%
%      CONTROL_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CONTROL_GUI.M with the given input arguments.
%
%      CONTROL_GUI('Property','Value',...) creates a new CONTROL_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Control_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Control_GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Control_GUI

% Last Modified by GUIDE v2.5 27-Sep-2017 17:35:21

global g_obs_cell;
% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Control_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @Control_GUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

% --- Executes just before Control_GUI is made visible.
function Control_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Control_GUI (see VARARGIN)
addpath('astar','lib');
global is_realtime;

is_realtime = true;

global msg_gps;   
global cnt_h_pos;
global g_obs_cell;
   clear g_obs_cell;
   global bias;
   global g_map_boundary_lat;
   global g_map_boundary_lon;
   global test_timer;
   global idx_path;
   global cnt_plot;
   cnt_plot = 0;
   global modePub;
   global msgMode; % control mode
   global isGoal;
   
   global lookPub;
   global msglook; % look
   
   global bias;
bias.lon = 0;
bias.lat = 0;
   global WPPub;
   global msgWP; % WP39.952739, -75.189687 39.951567, -75.191655
   %g_map_boundary_lat = [39.952739    39.951567   ];  
   %g_map_boundary_lon = [-75.189687    -75.191655   ];
%    g_map_boundary_lat = [39.9515015    39.9511602   ]; %test1.png
%    g_map_boundary_lon = [-75.189214    -75.1902051   ];
   
%    g_map_boundary_lat = [39.951062    39.949707   ]; %pennpart1.png
%    g_map_boundary_lon = [-75.184125    -75.186333   ];

  
  % g_map_boundary_lat = [43.230268  43.229326   ]; %nuair.png
  % g_map_boundary_lon = [-75.418484   -75.420925   ];

%   g_map_boundary_lat = [43.217082  43.216418   ]; %nuair_test.png
%   g_map_boundary_lon = [-75.393875   -75.394937   ];
%    
%    g_map_boundary_lat = [43.230899  43.229507   ]; %tri_R.png
%    g_map_boundary_lon = [-75.406233   -75.408765   ];
%    
 %   g_map_boundary_lat = [43.230619  43.229093   ]; %tri_L.png
 %   g_map_boundary_lon = [-75.425576   -75.428550   ];
   

  % g_map_boundary_lat = [39.9504  39.9499   ]; %pennpart3.png
  % g_map_boundary_lon = [-75.1848    -75.1857   ];
   
  %  g_map_boundary_lat = [39.951376    39.950003   ]; %pennpart2.png
  %  g_map_boundary_lon = [-75.184952    -75.187266   ];
   
   %39.951062, -75.184125, 39.949707, -75.186333
   %39.951376, -75.184952  39.950003, -75.187266
   
%g_map_boundary_lat = [39.951748    39.950945   ]; % shoemakergreen.png
%g_map_boundary_lon = [-75.189046    -75.190390   ];

   
%g_map_boundary_lat = [39.952582    39.951910   ]; % moorebuilding.png
%g_map_boundary_lon = [-75.190025    -75.191332   ];.

g_map_boundary_lat = [32.014932    32.010429   ]; % lane3.png
g_map_boundary_lon = [-81.824915    -81.830172   ];

   plot(g_map_boundary_lon,g_map_boundary_lat,'.')
   plot_google_map
   
%[x,y] = ginput(4)

% Choose default command line output for Control_GUI
handles.output = hObject;

rosshutdown;

%MasterIP =  '192.168.0.21';    
MasterIP =  'http://daewon:11311/';
% rostopic pub -r 10 /mavros/global_position/raw/fix sensor_msgs/NavSatFix "header:

% rostopic pub -r 10 /Jackal_GPS sensor_msgs/NavSatFix "header:
%   seq: 0
%   stamp: {secs: 0, nsecs: 0}
%   frame_id: ''
% status: {status: 0, service: 0}
% latitude: 32.014768
% longitude: -81.825183
% altitude: 0.0
% position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
% position_covariance_type: 0"

%rostopic pub -r 10 /Jackal_heading std_msgs/Float64 "data: 0.0" 




rosinit(MasterIP)
idx_path = 1;
cnt_h_pos = 0;
isGoal = 0;

modePub = rospublisher('/jackal_mode','std_msgs/Int8');
msgMode = rosmessage(modePub);

lookPub = rospublisher('/jackal_look','std_msgs/Float64MultiArray');
msglook = rosmessage(lookPub);

WPPub = rospublisher('/jackal_waypoint','std_msgs/Float64MultiArray');
msgWP = rosmessage(WPPub);

global gps;
global heading;
%gps = rossubscriber('/navsat/fix', @GPSCallback); % <<<<<<<<<<<====
%gps = rossubscriber('/mavros/global_position/raw/fix', @GPSCallback); % <<<<<<<<<<<====
gps = rossubscriber('/Jackal_GPS', @GPSCallback); % <<<<<<<<<<<====
heading = rossubscriber('/Jackal_heading');



global bias;
bias = struct('lon',0,'lat',0,'heading',0);


guidata(hObject, handles);


function GPSCallback(src, message)
    %%ROS callback function for recieving raw GPS data

    global WPPub;
    global msgWP; % WP
    global path_s;
    global bias;
    global msg_gps;
    global idx_path;
    global isFollowPath;
    global modePub;
    global msgMode;
    global isGoal;
    
    msg_gps = message;

    xy_ = llToMeters(msg_gps.Longitude, msg_gps.Latitude);

    if(~isempty(bias))
        xy(1) = xy_(1) + bias.lon;
        xy(2) = xy_(2) + bias.lat;
    else
        xy(1) = xy_(1);
        xy(2) = xy_(2);
    end

   
    ll = metersToll(xy);
    msg_gps.Longitude = ll(1);
    msg_gps.Latitude = ll(2);
    %fprintf('Latitude: %f, Longitude: %f \n', msg_gps.Latitude, msg_gps.Longitude);
   % pause(1);
    %drawnow;
    
 update_display();
 thresh_dist = 3.5;
 %%
 if (isFollowPath == 1)
     if (~isempty(path_s))
         if (msgMode.Data == 1)
            idx_max = size(path_s,1);

            dist = findDistance(xy_,path_s(idx_path,:));
            if (dist < thresh_dist)
                if (idx_path == idx_max)
                    msgMode.Data = 0;
                    isGoal = true;
                    fprintf('The robot is at the goal position!\n');
                else
                    idx_path = idx_path + 1;
                end                  
            end
            wp_ll = metersToll(path_s(idx_path,:));
            fprintf('wp_index: %d\n',idx_path);

            %     plot(wp_ll(1), wp_ll(2),'--gs',...
            %     'LineWidth',2,...
            %     'MarkerSize',10,...
            %     'MarkerEdgeColor','b',...
            %     'MarkerFaceColor',[0.5,0.5,0.5])
            msgWP.Data = [wp_ll(1), wp_ll(2)];
            send(WPPub,msgWP);
         end
     end
 end
 
 send(modePub,msgMode);
 %%



% START USER CODE
function update_display(hObject, eventdata, handles)
% Timer timer1 callback, called each time timer iterates.
% Gets surface Z data, adds noise, and writes it back to surface object.
global msg_gps;
global h_start;
global heading;
global tri;
global idx_path;
global path_s;
global sight;
global msgMode;
global h_sight;
global isGoal;
global bias;
%%
global cnt_plot;
cnt_plot = cnt_plot + 1;
if (cnt_plot == 10)
    if(~isempty(path_s))
        for i = 1: idx_path
            path_ll(i,:) =  metersToll(path_s(i,:));
%             hold on
%             plot(path_ll(i,1), path_ll(i,2),'--rs',...
%             'LineWidth',2,...
%             'MarkerSize',10,...
%             'MarkerEdgeColor','b',...
%             'MarkerFaceColor',[0.5,0.5,0.5]);
%             drawnow
        end
    end
    cnt_plot = 0;
    drawnow;
end
%%

msg_heading = receive(heading,1);
% if (~isempty(bias))
%     bias
%     msg_heading.Data = msg_heading.Data + bias.heading;
% end
psi = msg_heading.Data * pi/180;
rot = [cos(psi), sin(psi); -sin(psi), cos(psi)];

if(~isempty(tri))
    tri_rot = tri * rot;

    tri_x(:) = tri_rot(:,1)+msg_gps.Longitude;
    tri_y(:) = tri_rot(:,2)+msg_gps.Latitude;

     set(h_start,'XData',tri_x);
     set(h_start,'YData',tri_y);
     if(isGoal)
         set(h_start,'facecolor','b');
     end
end
 
 if (msgMode.Data == 2)
     if(~isempty(h_sight))
        sight_rot = sight * rot;
        sight_x(:) = sight_rot(:,1)+msg_gps.Longitude;
        sight_y(:) = sight_rot(:,2)+msg_gps.Latitude;

        set(h_sight,'XData',sight_x);
        set(h_sight,'YData',sight_y);
     end
 else
     delete(h_sight)
 end
 
 
 
 
 
 
 

% END USER CODE


% --- Outputs from this function are returned to the command line.
function varargout = Control_GUI_OutputFcn(hObject, eventdata, handles) 
varargout{1} = handles.output;


% --- Executes on button press in sattlite.
function sattlite_Callback(hObject, eventdata, handles)
   
   
  plot_google_map('maptype', 'satellite')



% --- Executes on button press in roadmap.
function roadmap_Callback(hObject, eventdata, handles)
   
   
  plot_google_map('MapType', 'roadmap') 




% --- Executes on button press in aStart.
function aStart_Callback(hObject, eventdata, handles)

addpath('/home/daewon/Documents/MATLAB/ASTAR')

global g_start;
global g_goal;
global g_obs_cell;
global g_att_cell;
global g_map_boundary_lat;
global g_map_boundary_lon;
global msg_gps;
global path;
global isRealtime;
global manual_start;
if (isRealtime == 1.0)
    g_start = [msg_gps.Longitude,msg_gps.Latitude];
else
    g_start = manual_start;
end
disp('in ASTAR');
disp('you have these obstacles: ')


map_boundary_1 = llToMeters(g_map_boundary_lon(1), g_map_boundary_lat(1));
map_boundary_2 = llToMeters(g_map_boundary_lon(2), g_map_boundary_lat(2));
alpha = 20; % to have enough space for the map
x_map = round([min([map_boundary_1(1), map_boundary_2(1)])-alpha, max([map_boundary_1(1), map_boundary_2(1)])]+alpha);
y_map = round([min([map_boundary_1(2), map_boundary_2(2)])-alpha, max([map_boundary_1(2), map_boundary_2(2)])]+alpha);


start_m = round(llToMeters(g_start(1), g_start(2)));
goal_m = round(llToMeters(g_goal(1), g_goal(2)));

start_ll = metersToll(start_m);
goal_ll = metersToll(goal_m);

obs_m = cell(0);
for i=1:length(g_obs_cell)
    for j=1:size(g_obs_cell{i},1)
        obs_m{i}(j,:) = round(llToMeters(g_obs_cell{i}(j,1), g_obs_cell{i}(j,2)));
    end
end

att_m = cell(0);
for i=1:length(g_att_cell)
    for j=1:size(g_att_cell{i},1)
        att_m{i}(j,:) = round(llToMeters(g_att_cell{i}(j,1), g_att_cell{i}(j,2)));
    end
end


path = findPathAStar2(att_m, obs_m, x_map, y_map, start_m, goal_m );

for i=1:size(handles.axes1.Children,1)
    isMark = findprop(handles.axes1.Children(i),'Marker');
    if(~isempty(isMark))
        set(handles.axes1.Children(i),'Marker','none');
    end
end

for i = 1: size(path,1)
   path_ll(i,:) =  metersToll(path(i,:));

   ID_path(i) = plot(path_ll(i,1), path_ll(i,2),'--gs',...
    'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','g',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
end

% --- Executes on button press in start.
function start_Callback(hObject, eventdata, handles)

global h_start;
global manual_start;
global tri;
global g_start;
[start_x,start_y] = ginput(1)
 manual_start = [start_x, start_y];
 g_start = manual_start;
%  start_m = (llToMeters(g_start(1), g_start(2)));
%  start_ll = metersToll(start_m);

scale = 0.00002;
location = [start_x, start_y];
tri = scale * [0,0.7; -0.3, -0.3; 0,0; 0.3, -0.3];
psi = -90 * pi/180;
rot = [cos(psi), sin(psi); -sin(psi), cos(psi)];
tri = tri * rot;

%tri(:,1) = tri(:,1)+location(1);
%tri(:,2) = tri(:,2)+location(2);
for i=1:size(handles.axes1.Children,1)
    isTri = findprop(handles.axes1.Children(i),'Faces');
    if(~isempty(isTri))
        set(handles.axes1.Children(i),'Marker','none');
    end
end
hold on
if(~isempty(h_start))
  delete(h_start);
end
h_start = fill(tri(:,1)+location(1),tri(:,2)+location(2),'y','facealpha',0.9,'LineWidth',1);

% --- Executes on button press in goal.
function goal_Callback(hObject, eventdata, handles);
global g_goal;
[goal_x,goal_y] = ginput(1)

hold on
gl=plot(goal_x, goal_y,'x','linewidth',2,'color','r')
g_goal = [goal_x, goal_y];



% --- Executes on button press in obstacle_4pt.
function obstacle_4pt_Callback(hObject, eventdata, handles)

global g_obs_cell;
if isempty(g_obs_cell)
    obs = cell(0);
end

[obs_x,obs_y] = ginput(4);
obs_4ll=[obs_x,obs_y;obs_x(1), obs_y(1)];

g_obs_cell{length(g_obs_cell) + 1} = obs_4ll;

hold on
h4 = fill(obs_x,obs_y,'r');
set(h4,'facealpha',.5)



% --- Executes on button press in clear.
function clear_Callback(hObject, eventdata, handles)
global start_x start_y;
global goal_x  goal_y;
global g_obs_cell;
global g_occ_cell;
global g_att_cell;
global path_s;
global h_start;
global bias;
global g_goal;
cnt = 1;
for i=1:size(handles.axes1.Children,1)
    isMark = findprop(handles.axes1.Children(cnt),'Marker');

    %isMark = findprop(handles.axes1.Children(i),'Marker');
    if(~isempty(isMark))
        %set(handles.axes1.Children(i),'Marker','none');
        delete(handles.axes1.Children(cnt));
    elseif(isempty(isMark))
            cnt = cnt + 1;
    end
end
h_start = [];
g_obs_cell = [];
g_occ_cell = [];
g_att_cell = [];
goal_x = [];
goal_y = [];
start_x = [];
start_y = [];
path_s = [];
bias = [];
g_goal = [];



% --- Executes on button press in realtime.
function realtime_Callback(hObject, eventdata, handles)
global isRealtime;
value=get(hObject,'Value');
isRealtime = value;


% --- Executes on button press in Attraction.
function Attraction_Callback(hObject, eventdata, handles)
% hObject    handle to Attraction (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global g_att_cell;
if isempty(g_att_cell)
    att = cell(0);
end

[att_x,att_y] = ginput(4);
att_4ll=[att_x,att_y;att_x(1), att_y(1)];

g_att_cell{length(g_att_cell) + 1} = att_4ll;

hold on
h3 = fill(att_x,att_y,'g');
set(h3,'facealpha',.5)


% --- Executes on button press in loadObs.
function loadObs_Callback(hObject, eventdata, handles)
global g_obs_cell;
FileName = uigetfile('*.dat');
data = load(FileName);
%data = load('obs.dat');
for i=1:size(data,1)
    g_obs_cell{i}(:,1)=data(i,1:5);
    g_obs_cell{i}(:,2)=data(i,6:10);
    hold on
    h4 = fill(g_obs_cell{i}(:,1),g_obs_cell{i}(:,2),'r');
    set(h4,'facealpha',.5)
end



% --- Executes on button press in saveObs.
function saveObs_Callback(hObject, eventdata, handles)
global g_obs_cell;
FileName = uiputfile('*.dat','Save As');
fileID = fopen(FileName,'w');
[nrows, ncols] = size(g_obs_cell{1});
formatSpec = '%e %e %e %e %e %e %e %e %e %e\n';
for i=1:length(g_obs_cell)
   fprintf(fileID, formatSpec, g_obs_cell{i}(:,:)); 
end


% --- Executes on button press in STOP.
function STOP_Callback(hObject, eventdata, handles)
global modePub;
global msgMode;
msgMode.Data = 0;
send(modePub,msgMode);


% --- Executes on button press in GO.
function GO_Callback(hObject, eventdata, handles)
global modePub;
global msgMode;
msgMode.Data = 1;
send(modePub,msgMode);


% --- Executes on button press in  .
function LOOK_Callback(hObject, eventdata, handles)
global modePub;
global msgMode;
global lookPub;
global msglook; % look
global sight;
global msg_gps;
global h_sight;
[look_x,look_y] = ginput(1);


hold on
%plot(look_x, look_y,'x','linewidth',2,'color','r');

scale = 0.0005;

fov = 60*pi/180;
p1 = [-sin(fov/2), cos(fov/2)];
p2 = [sin(fov/2), cos(fov/2)];
p3 = [0,0];
sight = scale * [p1;p2;p3];
psi = -90 * pi/180;
rot = [cos(psi), sin(psi); -sin(psi), cos(psi)];
sight = sight * rot;
sight_x(:) = sight(:,1)+msg_gps.Longitude;
sight_y(:) = sight(:,2)+msg_gps.Latitude;
hold on
delete(h_sight)
h_sight=fill(sight_x',sight_y',[1, 0.5 ,0],...
'facealpha',0.2,...
'LineWidth',1,...
'edgecolor',[1, 0.5 ,0],...
'edgealpha',0.2);
plot(look_x, look_y,'x','linewidth',2,'color','r');

msgMode.Data = 2;
send(modePub,msgMode);

msglook.Data = [look_y, look_x];
send(lookPub,msglook);


% --- Executes on button press in WP.
function WP_Callback(hObject, eventdata, handles)
global WPPub;
global msgWP; % WP
[WP_x,WP_y] = ginput(1);

hold on
plot(WP_x, WP_y,'--gs',...
    'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','g',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
msgWP.Data = [WP_x, WP_y];
send(WPPub,msgWP);


% --- Executes on button press in MANUAL.
function MANUAL_Callback(hObject, eventdata, handles)
global modePub;
global msgMode;
msgMode.Data = 3;
send(modePub,msgMode);


% --- Executes on button press in simplify.
function simplify_Callback(hObject, eventdata, handles)
global path;
global path_s;
global idx_path;
global bias;
idx_path = 1;
threshold = 1.2;
path_s = simplyfyPath( path, threshold );


for i = 1: size(path_s,1)
   path_ll(i,:) =  metersToll(path_s(i,:));

   ID_path(i) = plot(path_ll(i,1), path_ll(i,2),'--rs',...
    'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','r',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
end
if(~isempty(bias))
    path_s(:,1) = path_s(:,1) - bias.lon;
    path_s(:,2) = path_s(:,2) - bias.lat;
end



% --- Executes on button press in bias_north.
function bias_north_Callback(hObject, eventdata, handles)
global bias;
if (isempty(bias))
    bias.lon = 0;
    bias.lat = 0;
end
bias.lat = bias.lat + 1; 

% --- Executes on button press in bias_west.
function bias_west_Callback(hObject, eventdata, handles)
global bias;
if (isempty(bias))
    bias.lon = 0;
    bias.lat = 0;
end
bias.lon = bias.lon - 1; 


% --- Executes on button press in bias_east.
function bias_east_Callback(hObject, eventdata, handles)
global bias;
if (isempty(bias))
    bias.lon = 0;
    bias.lat = 0;
end
bias.lon = bias.lon + 1; 

% --- Executes on button press in bias_south.
function bias_south_Callback(hObject, eventdata, handles)
global bias;
if (isempty(bias))
    bias.lon = 0;
    bias.lat = 0;
end
bias.lat = bias.lat - 1; 

% --- Executes on button press in bias_CW.
function bias_CW_Callback(hObject, eventdata, handles)
global bias;
if (isempty(bias))
    bias.heading = 0;
end
bias.heading = bias.heading - 1; 

% --- Executes on button press in bias_CCW.
function bias_CCW_Callback(hObject, eventdata, handles)
global bias;
if (isempty(bias))
    bias.heading = 0;
end
bias.heading = bias.heading + 1; 


% --- Executes on button press in followPath.
function followPath_Callback(hObject, eventdata, handles)
global isFollowPath;
value=get(hObject,'Value');
isFollowPath = value;


% --- Executes on button press in Occlusion.
function Occlusion_Callback(hObject, eventdata, handles)
global g_obs_cell;
global g_occ_cell;
if isempty(g_obs_cell)
    obs = cell(0);
end
if isempty(g_occ_cell)
    occ = cell(0);
end

[obs_x,obs_y] = ginput(4);
obs_4ll=[obs_x,obs_y;obs_x(1), obs_y(1)];

g_obs_cell{length(g_obs_cell) + 1} = obs_4ll;
g_occ_cell{length(g_occ_cell) + 1} = obs_4ll;

hold on
h4 = fill(obs_x,obs_y,'b');
set(h4,'facealpha',.5)


% --- Executes on button press in Goal_Opt.
function Goal_Opt_Callback(hObject, eventdata, handles)
global g_obs_cell; %lonlat
global g_occ_cell;
global g_map_boundary_lat; % (39.9526   39.9519)
global g_map_boundary_lon; %(-75.1900  -75.1913)
global g_target;
global g_goal;
global g_stationary;
offset = llToMeters(min(g_map_boundary_lon), min(g_map_boundary_lat));
map_max = llToMeters(max(g_map_boundary_lon), max(g_map_boundary_lat)) - offset;
target_m = llToMeters(g_target(1),g_target(2)) - offset;
stationary_m = llToMeters(g_stationary(1),g_stationary(2)) - offset;
obs_cell = g_obs_cell;
for n=1:size(g_obs_cell,2)
    for k=1:5
        obs_cell{n}(k,:)=llToMeters(g_obs_cell{n}(k,1), g_obs_cell{n}(k,2))-offset;
    end
end

occ_cell = g_occ_cell;
shade_cell = cell(0);

for n=1:size(g_occ_cell,2)
    in_34quad = 0;
    for k=1:5
        occ_cell{n}(k,:)=llToMeters(g_occ_cell{n}(k,1), g_occ_cell{n}(k,2))-offset;
        corner_angle(k,1) = atan2(occ_cell{n}(k,2)-target_m(2),occ_cell{n}(k,1)-target_m(1));
        corner_angle(k,2:3) = occ_cell{n}(k,1:2);
        
        if(in_34quad == 0)
            if (corner_angle(k,1) > pi/2)
                in_34quad = in_34quad + 1;
            elseif (corner_angle(k,1) < -pi/2)
                in_34quad = in_34quad - 1;
            end
        end
        
        if(in_34quad == 1)
            if (corner_angle(k,1) < -pi/2)
                in_34quad = in_34quad + 1;
            end
        end
        
        if(in_34quad == -1)
            if (corner_angle(k,1) > pi/2)
                in_34quad = in_34quad - 1;
            end
        end
        %
    end
    
    if (abs(in_34quad) == 2)
        for k=1:5
            if ( corner_angle(k,1) < 0)
                corner_angle(k,1) = corner_angle(k,1) + 2*pi;
            end
        end
    end
        %corner_selected(n,:)=[min(corner_angle), 
        id_min = find(corner_angle==min(corner_angle(:,1)),1);
        id_max = find(corner_angle==max(corner_angle(:,1)),1);
        A = 300;
        min_ext = target_m + A*[cos(corner_angle(id_min,1)), sin(corner_angle(id_min,1))];
        max_ext = target_m + A*[cos(corner_angle(id_max,1)), sin(corner_angle(id_max,1))];
        shade_cell{n}=[corner_angle(id_min,2:3) ;  min_ext; max_ext; corner_angle(id_max,2:3); corner_angle(id_min,2:3)];
        
end

hold on
for n=1:size(shade_cell,2)
    for k = 1:5
       shade_m = shade_cell{n}+offset;
       shade_ll(k,:) =  metersToll(shade_m(k,:));
    end
    hold on
h4 = fill(shade_ll(:,1),shade_ll(:,2),'y');
set(h4,'facealpha',.2)
end


%% navigable map
navigable_map = zeros(round(map_max(1)),round(map_max(2))); 
% for i=1:length(obs_cell)
%     navigable_map = occupyObsMap(navigable_map_init,obs_cell{i});
%     navigable_map_init = navigable_map;
% end

%% occlusion map

%map_out = occupyOcclusionMap( map, obs_cell,  shade_cell, target )
cnt = 0;
for i = 1:size(navigable_map,1)
   for j = 1:size(navigable_map,2)
       in_obs = 0;
       in_shade = 0;
       for k = 1:size(obs_cell,2)
            if(inpolygon(i,j,obs_cell{k}(:,1),obs_cell{k}(:,2)))
                in_obs = 1;
            end
       end
       
       if (in_obs == 0)
             for  m = 1:size(shade_cell,2)
                if(inpolygon(i,j,shade_cell{m}(:,1),shade_cell{m}(:,2)))
                    in_shade = 1;
                end
             end
       end
       
       if(in_obs == 0 && in_shade == 0)
            navigable_map(i,j) = 1;
            cnt = cnt + 1;
            v1 = stationary_m - target_m;
            v2 = -target_m + [i,j];
            possible_spot_angle(cnt,1) = acos(sum(v1.*v2)/(norm(v1)*norm(v2)));
            possible_spot_angle(cnt,2:3) = [i,j];
       end
   end
end
id_best_spot = find(possible_spot_angle==max(possible_spot_angle(:,1)),1);
best_spot = possible_spot_angle(id_best_spot,2:3);
best_spot = best_spot + offset;
best_spot_ll = metersToll(best_spot);
gl=plot(best_spot_ll(1), best_spot_ll(2),'x','markersize',4,'linewidth',2,'color','m')
g_goal = best_spot_ll;
% --- Executes on button press in Target.
function Target_Callback(hObject, eventdata, handles)
global g_target;
[target_x,target_y] = ginput(1)

hold on
gl=plot(target_x, target_y,'x','markersize',4,'linewidth',2,'color','r')
g_target = [target_x, target_y];


% --- Executes on button press in Stationary_cam.
function Stationary_cam_Callback(hObject, eventdata, handles)
global g_stationary;
[stationary_x,stationary_y] = ginput(1)

hold on
gl=plot(stationary_x, stationary_y,'*','markersize',4,'linewidth',2,'color','g')
g_stationary = [stationary_x, stationary_y];

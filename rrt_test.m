clear;
clc;

% MAZE
map = load('maze.mat'); %원하는 파일 maze 불러오면 됨

% 출발, 도착지 설정 (map 마다 다르게 진행)
q_ini = [50, 50]; %50, 50
q_fin = [600, 600]; % 600, 600

map = map.map; %지도 가져옴.

o = 100000; %반복 횟수
del_q = 60; %노드 간 최대 거리
p = 0.3; %목표 지점 선택 확률


delta = 5; %smooth 함수에서 사용할 경로 스무딩에 필요한 매개 변수 설정.
[vertice, edge, path] = rrt(map, q_ini, q_fin, o, del_q, p);
path_smooth = smooth(map, path, vertice, delta);

%시각화
imshow(int32(1 - map), []);

title('RRT Based Path Planning');

hold on;

[edgesRowCount, ~] = size(edge);

for ii = 1 : edgesRowCount
    plot(vertice(ii, 1), vertice(ii, 2), 'cyan*', 'linewidth', 1);
    plot([vertice(edge(ii, 1), 1), vertice(edge(ii, 2), 1)], ...
    [vertice(edge(ii, 1), 2), vertice(edge(ii, 2), 2)], ...
     'b', 'LineWidth', 1);
    drawnow;  
    pause(0.00001);  % 속도 조절

end

plot(q_ini(1), q_ini(2), 'g*', 'linewidth', 1);
plot(q_fin(1), q_fin(2), 'r*', 'linewidth', 1);


[~, pathCount] = size(path);

 for ii = 1 : pathCount - 1
    plot([vertice(path(ii), 1), vertice(path(ii + 1), 1)], ...
    [vertice(path(ii), 2), vertice(path(ii + 1), 2)], ...
     'r', 'LineWidth', 1);
    drawnow;  
    pause(0.00001);  % 속도 조절

 end

[~, pathCount] = size(path_smooth);

for ii = 1 : pathCount - 1
    x(ii) = double(vertice(path_smooth(ii), 1));
    y(ii) = double(vertice(path_smooth(ii), 2));
    plot([vertice(path_smooth(ii), 1), vertice(path_smooth(ii + 1), 1)], ...
    [vertice(path_smooth(ii), 2), vertice(path_smooth(ii + 1), 2)], ...
     'black', 'LineWidth', 2);
    x(pathCount)=q_ini(1);
    y(pathCount)=q_ini(2);
    drawnow;  
    pause(0.00001);  % 속도 조절
end


size = length(x);
t = linspace(0, 1, size);  

% pchip or spline
t_interp = linspace(0, 1, 100);  
x_interp = pchip(t, x, t_interp);
y_interp = pchip(t, y, t_interp);


lineWidth = 2; % 라인 두께
lineColor = [1, 0.5, 0]; % 주황색
plot(x_interp, y_interp, '-o', 'LineWidth', lineWidth, 'Color', lineColor);

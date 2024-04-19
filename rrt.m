function [vertice, edge, path] = rrt(map, q_ini, q_fin, o, del_q, p)
%Algorithm to build a tree to solve map
%vertice: 정점의 x,y좌표
%첫번째 정점은 시작 위치, 마지막 정점은 목표 위치(x,y 2개의 열과 n개의 행으로 이루어져 있어야 함)
%edge: 트리의 선 n-1개가 있으며 n-1행으로 저장. 목표 저장. 
%정점은 루트 방향으로 한 단계 앞에 있는 정점의 인덱스를 포함해야 한다. 목표를 찾으면 마지막 행에 목표 인덱스가 저장.
%map: 불러온 지도
%q_ini: 시작점 좌표
%q_fin: 목표점 좌표
%o: 최대로 생성해볼 샘플 점
%del_q: q_new와 q_near 사이의 거리 (일단은 50)
%p: q_goal을 q_ran으로 직접 설정할 수 있는지 결정 (0~1로) 작은 p값은 목표 지점을 무시하고 주로 무작위로 확장하게 하며, 
%큰 p 값은 목표 지점에 직접 향하도록 더 자주 확장하게 함.
tic;

clc;

%기본적인 오류를 체크하는 항목
if nargin < 3
    error('첫 번째 3개의 매개변수가 필요합니다: 2D 맵 행렬, 시작 좌표 및 목표 좌표입니다.');

elseif nargin < 4
    o = 11000; del_q = 50; p = 0.3;

elseif nargin < 5
    del_q = 50; p = 0.3;

elseif nargin < 6
    p = 0.3;

end

%checkPara 함수는 매개변수가 제대로 들어갔는지 확인하는 함수
checkPara(map, q_ini, q_fin, o, del_q, p);

% map: 높이가 x, 너비는 y이다.
[mapHeight, mapWidth] = size(map); %맵 분석 시에는 x좌표와 y좌표가 일반적인 의미와 반대로 설정되어있음을 유의.

q_ini = int32(q_ini); %q_ini(1):y, q_ini(2):x를 의미한다.
q_fin = int32(q_fin); %q_fin(1):y, q_fin(2): x를 의미한다.
vertice = q_ini; %정점의 값을 q_ini로 초기화 시킨다.

edge = int32.empty(0, 2); 
q_ran = int32.empty(0, 2);
q_near = int32.empty(0, 2);
q_new = int32.empty(0, 2);

%샘플링과 트리확장 코드(가장 핵심 코드)

for ii = 1 : o %o번 반복    
    if rand() < p %rand 함수로 [0,1) 범위의 난수를 생성하고 이 값이 p보다 작으면 q_ran을 목표 지점 q_fin으로 설정
        q_ran = q_fin;
    else %아니면 q_ran은 맵 내에 무작위로 생성됨
        q_ran = int32([randi(mapWidth) randi(mapHeight)]); %열이 x,행이 y
    end
    
    [q_near, qNearIn] = findQNear(q_ran, vertice); %q_ran에서 가장 가까운 노드인 q_near을 정점들 중 찾아낸다.
    
    q_new = findQNew(q_near, q_ran, del_q); % q_near에서 q_ran 방향으로 del_q 만큼 떨어진 위치에 새로운 노드 q_new를 생성한다.
    
    if q_new(1) < 1 || q_new(2) < 1 || q_new(1) > mapWidth || q_new(2) > mapHeight
        continue;
    end %생성된 q_new가 맵의 경계를 벗어나는지 확인하고 벗어나면 반복문의 처음으로 돌아간다.
    
    if map(q_new(2), q_new(1)) == 0 % q_new가 자유 공간에 속하는지 확인
        
        % q_near에서 q_new로의 엣지가 자유 공간에 속하는지 확인.
        if isEdgeQNearQNewBelongsFreeSpace(map, q_near, q_new)
            
            % q_new를 노드 목록에 추가, q_near에서 q_new로의 엣지를 엣지 목록에 추가
            vertice = [vertice; q_new];
            [qNewIndex, ~] = size(vertice);
            edge = [edge; [int32(qNewIndex), int32(qNearIn)]];
            
            % q_new가 목표지점 q_fin과 일치하거나 q_fin이 q_near에서 q_new 엣지 위에 있는 경우
            if isequal(q_new, q_fin) || isQGoalOnQNearQNewEdge(q_near, q_new, q_fin)
                
                if ~isequal(q_new, q_fin) %목표 지점이 q_new와 일치하지 않으면서, q_fin이 q_near에서 q_new엣지 위에 있다면: 
                    % 목표가 엣지 상에 있는 경우, 마지막 노드를 제거하고 대신 목표 지점 q_fin을 노드 목록에 추가
                    vertice = vertice(1 : (end - 1), :);
                    vertice = [vertice; q_fin];
                end
                
               path = fillSolutionPath(edge, vertice);
              
                             
                toc;
                
                return;
            end
        end
    end
end %목표 지점에 도달했거나 목표가 엣지 상에 있는 경우, 경로를 구성하고 RRT 알고리즘을 종료한다.

    path = int32.empty(0, 2);
    toc;
    
    error('해가 없습니다.:(');
end %알고리즘이 끝나도 목표지점에 도달하지 못하면, 빈 경로를 반환하고 에러 메시지를 출력한다.

function checkPara(map, q_ini, q_fin, o, del_q, p)

    [mapHeight, mapWidth] = size(map);
    
    if mapWidth < 1 || mapHeight < 1
        error('맵 크기 오류. 2차원의 맵 할당 필요.');
    end
    %만약 mapWidth와 mapHeight가 1보다 작다면 에러
    [x, y] = size(q_ini); %초기 위치 q_ini 행렬의 크기를 얻어옴.
    if x ~= 1 || y ~= 2
        error('q_ini - start position가 [56, 42]처럼 1*2 행렬이어야 합니다.'); %만약 1*2행렬이 아니거나 맵의 범위가 아니면 오류
    elseif q_ini(1) < 0 || q_ini(1) > mapWidth || q_ini(2) < 0 || q_ini(2) > mapHeight
        error('q_ini - start position가 맵 범위에 있어야 합니다.'); %그렇지 않고, 만약 q_ini의 첫 번째 요소가 맵의 너비를 벗어나거나, 
                                                                    %두 번째 요소가 맵의 높이를 벗어나면 에러 발생
    end
    
    [x, y] = size(q_fin); %목표 위치 q_fin 행렬의 크기를 얻어옴.
    if x ~= 1 || y ~= 2
        error('q_fin - goal position - 가 [302, 521]처럼 1*2 행렬이어야 합니다.'); %만약 q_fin이 1*2 행렬이 아니면 에러
    elseif q_fin(1) < 0 || q_fin(1) > mapWidth || q_fin(2) < 0 || q_fin(2) > mapHeight
        error('q_fin - goal position가 맵 범위에 있어야 합니다.'); %q_fin의 첫 번째 요소가 맵의 너비를 벗어나거나, 
                                                                  %두 번째 요소가 맵의 높이를 벗어나면 에러 발생.
    end
    
    %기본 조건들
    if o < 1
        error('o - 반복횟수가 양수여야 합니다.');
    end
    
    if del_q < 1
        error('del_q - 거리가 양수여야 합니다.');
    end
    
    if p < 0 || p > 1
        error('p - probability가 0과 1 사이어야 합니다.');
    end
    
end

%q_ran과 vertice에서 가장 가까운 노드를 찾는 함수를 정의. 결과로는 q_near와 qNearIndex를 반환.
function [q_near, qNearIndex] = findQNear(q_ran, vertice)

    [rowCount, ~] = size(vertice); %verice 행렬의 행 수를 rowCount에 저장. (노드의 개수)
    
    if rowCount < 1
        error('해가 없습니다. :(');
    end
    %노드가 없다면 에러
    euclideanDistances = double.empty(0, 1); %유클리디안거리를 저장할 빈 벡터 초기화.
    
    %각 노드에 대해 반복.
    for ii = 1 : rowCount
        euclideanDistances(ii, 1) = pdist2(double(q_ran), double(vertice(ii, :)), 'euclidean'); %q_rand와 현재 노드 사이의 유클리디안 거리 계산 후 벡터에 저장.
    end
    
    minDistanceIndex = find(euclideanDistances == min(euclideanDistances)); %최소 거리를 가진 노드의 인덱스 찾음.
    
    q_near = vertice(minDistanceIndex(1), :); %최소 거리를 가진 노드를 q_near로 설정.
    
    qNearIndex = minDistanceIndex(1); %q_near의 인덱스를 qNearIndex로 설정.

end

%현재 노드 q_near에서 목표 노드 q_rand로 이동하기 위해 del_q만큼 떨어진 새로운 노드 q_new를 생성.
function [q_new] = findQNew(q_near, q_ran, del_q)

    v = double(q_ran - q_near); %q_rand와 q_near 사이의 벡터 v 계산(실수형)
    
    u = v / norm(v); %벡터 v를 길이로 나누어 정규화. 이로써 u는 v의 방향을 나타내는 단위 벡터가 된다.
    
    q_new = int32(double(q_near) + del_q * u); %현재 노드 q_near를 실수형으로 변환하고 이에 방향과 거리를 곱한 값을 더하여 새로운 노드 q_new를 생성. 마지막으로 결과 정수형으로 변환
    
end

% 두 노드 q_near와 q_new를 연결하는 엣지가 자유 공간에 속하는지 확인.
function [isBelongsFreeSpace] = isEdgeQNearQNewBelongsFreeSpace(map, q_near, q_new)
    
   
    intermediatePointCount = 10; %중간 점들의 개수 지정. 10개의 중간 점을 사용.
    v = double(q_new - q_near);%q_new와 q_near 사이의 벡터 v 계산 (실수형)

    distance = norm(v);  
    u = v / distance;
    %정규화

    del_q = distance / intermediatePointCount; %각 중간 점 사이의 거리를 계산해 del_q에 저장   
    currentCoordinate = double(q_near); %현재 좌표를 q_near로 초기화
    
    for ii = 1 : intermediatePointCount %중간 점들에 대한 반복문 시작.
        
        currentCoordinate = currentCoordinate + (del_q * u); %현재 좌표에 중간 점 간격만큼 이동.
        
        if map(int32(currentCoordinate(2)), int32(currentCoordinate(1))) == 1 % 현재 좌표가 지도 상에서 장애물에 해당하면 
            isBelongsFreeSpace = 0; %설정 후 함수 종료
            return;
        end
        
    end
    
    isBelongsFreeSpace = 1; %모든 중간 점에 대해 장애물이 없다면 1로 설정

end %q_near에서 시작해 q_new로 향하는 엣지가 자유공간에 있는지 확인하기 위해 중간 점들을 이용. 만약에 장애물이 발견되면 0, 발견되지 않으면 1로 설정!


% q_near, q_new, q_fin을 이어주는 엣지 중에서 q_goal이 엣지 상에 위치하는지 여부 확인.
function [isQGoalOnEdge] = isQGoalOnQNearQNewEdge(q_near, q_new, q_fin)
    
    v = double(q_new - q_near); %q_new와 q_near 사이의 벡터 v 계산    
    distance = norm(v);
    
    u = v / distance; %정규화
    
    distanceQNearQGoal = norm(double(q_fin - q_near));
    %q_neqr에서 q_goal까지 거리가 q_near에서 q_new까지의 거리보다 크다면
    if distanceQNearQGoal > distance
        isQGoalOnEdge = 0; %설정하고 함수 종료.
        return;
    end
    
    q_goal_hat = double(q_near) + (distanceQNearQGoal * u); %q_near에서 q_new 방향으로 q_goal까지의 거리만큼 이동한 좌표 q_goal_hat을 계산.  
    isQGoalOnEdge = isequal(int32(q_goal_hat), q_fin); %q_goal_hat이 q_fin과 일치하는지 확인하여 결과를 저장
    
end %q_neqr, q_new, q_fin 세 노드로 이루어진 엣지가 있을 때, q_goal,이 해당 엣지 상에 있는지 확인. 만약 엣지 위에 있으면 1, 아니면 0

function [path] = fillSolutionPath(edges, vertice)

    path = edges(end, 1); %초기에 path를 경로의 첫번째 노드로 설정
    prev = edges(end, 2); %초기에 prev를 경로의 두 번째 노드로 설정

    
    ii = 0; %반복횟수 초기화
    [edgesCount, ~] = size(edges);
    
    while prev ~= 1 %prev가 1이 될 때까지 반복
        if ii > edgesCount %반복 횟수가 edges 행렬의 행 수를 초과하면 에러 발생
            error('RRT: no path found :(');
        end
        prevIndex = find(edges(:, 1) == prev); %edge에서 prev와 연결된 엣지의 인덱스를 찾음
        prev = edges(prevIndex(1), 2);
        path = [path, prev];
        ii = ii + 1;
    end

end  %edge와 vertice를 이용하여 RRT 알고리즘이 찾은 경로를 path로 구성, 이 경로를 반환. 아니면 에러 발생

% % RRT 결과 시각화 함수
function rrtDraw(map, q_ini, q_fin, vertice, edge, path)

    imshow(int32(1 - map), []);
    title('RRT Based Path Planning');    
    hold on;
    
    [edgesRowCount, ~] = size(edge);
    
    for ii = 1 : edgesRowCount
        plot(vertice(ii, 1), vertice(ii, 2), 'cyan*', 'linewidth', 1);
        plot([vertice(edge(ii, 1), 1), vertice(edge(ii, 2), 1)], ...
        [vertice(edge(ii, 1), 2), vertice(edge(ii, 2), 2)], ...
         'b', 'LineWidth', 1);
    end
    
    plot(q_ini(1), q_ini(2), 'g*', 'linewidth', 1);
    plot(q_fin(1), q_fin(2), 'r*', 'linewidth', 1);
    
    
    [~, pathCount] = size(path);
    
    for ii = 1 : pathCount - 1
        plot([vertice(path(ii), 1), vertice(path(ii + 1), 1)], ...
        [vertice(path(ii), 2), vertice(path(ii + 1), 2)], ...
         'r', 'LineWidth', 2);
    end
    
end

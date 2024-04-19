function [path_smooth] = smooth(map, path, vertice, delta)
%map: 지도 스크립트
%path: q_start에서 q_goal로 이르는 정점 인덱스 목록 (행 벡터 표현)
%vertices: 정점들의 x,y좌표 목록. 첫 번째 정점은 시작 위치, 마지막 정점은 목표 위치. 2개의 열과 n개의 행(정점의 수)
%delta: 직접적인 연결이 자유 공간 내에 있는지 확인하는 데 사용되는 증분 거리, edge는 이 delta 거리로 나뉘어 여러 지점을 얻음.
%path_smooth: smoothing 알고리즘을 적용한 후의 q_start에서 q_goal까지의 정점 인덱스를 나타내는 축소된 목록 (행 벡터 표현)

path_smooth = path(1); % path의 첫 원소를 path_smooth로 할당
currentIndex = 1; % currentIndex를 1로 초기화하여 ‘path’ 배열을 반복할 때 사용할 인덱스(순방향의미)
currentSmoothIndex = numel(path); % numel(path)는 path 배열의 요소 수를 나타내며, 이를 currentSmoothIndex에 할당. 이 변수는 경로 배열을 역방향으로 이동할 때 사용
%currentIndex와 currentSmoothIndex를 사용해 배열을 반복하거나 역방향으로 반복하는데 활용.

while currentIndex < numel(path) %currentIndex가 path 배열의 길이보다 작을 때까지 반복
    
    while currentIndex < currentSmoothIndex %currentIndex가 currentSmoothIndex보다 작을 때까지 반복.
        
        if isEdgeBelongsFreeSpace(map, vertice(path(currentSmoothIndex), :), vertice(path(currentIndex), :), delta)
            path_smooth = [path_smooth, path(currentSmoothIndex)];
            currentIndex = currentSmoothIndex; %만약 현재 엣지가 자유공간에 있다면 path_smooth배열에 path(currentSmoothIndex)를 추가.
            break;
        else
            currentSmoothIndex = currentSmoothIndex - 1; %아니라면 currentSmoothIndex를 1만큼 감소시킴.
        end
        
    end
    
    currentSmoothIndex = numel(path); %내부 반복이 끝나면 ‘currentSmoothIndex’를 다시 path 배열의 길이로 초기화.
  
end

end

%startP와 endP를 연결하는 엣지가 자유공간에 있는지 확인
function [isBelongsFreeSpace] = isEdgeBelongsFreeSpace(map, startP, endP, delta)
    
    v = double(endP - startP); %endP와 startP를 연결하는 v를 계산하고 거리 구함
    distance = norm(v);
    u = v / distance; %단위 벡터로 정규화
    
    interPCount = distance / delta; %delta 간격으로 나누어지는 중간 지점 계산.
    
    currentCoordinate = double(startP); %현재 좌표를 startP로 초기화. 이 좌표는 반복문을 통해 업데이트되며 엣지 상의 중간 지점 나타냄.
    
    for ii = 1 : interPCount %1부터 중간 지점까지 반복
        
        currentCoordinate = currentCoordinate + (delta * u); %현재 좌표를 엣지 상의 다음 중간 지점으로 이동.
        
        if map(int32(currentCoordinate(2)), int32(currentCoordinate(1))) == 1 %이동한 좌표가 지도에서 장애물에 해당하는 위치인 경우
            isBelongsFreeSpace = 0; %isBelongsFreeSpace를 0으로 설정후 함수 종료. (자유공간 아님)
            return;
        end
        
    end
    
    isBelongsFreeSpace = 1;

end

%이미지 시각화 코드
function rrtSmoothDraw(map, path_smooth, vertice)

    imshow(int32(map), []);
    title('RRT (Rapidly-Exploring Random Trees) - Smooth Path');
    
    hold on;

    [~, pathCount] = size(path_smooth);
    
    for ii = 1 : pathCount - 1
        
        plot([vertice(path_smooth(ii), 1), vertice(path_smooth(ii + 1), 1)], ...
        [vertice(path_smooth(ii), 2), vertice(path_smooth(ii + 1), 2)], ...
         'r', 'LineWidth', 2);
    end
    
end
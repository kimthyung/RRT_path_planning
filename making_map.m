wall_size = 10;
path_size = 140;
n = 5;

intv = wall_size+path_size;
map_size = wall_size*(n+1) + path_size * n;
map = zeros(map_size,map_size);

for i = 0:n
    map(intv*i + 1:intv*i + wall_size,:) = 1;
    map(:,intv*i + 1:intv*i + wall_size) = 1;
end

while 1
    prompt = "wall(1) or path(2) or end(0)?: ";
    wp = input(prompt);
    if wp ==0
        break
    end
    while 1
        if wp ==2
            prompt = "row: ";
            a = input(prompt);
            if a ==0
                break
            end
            prompt = "col: ";
            b = input(prompt);
            prompt = "width(1) or length(2)?: ";
            c = input(prompt);
    
            if c == 2
               map(intv*a + 1:intv*a + wall_size,wall_size+intv*(b-1)+1:intv*b)=0;
            elseif c == 1
               map(wall_size+intv*(a-1)+1:intv*a,intv*b + 1:intv*b + wall_size)=0;
            else
                break
            end
            imshow(int32(1 - map), []);

        elseif wp ==1
            prompt = "row: ";
            a = input(prompt);
            if a ==0
              break
            end
            prompt = "col: ";
            b = input(prompt);
            prompt = "width(1) or length(2)?: ";
            c = input(prompt);
            if c == 2
               map(intv*a + 1:intv*a + wall_size,wall_size+intv*(b-1)+1:intv*b)=1;
            elseif c == 1
               map(wall_size+intv*(a-1)+1:intv*a,intv*b + 1:intv*b + wall_size)=1;
            else
                break
            end
            imshow(int32(1 - map), []);
        end
    end
end
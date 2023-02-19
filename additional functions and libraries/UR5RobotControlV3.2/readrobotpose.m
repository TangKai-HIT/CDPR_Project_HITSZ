% t is the Socket Connection handle
% P is translation in mm and rotation vector
function P = readrobotpose(t)

if t.BytesAvailable>0
    fscanf(t,'%c',t.BytesAvailable);
end
fprintf(t,'(2)'); % task = 2 : reading task
while t.BytesAvailable==0
end
rec = fscanf(t,'%c',t.BytesAvailable);
if ~strcmp(rec(1),'p') || ~strcmp(rec(end),']')
    error('robotpose read error')
end
rec(end) = ',';
Curr_c = 2;
for i = 1 : 6
    C = [];
    Done = 0;
    while(Done == 0)
        Curr_c = Curr_c + 1;
        if strcmp(rec(Curr_c) , ',')
            Done = 1;
        else
            C = [C,rec(Curr_c)];
        end
    end
    P(i) = str2double(C);   
end
for i = 1 : length(P)
    if isnan(P(i))
        error('robotpose read error (Nan)')
    end
end
P(1:3) = P(1:3)*1000; % converting to mm
end
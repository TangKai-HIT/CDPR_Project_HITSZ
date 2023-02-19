function ActivateCoopMode(t)
if t.BytesAvailable>0
    fscanf(t,'%c',t.BytesAvailable);
end
fprintf(t,'(3)'); % task = 3 : Coopmode activate
while t.BytesAvailable==0
end
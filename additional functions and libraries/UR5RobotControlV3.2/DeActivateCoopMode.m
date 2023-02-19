function DeActivateCoopMode(t)
if t.BytesAvailable>0
    fscanf(t,'%c',t.BytesAvailable);
end
fprintf(t,'(4)'); % task = 3 : Coopmode deactivate
while t.BytesAvailable==0
end
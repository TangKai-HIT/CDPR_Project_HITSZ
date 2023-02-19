%% calculateAnchorDistance 程序
% 说明：计算动平台上锚点两两间的距离并将结构保存至变量anchorDistance，将上三角阵压缩为1X28数组
% 计算动平台上锚点两两间的单位向量（单向，方向i->j且i<j）
load('anchorPosition_P');
num=size(anchorPosition_P,2);
anchorDistance=zeros(1, nchoosek(num,2));
anchorUniVec_P=zeros(3, nchoosek(num,2));
for i=1:num
    for j=(i+1):num
        index=(j-i)+((num-1)+(num-i+1))*(i-1)/2;
        anchorDistance(index)=norm(anchorPosition_P(:,j)-anchorPosition_P(:,i));
        anchorUniVec_P(:, index)=(anchorPosition_P(:,j)-anchorPosition_P(:,i))/anchorDistance(index);
    end
end
save('anchorDistance', 'anchorDistance');
save('anchorUniVec_P', 'anchorUniVec_P');
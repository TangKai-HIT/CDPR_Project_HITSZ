function A_pseudoInv=psedoInverse_svd(A)
%% 运用精简svd(compact svd)分解求解伪逆(定义法)
% 输入：任意非零矩阵A
% 输出：A的伪逆
[u,s,v]=svd(A,'econ'); %精简svd
s_inv=diag(1./s(find(s~=0))); %s的逆
A_pseudoInv = v* s_inv * u';
end
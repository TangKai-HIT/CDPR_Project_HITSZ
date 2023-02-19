#include "mex.h"
#include"ControlCAN.h"
#pragma comment(lib,"ControlCAN.lib")
void mexFunction(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[])
{
	if( nrhs != 4)  //判断输入参数的个数
	{
		mexErrMsgTxt("输入参数个数不对!");
	}
	double p1 =*((double*)mxGetPr(prhs[0]));
	double p2 =*((double*)mxGetPr(prhs[1]));
	double p3 =*((double*)mxGetPr(prhs[2]));
    
	VCI_INIT_CONFIG tVCI_INIT_CONFIG;
	tVCI_INIT_CONFIG.AccCode   = *((double*)(mxGetPr(prhs[3])));
	tVCI_INIT_CONFIG.AccMask   = *((double*)(mxGetPr(prhs[3])+1));
	tVCI_INIT_CONFIG.Reserved  = *((double*)(mxGetPr(prhs[3])+2));
    tVCI_INIT_CONFIG.Filter    = *((double*)(mxGetPr(prhs[3])+3));
	tVCI_INIT_CONFIG.Timing0  = *((double*)(mxGetPr(prhs[3])+4));
    tVCI_INIT_CONFIG.Timing1  = *((double*)(mxGetPr(prhs[3])+5));
	tVCI_INIT_CONFIG.Mode     = *((double*)(mxGetPr(prhs[3])+6));
	nlhs =1;
	plhs[0]=mxCreateDoubleMatrix(1,1,mxREAL);
	double *output =(double*)mxGetPr(plhs[0]);
	*output=VCI_InitCAN(p1,p2,p3,&tVCI_INIT_CONFIG);
}

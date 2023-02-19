#include "mex.h"
#include"ControlCAN.h"
#pragma comment(lib,"ControlCAN.lib")
void mexFunction(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[])
{
	if( nrhs != 3)
	{
		mexErrMsgTxt("输入参数个数不对!");
		return;
	}
	unsigned long p1 =*((unsigned long*)mxGetPr(prhs[0]));
	unsigned long p2 =*((unsigned long*)mxGetPr(prhs[1]));
	unsigned long p3 =*((unsigned long*)mxGetPr(prhs[2]));
	plhs[0]=mxCreateDoubleMatrix(1,1,mxREAL);
	unsigned long *output =(unsigned long*)mxGetPr(plhs[0]);
	*output=VCI_ClearBuffer(p1,p2,p3);
}

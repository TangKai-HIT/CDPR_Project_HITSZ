#include "mex.h"
#include"ControlCAN.h"
#pragma comment(lib,"ControlCAN.lib")
void mexFunction(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[])
{
	if( nrhs != 2)
	{
		mexErrMsgTxt("输入参数个数不对!");
		return;
	}
	double p1 =*((double*)mxGetPr(prhs[0]));
	double p2 =*((double*)mxGetPr(prhs[1]));
	plhs[0]=mxCreateDoubleMatrix(1,1,mxREAL);
	double *output =(double*)mxGetPr(plhs[0]);
	*output=VCI_CloseDevice(p1,p2);
}

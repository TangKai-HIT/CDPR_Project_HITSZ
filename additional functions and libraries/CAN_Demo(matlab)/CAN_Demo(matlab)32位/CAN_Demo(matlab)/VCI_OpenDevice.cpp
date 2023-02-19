#include "mex.h"
#include"ControlCAN.h"
#pragma comment(lib,"ControlCAN.lib")
void mexFunction(int nlhs,mxArray *plhs[],int nrhs, const mxArray *prhs[])
{
	if( nrhs != 3) 
	{
		mexErrMsgTxt("输入参数个数不对!");
		return;
	}
	nlhs=2;
	double p1 = *((double*)mxGetPr(prhs[0]));
	double p2 = *((double*)mxGetPr(prhs[1]));
	double p3 = *((double*)mxGetPr(prhs[2]));
	plhs[0]=mxCreateDoubleMatrix(1,1,mxREAL);
	double *output1 =(double*)mxGetPr(plhs[0]);
	*output1=VCI_OpenDevice(p1,p2,p3);
}

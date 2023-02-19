#include "mex.h"
#include "matrix.h"
#include"ControlCAN.h"
#include <string.h>
#pragma comment(lib,"ControlCAN.lib")
void mexFunction(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[])
{

	if( nrhs != 2)  //判断输入参数的个数
	{
		mexErrMsgTxt("输入参数个数不对!");
	}
	double DevType =*((double*)mxGetPr(prhs[0]));
	double DevIndex =*((double*)mxGetPr(prhs[1]));
	nlhs=2;
	PVCI_BOARD_INFO  ptVCI_BOARD_INFO = new VCI_BOARD_INFO;
	plhs[0]=mxCreateDoubleMatrix(1,1,mxREAL);
	double *output1 =(double*)mxGetPr(plhs[0]);
    *output1 = VCI_ReadBoardInfo(DevType,  DevIndex, ptVCI_BOARD_INFO);
 	mxArray *pa=NULL;
	const int dim[1]={9};
	plhs[1] = mxCreateCellArray(1,dim);
	pa= mxCreateDoubleMatrix(1,1,mxREAL);
	double *data=mxGetPr(pa);
	for (int i=0;i<6;i++)
	{
		pa = mxCreateDoubleMatrix(1,1,mxREAL);
		double *data = mxGetPr(pa);
		switch(i)
		{
		case 0:
			*data =ptVCI_BOARD_INFO->hw_Version;
			
			break;
		case 1:
			*data = ptVCI_BOARD_INFO->fw_Version;
			break;
		case 2:
			*data = ptVCI_BOARD_INFO->dr_Version;
			break;
		case 3:
			*data = ptVCI_BOARD_INFO->in_Version;
			break;
		case 4:
			*data = ptVCI_BOARD_INFO->irq_Num;
			break;
		case 5:
			*data = ptVCI_BOARD_INFO->can_Num;
			break;
		}
		 mxSetCell(plhs[1],i,pa);
	}
	char str_Serial_Num[20];
	memcpy(str_Serial_Num,ptVCI_BOARD_INFO->str_Serial_Num,20);
    mxArray *mxstr= mxCreateString(str_Serial_Num);
    mxSetCell(plhs[1],6,mxstr);

	char str_hw_Type[40];
	memcpy(str_hw_Type,ptVCI_BOARD_INFO->str_hw_Type,40);
	mxArray *mxstr1=mxCreateString(str_Serial_Num);
    mxSetCell(plhs[1],7,mxstr1);

	char Reserved[4];
	memcpy(Reserved,ptVCI_BOARD_INFO->str_hw_Type,4);
	mxArray *mxstr2=mxCreateString(Reserved);
	mxSetCell(plhs[1],8,mxstr2);
    double *output2=(double*)mxGetPr(plhs[1]);
    delete	ptVCI_BOARD_INFO;
}

#include "mex.h"
#include"ControlCAN.h"
#pragma comment(lib,"ControlCAN.lib")
void mexFunction(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[])
{
	if( nrhs != 5)  //判断输入参数的个数
	{
		mexErrMsgTxt("输入参数个数不对!");
		return;
	}
	double DevType  =*((double*)mxGetPr(prhs[0]));
	double DevIndex =*((double*)mxGetPr(prhs[1]));
	double CANIndex =*((double*)mxGetPr(prhs[2]));

	double Len = *((double*)mxGetPr(prhs[3]));
	double WaitTime = *((double*)mxGetPr(prhs[4]));


	PVCI_CAN_OBJ  ptVCI_CAN_OBJ = new VCI_CAN_OBJ[Len];
    nlhs = 2;
	//创建一个1*1的实时double类型的矩阵
	plhs[0]=mxCreateDoubleMatrix(1,1,mxREAL);
	//得到输出的第一个参数的指针
	double *output0 = (double*)mxGetPr(plhs[0]);
	*output0=VCI_Receive(DevType,DevIndex,CANIndex,ptVCI_CAN_OBJ,Len,WaitTime);
	int nRows = *output0;
	int nCols = 18;
    plhs[1]=mxCreateDoubleMatrix(nRows,nCols,mxREAL);
	double *output1 = (double*)mxGetPr(plhs[1]);
    PVCI_CAN_OBJ p=ptVCI_CAN_OBJ;
	for (int nRow=0;nRow<nRows;nRow++,p++)
	{
		for(int nCol=0;nCol<nCols;nCol++)
		{
			switch(nCol)
			{
			case 0:
				output1[nCol*nRows+nRow]=p->ID;
				break;
			case 1:
				output1[nCol*nRows+nRow]=p->TimeStamp;
				break;
			case 2:
				output1[nCol*nRows+nRow]=p->TimeFlag;
				break;
			case 3:
				output1[nCol*nRows+nRow]=p->SendType;
				break;
			case 4:
				output1[nCol*nRows+nRow]=p->RemoteFlag;
				break;
			case 5:
				output1[nCol*nRows+nRow]=p->ExternFlag;
				break;
			case 6:
				output1[nCol*nRows+nRow]=p->DataLen;
				break;
			case 7:
				output1[nCol*nRows+nRow]=p->Data[0];
				break;
			case 8:
				output1[nCol*nRows+nRow]=p->Data[1];
				break;
			case 9:
				output1[nCol*nRows+nRow]=p->Data[2];
				break;
			case 10:
				output1[nCol*nRows+nRow]=p->Data[3];
				break;
			case 11:
				output1[nCol*nRows+nRow]=p->Data[4];
				break;
			case 12:
				output1[nCol*nRows+nRow]=p->Data[5];
				break;
			case 13:
				output1[nCol*nRows+nRow]=p->Data[6];
				break;
			case 14:
				output1[nCol*nRows+nRow]=p->Data[7];
				break;
			case 15:
				output1[nCol*nRows+nRow]=p->Reserved[0];
				break;
			case 16:
				output1[nCol*nRows+nRow]=p->Reserved[1];
				break;
			case 17:
				output1[nCol*nRows+nRow]=p->Reserved[2];
				break;
			default:
				break;
			}

		}
	}
  
	delete [] ptVCI_CAN_OBJ;
}

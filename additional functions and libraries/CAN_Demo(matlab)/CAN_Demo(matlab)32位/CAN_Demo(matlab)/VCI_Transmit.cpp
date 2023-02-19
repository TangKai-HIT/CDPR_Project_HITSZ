#include "mex.h"
#include"ControlCAN.h"
#include <string.h>
#pragma comment(lib,"ControlCAN.lib")
void mexFunction(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[])
{
	
	if( nrhs != 5)  //判断输入参数的个数
	{
		mexErrMsgTxt("输入参数个数不对!");
	}
	double DevType =*((double*)mxGetPr(prhs[0]));
	double DevIndex =*((double*)mxGetPr(prhs[1]));
	double CANIndex =*((double*)mxGetPr(prhs[2]));
	double *inData;
	inData = mxGetPr(prhs[3]);
	double Len = *((double*)mxGetPr(prhs[4]));
    int nRows = mxGetM(prhs[3]);
	int nCols = mxGetN(prhs[3]);
	PVCI_CAN_OBJ  ptVCI_CAN_OBJ = new VCI_CAN_OBJ[Len];
    PVCI_CAN_OBJ p = ptVCI_CAN_OBJ;
	for (int nRow=0;nRow<nRows;nRow++,p++)
	{
		for (int nCol=0;nCol<nCols;nCol++)
		{
			switch(nCol)
			{
			case 0:
				p->ID = inData[nCol*nRows+nRow];//inData[(nCols-1-nCol)*nRows+nRow];
				break;
			case 1:
				p->TimeStamp = inData[nCol*nRows+nRow];
				break;
			case 2:
				p->TimeFlag = inData[nCol*nRows+nRow];
				break;
			case 3:
				p->SendType = inData[nCol*nRows+nRow];
				break;
			case 4:
				p->RemoteFlag = inData[nCol*nRows+nRow];
				break;
			case 5:
				p->ExternFlag = inData[nCol*nRows+nRow];
				break;
			case 6:
				p->DataLen = inData[nCol*nRows+nRow];
				break;
			case 7:
				p->Data[0] = inData[nCol*nRows+nRow];
				break;
			case 8:
				p->Data[1] = inData[nCol*nRows+nRow];
				break;
			case 9:
				p->Data[2] = inData[nCol*nRows+nRow];
				break;
			case 10:
				p->Data[3] = inData[nCol*nRows+nRow];
				break;
			case 11:
				p->Data[4] = inData[nCol*nRows+nRow];
				break;
			case 12:
				p->Data[5] = inData[nCol*nRows+nRow];
				break;
			case 13:
				p->Data[6] = inData[nCol*nRows+nRow];
				break;
			case 14:
				p->Data[7] = inData[nCol*nRows+nRow];
				break;
			case 15:
				p->Reserved[0] = inData[nCol*nRows+nRow];
				break;
			case 16:
				p->Reserved[1] = inData[nCol*nRows+nRow];
				break;
			case 17:
				p->Reserved[2] = inData[nCol*nRows+nRow];
				break;
			default:
				break;
			}
		}
	}

	//创建一个1*1的实时double类型的矩阵
	plhs[0]=mxCreateDoubleMatrix(1,1,mxREAL);
	//得到输出的第一个参数的指针
	double *output =(double*)mxGetPr(plhs[0]);
	*output=VCI_Transmit(DevType,DevIndex,CANIndex,ptVCI_CAN_OBJ,Len);
	delete [] ptVCI_CAN_OBJ;
}

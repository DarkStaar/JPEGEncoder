#ifndef NXNDCT_H_
#define NXNDCT_H_
#include <QDebug>
		

void GenerateDCTmatrix(double* DCTKernel, int order);

void DCT(const uchar input[], short output[], int N, double* DCTKernel);
    
void IDCT(const short input[], uchar output[], int N, double* DCTKernel);
   
void extendBorders(uchar* input, int xSize, int ySize, int N, uchar** output, int* newXSize, int* newYSize);

void cropImage(uchar* input, int xSize, int ySize, uchar* output, int newXSize, int newYSize);

//Dodate funkcije

void DCTN(const char input[], short output[], int N, double* DCTKernel);

void extendingBordersN(char* input, int xSize, int ySize, int N, char** p_output, int* newXSize, int* newYSize);

void performZigZag(uint8_t matrix[], int N);

void performZigZagShort(short matrix[], int N);

#endif // NXNDCT_H_

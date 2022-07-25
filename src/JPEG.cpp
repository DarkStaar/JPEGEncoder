#include "JPEG.h"
#include "NxNDCT.h"

#include "JPEGBitStreamWriter.h"


#define DEBUG(x) do{ qDebug() << #x << " = " << x;}while(0)



// quantization tables from JPEG Standard, Annex K
const uint8_t QuantLuminance[8*8] =
    { 16, 11, 10, 16, 24, 40, 51, 61,
      12, 12, 14, 19, 26, 58, 60, 55,
      14, 13, 16, 24, 40, 57, 69, 56,
      14, 17, 22, 29, 51, 87, 80, 62,
      18, 22, 37, 56, 68,109,103, 77,
      24, 35, 55, 64, 81,104,113, 92,
      49, 64, 78, 87,103,121,120,101,
      72, 92, 95, 98,112,100,103, 99 };
const uint8_t QuantChrominance[8*8] =
    { 17, 18, 24, 47, 99, 99, 99, 99,
      18, 21, 26, 66, 99, 99, 99, 99,
      24, 26, 56, 99, 99, 99, 99, 99,
      47, 66, 99, 99, 99, 99, 99, 99,
      99, 99, 99, 99, 99, 99, 99, 99,
      99, 99, 99, 99, 99, 99, 99, 99,
      99, 99, 99, 99, 99, 99, 99, 99,
      99, 99, 99, 99, 99, 99, 99, 99 };










uint8_t quantQuality(uint8_t quant, uint8_t quality) {
    // Convert to an internal JPEG quality factor, formula taken from libjpeg
    int16_t q = quality < 50 ? 5000 / quality : 200 - quality * 2;
    return clamp((quant * q + 50) / 100, 1, 255);
}


//JPEGBitStreamWriter streamer("example.jpg");
void performJPEGEncoding(uchar Y_buff[], char U_buff[], char V_buff[], int xSize, int ySize, int quality)
{
    DEBUG(quality);

    uchar* Y = new uchar[8*8];
    char* U = new char[8*8];
    char* V = new char[8*8];
    short* Ys = new short[8*8];
    short* Us = new short[8*8];
    short* Vs = new short[8*8];
    uchar* input1;
    char* input2;
    char* input3;
    int xNewSize,yNewSize;
    int xNewestsize,yNewestsize;

    extendBorders(Y_buff,xSize,ySize,16,&input1,&xNewSize,&yNewSize);
    extendingBordersN(U_buff,xSize/2,ySize/2,8,&input2,&xNewestsize,&yNewestsize);
    extendingBordersN(V_buff,xSize/2,ySize/2,8,&input3,&xNewestsize,&yNewestsize);



    uint8_t Luminance[8*8];
    uint8_t Chrominance[8*8];

    for(int i = 0;i<64;i++){
        Luminance[i] = quantQuality(QuantLuminance[i],quality);
    }

    for(int i = 0;i<64;i++){
       Chrominance[i] = quantQuality(QuantChrominance[i],quality);
    }

    auto s = new JPEGBitStreamWriter("example.jpg");

    //TODO

    performZigZag(Luminance,8);
    performZigZag(Chrominance,8);

    double Kernel[64];
    GenerateDCTmatrix(Kernel,8);




    s->writeHeader();
    s->writeQuantizationTables(Luminance,Chrominance);
    s->writeImageInfo(xSize,ySize);
    s->writeHuffmanTables();

    for(int i = 0 ; i <ySize;i=i+16){
        for(int j = 0; j < xSize;j=j+16)
        {

            for(int k = 0; k < 8; k++)
            {
                for(int l = 0; l < 8; l++)
                {
                    Y[(k*8)+l] = input1[(i+k)*xNewSize+(j+l)];
                }
            }

            DCT(Y,Ys,8,Kernel);
            performZigZagShort(Ys,8);

            for(int m = 0;m<64;m++)
            {
                Ys[m] = Ys[m]/Luminance[m];
            }
            s->writeBlockY(Ys);

            for(int k=0;k<8;k++)
            {
                for(int l=0;l<8;l++)
                {
                    Y[(k*8)+l] = input1[(i+k)*xNewSize+(j+l+8)];
                }
            }

            DCT(Y,Ys,8,Kernel);
            performZigZagShort(Ys,8);

            for(int m = 0;m<64;m++)
            {
                Ys[m] = Ys[m]/Luminance[m];
            }
            s->writeBlockY(Ys);


            for(int k=0;k<8;k++)
            {
                for(int l=0;l<8;l++)
                {
                    Y[(k*8)+l] = input1[(i+k+8)*xNewSize+(j+l)];
                }
            }

            DCT(Y,Ys,8,Kernel);
            performZigZagShort(Ys,8);

            for(int m = 0;m<64;m++)
            {
                Ys[m] = Ys[m]/Luminance[m];
            }
            s->writeBlockY(Ys);

            for(int k=0;k<8;k++)
            {
                for(int l=0;l<8;l++)
                {
                    Y[(k*8)+l] = input1[(i+k+8)*xNewSize+(j+l+8)];
                }
            }

            DCT(Y,Ys,8,Kernel);
            performZigZagShort(Ys,8);

            for(int m = 0;m<64;m++)
            {
                Ys[m] = Ys[m]/Luminance[m];
            }
            s->writeBlockY(Ys);


            for(int k = 0;k<8;k++)
            {
                for(int l=0;l<8;l++)
                {
                    U[k*8 + l] = input2[((i/2)+k)*(xNewSize/2) + (j/2) + l];
                }
            }

            DCTN(U,Us,8,Kernel);
            performZigZagShort(Us,8);
            for(int m = 0;m<64;m++)
            {
                Us[m] = Us[m]/Chrominance[m];
            }
            s->writeBlockU(Us);

            for(int k = 0;k<8;k++)
            {
                for(int l=0;l<8;l++)
                {
                    V[k*8 + l] = input3[((i/2)+k)*(xNewSize/2) + (j/2) + l];
                }
            }

            DCTN(V,Vs,8,Kernel);
            performZigZagShort(Vs,8);
            for(int m = 0;m<64;m++)
            {
                Vs[m] = Vs[m]/Chrominance[m];
            }
            s->writeBlockV(Vs);



        }
    }




    s->finishStream();




    delete s;
}

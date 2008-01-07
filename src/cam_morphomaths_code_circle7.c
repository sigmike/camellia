/***************************************
 *
 *  Camellia Image Processing Library
 *

    The Camellia Image Processing Library is an open source low-level image processing library.
    As it uses the IplImage structure to describe images, it is a good replacement to the IPL (Intel) library
    and a good complement to the OpenCV library. It includes a lot of functions for image processing
    (filtering, morphological mathematics, labeling, warping, loading/saving images, etc.),
    some of them being highly optimized; It is also cross-platform and robust. It is doxygen-documented
    and examples of use are provided.

    This software library is an outcome of the Camellia european project (IST-2001-34410).
    It was developped by the Ecole des Mines de Paris (ENSMP), in coordination with
    the other partners of the project.

  ==========================================================================

    Copyright (c) 2002-2007, Ecole des Mines de Paris - Centre de Robotique
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

        * Redistributions of source code must retain the above copyright
          notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
          notice, this list of conditions and the following disclaimer
          in the documentation and/or other materials provided with the distribution.
        * Neither the name of the Ecole des Mines de Paris nor the names of
          its contributors may be used to endorse or promote products
          derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
    THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
    PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR 
    CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
    EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
    PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
    PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
    LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
    NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

  ==========================================================================
*/

/* Fundamental Morphological Mathematics Kernel
 * C code */

i=x-CAM_MM_NEIGHB+1;
if (firsttime) {

    valX[0]=linesPtr[0][i+2];
    valX[1]=linesPtr[1][i+1];
    valX[0]=valX[valX[1] XOP valX[0]];
    valX[1]=linesPtr[2][i];
    valX[0]=valX[valX[1] XOP valX[0]];
    valX[1]=linesPtr[3][i];
    valX[0]=valX[valX[1] XOP valX[0]];
    valX[1]=linesPtr[4][i];
    valX[0]=valX[valX[1] XOP valX[0]];
    valX[1]=linesPtr[5][i+1];
    valX[0]=valX[valX[1] XOP valX[0]];
    valX[1]=linesPtr[6][i+2];
    valX[0]=valX[valX[1] XOP valX[0]];

    tabX[0]=linesPtr[0][i+3];
    tabX[1]=linesPtr[1][i+2];
    tabX[0]=tabX[tabX[1] XOP tabX[0]];
    tabX[1]=linesPtr[2][i+1];
    tabX[0]=tabX[tabX[1] XOP tabX[0]];
    tabX[1]=linesPtr[3][i+1];
    tabX[0]=tabX[tabX[1] XOP tabX[0]];
    tabX[1]=linesPtr[4][i+1];
    tabX[0]=tabX[tabX[1] XOP tabX[0]];
    tabX[1]=linesPtr[5][i+2];
    tabX[0]=tabX[tabX[1] XOP tabX[0]];
    tabX[1]=linesPtr[6][i+3];
    tabX[0]=tabX[tabX[1] XOP tabX[0]];

    tabX[1]=linesPtr[1][i+3];
    tabX[2]=linesPtr[2][i+2];
    tabX[1]=tabX[1+(tabX[2] XOP tabX[1])];
    tabX[2]=linesPtr[3][i+2];
    tabX[1]=tabX[1+(tabX[2] XOP tabX[1])];
    tabX[2]=linesPtr[4][i+2];
    tabX[1]=tabX[1+(tabX[2] XOP tabX[1])];
    tabX[2]=linesPtr[5][i+3];
    tabX[1]=tabX[1+(tabX[2] XOP tabX[1])];
    
    tabX[2]=linesPtr[0][i+4];
    tabX[3]=linesPtr[6][i+4];
    tabX[2]=tabX[2+(tabX[3] XOP tabX[2])];

    tabX[3]=linesPtr[1][i+4];
    tabX[4]=linesPtr[2][i+3];
    tabX[3]=tabX[3+(tabX[4] XOP tabX[3])];
    tabX[4]=linesPtr[3][i+3];
    tabX[3]=tabX[3+(tabX[4] XOP tabX[3])];
    tabX[4]=linesPtr[4][i+3];
    tabX[3]=tabX[3+(tabX[4] XOP tabX[3])];
    tabX[4]=linesPtr[5][i+4];
    tabX[3]=tabX[3+(tabX[4] XOP tabX[3])];

    tabX[4]=linesPtr[2][i+4];
    tabX[5]=linesPtr[3][i+4];
    tabX[4]=tabX[4+(tabX[5] XOP tabX[4])];
    tabX[5]=linesPtr[4][i+4];
    tabX[4]=tabX[4+(tabX[5] XOP tabX[4])];

    tabX[5]=linesPtr[1][i+5];
    tabX[6]=linesPtr[5][i+5];
    tabX[5]=tabX[5+(tabX[6] XOP tabX[5])];

    tabX[6]=linesPtr[2][i+5];
    tabX[7]=linesPtr[3][i+5];
    tabX[6]=tabX[6+(tabX[7] XOP tabX[6])];
    tabX[7]=linesPtr[4][i+5];
    tabX[6]=tabX[6+(tabX[7] XOP tabX[6])];

} else {

    // Transfer data from right to left
    valX[0]=tabX[0];
    tabX[0]=tabX[1+(tabX[2] XOP tabX[1])];
    tabX[1]=tabX[3];
    tabX[2]=linesPtr[0][i+4];
    tabX[3]=linesPtr[6][i+4];
    tabX[2]=tabX[2+(tabX[3] XOP tabX[2])];
    tabX[3]=tabX[4+(tabX[5] XOP tabX[4])];
    tabX[4]=tabX[6];
    tabX[5]=linesPtr[1][i+5];
    tabX[6]=linesPtr[5][i+5];
    tabX[5]=tabX[5+(tabX[6] XOP tabX[5])];
    tabX[6]=tabX[7];

}

tabX[7]=linesPtr[2][i+6];
tabX[8]=linesPtr[3][i+6];
tabX[7]=tabX[7+(tabX[8] XOP tabX[7])];
tabX[8]=linesPtr[4][i+6];
tabX[7]=tabX[7+(tabX[8] XOP tabX[7])];

for (i=0;i<8;i++) {
    valX[1]=tabX[i];
    valX[0]=valX[valX[1] XOP valX[0]];
}


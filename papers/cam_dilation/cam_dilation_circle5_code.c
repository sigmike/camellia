i=x-CAM_MM_NEIGHB+1;
if (firsttime) {

    valX[0]=linesPtr[0][i+1];
    valX[1]=linesPtr[1][i];
    valX[0]=valX[valX[1] XOP valX[0]];
    valX[1]=linesPtr[2][i];
    valX[0]=valX[valX[1] XOP valX[0]];
    valX[1]=linesPtr[3][i];
    valX[0]=valX[valX[1] XOP valX[0]];
    valX[1]=linesPtr[4][i+1];
    valX[0]=valX[valX[1] XOP valX[0]];

    tabX[0]=linesPtr[0][i+2];
    tabX[1]=linesPtr[1][i+1];
    tabX[0]=tabX[tabX[1] XOP tabX[0]];
    tabX[1]=linesPtr[2][i+1];
    tabX[0]=tabX[tabX[1] XOP tabX[0]];
    tabX[1]=linesPtr[3][i+1];
    tabX[0]=tabX[tabX[1] XOP tabX[0]];
    tabX[1]=linesPtr[4][i+2];
    tabX[0]=tabX[tabX[1] XOP tabX[0]];

    tabX[1]=linesPtr[1][i+2];
    tabX[2]=linesPtr[2][i+2];
    tabX[1]=tabX[1+(tabX[2] XOP tabX[1])];
    tabX[2]=linesPtr[3][i+2];
    tabX[1]=tabX[1+(tabX[2] XOP tabX[1])];
    
    tabX[2]=linesPtr[0][i+3];
    tabX[3]=linesPtr[4][i+3];
    tabX[2]=tabX[2+(tabX[3] XOP tabX[2])];

    tabX[3]=linesPtr[1][i+3];
    tabX[4]=linesPtr[2][i+3];
    tabX[3]=tabX[3+(tabX[4] XOP tabX[3])];
    tabX[4]=linesPtr[3][i+3];
    tabX[3]=tabX[3+(tabX[4] XOP tabX[3])];

} else {
    // Transfer data from right to left
    valX[0]=tabX[0];
    tabX[0]=tabX[1+(tabX[2] XOP tabX[1])];
    tabX[1]=tabX[3];
    tabX[2]=linesPtr[0][i+3];
    tabX[3]=linesPtr[4][i+3];
    tabX[2]=tabX[2+(tabX[3] XOP tabX[2])];
    tabX[3]=tabX[4];
}

tabX[4]=linesPtr[1][i+4];
tabX[5]=linesPtr[2][i+4];
tabX[4]=tabX[4+(tabX[5] XOP tabX[4])];
tabX[5]=linesPtr[3][i+4];
tabX[4]=tabX[4+(tabX[5] XOP tabX[4])];

for (i=0;i<5;i++) {
    valX[1]=tabX[i];
    valX[0]=valX[valX[1] XOP valX[0]];
}


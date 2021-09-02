/* 
Purpose:
Scratchpad for working on array functions in cpp
Aug 31 2021
*/

#include <sstream>
#include <cmath>
#include <vector>

// Global state variables
const float EPSILON = 0.001; // for float equality test

// Utillity functions
bool floatIsEq(float x, float y)
{
    bool isEqual = false;
    if (std::abs(x - y) < EPSILON)
        isEqual = true;
    return isEqual;
}

template <unsigned int xSize> // populate array size in function param at compile time
std::vector<int> getNeighborsIx(float (&axisInc)[xSize], int axisSize, float axisSearch)
{
    // Purpose:
    // Given monotonically increasing axis - axisInc, and axis value of interest axisSearch,
    // return the indicies of the numerically neigboring axis values.
    // The first value in the returned vector gives the number of neighbors.
    // ie: axisInc = {1, 2, 3, 4}; axisSize = 4, axisVal=3.5; neighbor vals: 3,4 returns: {2,2,3}
    // ie2:axisInc = {1, 2, 3, 4}; axisSize = 4, axisVal=-1; neighbor vals: 1 returns: {1,0}
    // Note:
    // in Cpp, array as parameter decay's to a pointer to first element


    int ixTest = 0;
    float axisVal = axisInc[ixTest];
    int nNeighbor = 1; // for point in list, at least 1 neighbor, max of 2
    std::vector<int> nAndNeighborsIx = {nNeighbor,0,0};
    ixTest = 1;

    if (axisSearch > axisVal && not floatIsEq(axisSearch, axisVal))
    {
        // continue if not special case where
        //  axisSearch is "Left" numerically of first element
        while (ixTest < axisSize)
        {
            axisVal = axisInc[ixTest];
            if (axisSearch < axisVal)
            {
                // stop search, element left of ixTest is neighbor
                nNeighbor = 2;
                nAndNeighborsIx[0] = nNeighbor; // first element is n neibors
                nAndNeighborsIx[1] = ixTest - 1; // index of first val lt axisSearch
                nAndNeighborsIx[2] = ixTest; // index of first val gt axisSearch
                break;
            }
            ixTest += 1;
        }
        if (nNeighbor == 1)
        {
            // other special case, axisSearch is "Right" of right-most array val
            nAndNeighborsIx[1] = axisSize - 1;
        }
    }
    return nAndNeighborsIx;
}

float getRelDistToLNorm(float lVal, float rVal, float testVal)
{
    // Purpose:
    // Return relative distance of testVal to lVal and rVal, normalized to 0..1
    // NOTE: Unused
    float relDistNormL = (testVal - lVal)/(rVal - lVal);
    return relDistNormL;
}

//float floatInterp2(float xaxis[], float yaxis[], float table[][], float xval, float yval)
template <unsigned int xSize, unsigned int ySize> // populate array size in function param at compile time
float floatInterp2(float (&xaxis)[xSize], float (&yaxis)[ySize], float (&table)[xSize][ySize], float xval, float yval)
{
    // Purpose:
    // 2D lookup table
    // endpoint and beyond: latch value
    // algo:
    // determine neighboring axis values of input xval yval
    // directly lookup the four values from the table corressponding to the neighbors
    // determine normalized position of xval yval to neighboring axis values
    // multiply 
    // imp notes:
    // using primitive type because tables/axis are calibrations from params.yaml &
    // therefore resize / insert / swap ect are useless - static size / val's.
    // However, do not assume size of axis / table - work for size 1 and up.
    //     _|y0|y1|y2|..|yn   <--yaxis
    //    x1|a1|b1|c1|..|   <-:
    //    x2|a2|b2|c2|      <---- table 
    //    ..|
    //    xm|
    //    ^xaxis

    float interp2Result = 0.0; // return value to be stored here

    // get array size, assume table is size: [xSize,ySize]
    // UPDATE: done with template & pass by reference

    // Find indicies of axis values neighboring value of interest
    // returns: {nNeighbors 1||2, ixL, ixR}
    std::vector<int> nAndNeigX = getNeighborsIx(xaxis, xSize, xval);
    std::vector<int> nAndNeigY = getNeighborsIx(yaxis, ySize, yval);

    if(nAndNeigX[0] == 2 && nAndNeigY[0] ==2)
    {
        // Use weighted mean techniqe to compute bilinear interpolation
        // Normal case - xval & yval are within limits of xaxis and yaxis
        float denom = (xaxis[nAndNeigX[2]]-xaxis[nAndNeigX[1]])*
                      (yaxis[nAndNeigY[2]]-yaxis[nAndNeigY[1]]);
        float w11 = (xaxis[nAndNeigX[2]] - xval)*(yaxis[nAndNeigY[2]] - yval)/denom;
        float w12 = (xaxis[nAndNeigX[2]] - xval)*(yval - yaxis[nAndNeigY[1]])/denom;
        float w21 = (xval - xaxis[nAndNeigX[1]])*(yaxis[nAndNeigY[2]] - yval)/denom;
        float w22 = (xval - xaxis[nAndNeigX[1]])*(yval - yaxis[nAndNeigY[1]])/denom;
        interp2Result = w11*table[nAndNeigX[1]][nAndNeigY[1]] +
                        w12*table[nAndNeigX[1]][nAndNeigY[2]] +
                        w21*table[nAndNeigX[2]][nAndNeigY[1]] +
                        w22*table[nAndNeigX[2]][nAndNeigY[2]];
    }
    else if(nAndNeigX[0] == 1 && nAndNeigY[0] == 1)
    {
        // Simple case, value is outside both axis - return most extreme point
        interp2Result = table[nAndNeigX[1]][nAndNeigY[1]];
    }
    else if(nAndNeigX[0] == 1)
    {
        // Case xval is outside of xaxis, yval is within y axis
        float distToBound = getRelDistToLNorm(yaxis[nAndNeigY[1]], yaxis[nAndNeigY[2]], yval);
        float delta =   table[nAndNeigX[1]][nAndNeigY[2]] -
                        table[nAndNeigX[1]][nAndNeigY[1]];
        interp2Result = table[nAndNeigX[1]][nAndNeigY[1]] + 
                        delta*distToBound;
    }
    else if(nAndNeigY[0] == 1)
    {
        // Case yval is outside of yaxis, xval is within x axis
        float distToBound = getRelDistToLNorm(xaxis[nAndNeigX[1]], xaxis[nAndNeigX[2]], xval);
        float delta =   table[nAndNeigX[2]][nAndNeigY[1]] -
                        table[nAndNeigX[1]][nAndNeigY[1]];
        interp2Result = table[nAndNeigX[1]][nAndNeigY[1]] + 
                        delta*distToBound;
    }
    else
    {
        // Catch-all, shouldn't happen
        interp2Result = 0.0042069; // yolo 
    }

    return interp2Result;
}


int main(int argc, char **argv)
{
    float xaxis[4] = {1.0,2.0,3.0,4.0};
    float yaxis[3] = {1.0,2.0,3.0};
/*
    float **table;
    table = new float *[4];
    table[0] = {1.0, 2.0, 3.0};
    table[1] = {4.0, 5.0, 6.0};
    table[2] = {7.0, 8.0, 9.0};
    table[3] = {1.0, 2.0, 3.0};
*/

    float table[4][3] = { 
                        {1.0, 2.0, 3.0}, 
                        {4.0, 5.0, 6.0}, 
                        {7.0, 8.0, 9.0}, 
                        {1.0, 2.0, 3.0}, 
                        };

    // normal calse
    float xval = 1.1;
    float yval = 1.2;
    float interp2 = floatInterp2(xaxis, yaxis, table, xval, yval);

    // low x
    xval = 0.0;
    yval = 1.2;
    float interp2b = floatInterp2(xaxis, yaxis, table, xval, yval);

    // high x
    xval = 10.0;
    yval = 1.2;
    float interp2c = floatInterp2(xaxis, yaxis, table, xval, yval);

    // low y
    xval = 1.1;
    yval = 0.0;
    float interp2d = floatInterp2(xaxis, yaxis, table, xval, yval);

    // high y
    xval = 1.1;
    yval = 10.0;
    float interp2e = floatInterp2(xaxis, yaxis, table, xval, yval);

    // high xy
    xval = 11.1;
    yval = 10.0;
    float interp2f = floatInterp2(xaxis, yaxis, table, xval, yval);

    // low xy
    xval = 0.1;
    yval = 0.0;
    float interp2g = floatInterp2(xaxis, yaxis, table, xval, yval);

    // on xy
    xval = 1.0;
    yval = 1.0;
    float interp2h = floatInterp2(xaxis, yaxis, table, xval, yval);

    return 0;
}
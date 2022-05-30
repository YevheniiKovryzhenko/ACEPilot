/*
 * tools.c
 *
 * Author:	Yevhenii Kovryzhenko, Department of Aerospace Engineering, Auburn University.
 * Contact: yzk0058@auburn.edu
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL I
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Last Edit:  05/29/2022 (MM/DD/YYYY)
 */


#include "tools.h"

double finddt_s(uint64_t ti)
{
    double dt_s = (rc_nanos_since_boot() - ti) / (1e9);
    return dt_s;
}

double cubicPol(double xi, double xf, double xdoti, double xdotf, float tt_s, double dt)
{
    double dX = (2 * xi - 2 * xf + tt_s * (xdotf + xdoti)) / (pow(tt_s, 3)) * pow(dt, 3) -
        (3 * xi - 3 * xf + tt_s * (xdotf + 2 * xdoti)) / (pow(tt_s, 2)) * pow(dt, 2) +
        xdoti * dt + xi;
    /* Cubic BVP:
    initial position and velocity is 0
    final position is xf and velocity is 0
    tt_s is total time in seconds
    */
    // double dX = (-2*xf/(tt_s*tt_s*tt_s)) * dt*dt*dt + 3*xf/(tt_s*tt_s) * dt*dt;
    return dX;
}


int scan_file_d(void)
{
    uint64_t run_time = rc_nanos_since_boot();

    FILE* myFile;
    //myFile = fopen("trajectories/traj_min_snap.txt", "r");
    myFile = fopen("trajectories/test_polys.txt", "r");

    int i;
    int ii;
    int ind;

    int x_n_coef;
    int x_n_int;
    /*
    int y_n_coef;
    int y_n_int;
    int z_n_coef;
    int z_n_int;
    */

    if (myFile == NULL)
    {
        printf("\nERROR: Can't open file for scanning.");
        fclose(myFile);
        return -1;
    }
    else
    {
        printf("\nTrying to read the file...");

        ind = 0; //reset read index just in case

        // load the size of the x dimension:
        fscanf(myFile, "%d", &x_n_coef);
        ind++;
        fscanf(myFile, "%d", &x_n_int);
        ind++;
        double data_x[x_n_coef][x_n_int];

        for (ii = 0; ii < x_n_int; ii++)
        {
            for (i = 0; i < x_n_coef; i++)
            {
                fscanf(myFile, "%lf", &data_x[i][ii]);
                ind++;
            }
        }
        /*
        // load the size of the y dimension:
        fscanf(myFile, "%d", &y_n_coef);
        ind++;
        fscanf(myFile, "%d", &y_n_int);
        ind++;
        double data_y[y_n_coef][y_n_int];

        for (ii = 0; i < y_n_int; i++)
        {
            for (i = 0; i < y_n_coef; i++)
            {
                fscanf(myFile, "%lf", &data_y[i][ii]);
                ind++;
            }
        }


        // load the size of the z dimension:
        fscanf(myFile, "%d", &z_n_coef);
        ind++;
        fscanf(myFile, "%d", &z_n_int);
        ind++;
        double data_z[z_n_coef][z_n_int];

        for (ii = 0; i < y_n_int; i++)
        {
            for (i = 0; i < y_n_coef; i++)
            {
                fscanf(myFile, "%lf", &data_z[i][ii]);
                ind++;
            }
        }
        */

    }

    printf("\nX size is: %d %d\n", x_n_coef, x_n_int);
    //printf("\nY size is: %d %d\n", y_n_coef, y_n_int);
    //printf("\nZ size is: %d %d\n", z_n_coef, z_n_int);
    printf("\n Time to read is %f (s), number of datas %d\n", finddt_s(run_time), ind);

    fclose(myFile);
    return 0;
}

/*
// This function multiplies mat1[][] and mat2[][],
// and stores the result in res[][]
void multiply(double** mat1, double** mat2, double** res)
{
    int i, j, k;
    for (i = 0; i < 3; i++)
    {
        for (j = 0; j < 3; j++)
        {
            res[i][j] = 0;
            for (k = 0; k < 3; k++) res[i][j] += mat1[i][k] * mat2[k][j];
        }
    }
}


void rotate_i2b(double** vec)
{
    double R3[3][3] = { \
        {cos(state_estimate.yaw),   sin(state_estimate.yaw),    0.0},\
        {-sin(state_estimate.yaw),  cos(state_estimate.yaw),    0.0},\
        {0.0,                       0.0,                        1.0}};

    double R2[3][3] = { \
        {cos(state_estimate.pitch), 0.0,    -sin(state_estimate.pitch)},\
        {0.0,                       1.0,                        0.0},\
        {sin(state_estimate.pitch), 0.0,    cos(state_estimate.pitch)}};

    double R1[3][3] = { \
        {1.0,                       0.0,                        0.0},\
        {0.0, cos(state_estimate.roll),     sin(state_estimate.roll)},\
        {0.0, -sin(state_estimate.roll),    cos(state_estimate.roll)}};

    multiply(double mat1, double mat2, double res)
}
*/
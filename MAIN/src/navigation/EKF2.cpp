/*
 * EKF.cpp
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
 * Last Edit:  09/05/2022 (MM/DD/YYYY)
 *
 * Summary :
 * This class defines an Extended Kalman Filter V-1 operations.
 * Currently estimates attitude from IMU0.
 *
 * this code is based on the following thread:
 *	https://github.com/beagleboard/librobotcontrol/issues/74#issuecomment-331214723
 *
 */

#include "EKF2.hpp"
#include "settings.hpp"
#include "rc/math/quaternion.h"
#include "rc/math.h"
#include "tools.h"

#ifndef GET_VARIABLE_NAME
#define GET_VARIABLE_NAME(Variable) (#Variable)
#endif // !GET_VARIABLE_NAME

EKF2_t EKF2{};


void IMU_EKF2(double P[16], double q[4], const double Cov_info[3], const double
    omega[3], double accel[3], double mag[3], double dt, bool initilized,
    bool use_mag);

char EKF2_t::march(double omega[3], double accel[3], double mag[3])
{
    if (!initialized)
    {
        enable_mag = settings.IMU0.compass.enable;
        num_heading_spins = 0;
        continuous_heading_NED = 0.0;
        quat_raw[0] = 1.0;
        quat_raw[1] = 0.0;
        quat_raw[2] = 0.0;
        quat_raw[3] = 0.0;
    }
    /* from NED to ENU */
    double om[3] = { omega[1], omega[0], -omega[2] };
    double acc[3] = { accel[1], accel[0], -accel[2] };
    double magn[3] = { mag[1], mag[0], -mag[2] };

    /* march EKF-V2 */
    IMU_EKF2(Pcov, quat_raw, Cov_info, om, acc, magn, finddt_s(time), initialized, enable_mag);
    time = rc_nanos_since_boot(); //log new timestamp

    /* check for NaNs*/
    if (isnan(quat_raw[0]))
    {
        printf("WARNING in EKF2: NaN output detected, resetting...\n");
        initialized = false;
        return -1;
    }

    quat_NED[0] = quat_raw[0];
    rotate2NED(ENU, &quat_NED[1], &quat_raw[1]);

    /* Final calculations */
    // normalize quaternion because we don't trust the EKF to do it
    rc_normalize_quaternion_array(quat_NED);
    // generate tait bryan angles
    rc_quaternion_to_tb_array(quat_NED, att_tb_NED);

    /*	Heading
    continuous heading is more annoying since we have to detect spins
    also make sign negative since NED coordinates has Z point down
    */
    double diff_heading = att_tb_NED[2] + (num_heading_spins * TWO_PI) - continuous_heading_NED;
    // detect the crossover point at +-PI and update num yaw spins
    if (diff_heading < -M_PI) num_heading_spins++;
    else if (diff_heading > M_PI) num_heading_spins--;

    // finally the new value can be written
    continuous_heading_NED = att_tb_NED[2] + (num_heading_spins * TWO_PI);


    initialized = true;
    return 0;
}

char EKF2_t::reset(void)
{
    initialized = false;
    return 0;
}

uint64_t EKF2_t::get_time(void)
{
    return time;
}
void EKF2_t::get_quat(double* buff)
{
    for (int i = 0; i < 4; i++) buff[i] = quat_NED[i];
    return;
}
void EKF2_t::get_quat_raw(double* buff)
{
    for (int i = 0; i < 4; i++) buff[i] = quat_raw[i];
    return;
}
void EKF2_t::get_tb(double* buff)
{
    for (int i = 0; i < 3; i++) buff[i] = att_tb_NED[i];
    return;
}
double EKF2_t::get_continuous_heading(void)
{
    return continuous_heading_NED;
}



/** @name Logging class for EKF
* Defines how logging should be done for this class
*/
char EKF2_log_entry_t::update(EKF2_t& new_state)
{
    time = new_state.get_time();

    new_state.get_quat_raw(quat_raw);
    new_state.get_quat(quat_NED);
    new_state.get_tb(att_tb_NED);
    continuous_heading_NED = new_state.get_continuous_heading();
    return 0;
}
char EKF2_log_entry_t::print_vec(FILE* file, double* vec_in, int size)
{
    for (int i = 0; i < size; i++)
    {
        fprintf(file, ",%.4F", vec_in[i]);
    }
    return 0;
}

char EKF2_log_entry_t::print_header_vec(FILE* file, const char* prefix, const char* var_name, int size)
{
    for (int i = 0; i < size; i++)
    {
        fprintf(file, ",%s%s_%i", prefix, var_name, i);
    }
    return 0;
}

char EKF2_log_entry_t::print_header(FILE* file, const char* prefix)
{
    fprintf(file, ",%s%s", prefix, GET_VARIABLE_NAME(time));

    print_header_vec(file, prefix, GET_VARIABLE_NAME(quat_raw), 4);
    print_header_vec(file, prefix, GET_VARIABLE_NAME(quat_NED), 4);
    print_header_vec(file, prefix, GET_VARIABLE_NAME(att_tb_NED), 3);

    fprintf(file, ",%s%s", prefix, GET_VARIABLE_NAME(continuous_heading_NED));
    return 0;
}
char EKF2_log_entry_t::print_entry(FILE* file)
{
    fprintf(file, ",%" PRIu64, time);

    print_vec(file, quat_raw, 4);
    print_vec(file, quat_NED, 4);
    print_vec(file, att_tb_NED, 3);
    fprintf(file, ",%.4F", continuous_heading_NED);
    return 0;
}

static void mrdivide(const double A[12], const double B[9], double y[12]);
static double norm(const double x[3]);
static void mrdivide(const double A[12], const double B[9], double y[12])
{
    int rtemp;
    int r1;
    double b_A[9];
    int r2;
    int r3;
    double maxval;
    double a21;
    for (rtemp = 0; rtemp < 9; rtemp++) {
        b_A[rtemp] = B[rtemp];
    }

    r1 = 0;
    r2 = 1;
    r3 = 2;
    maxval = fabs(B[0]);
    a21 = fabs(B[1]);
    if (a21 > maxval) {
        maxval = a21;
        r1 = 1;
        r2 = 0;
    }

    if (fabs(B[2]) > maxval) {
        r1 = 2;
        r2 = 1;
        r3 = 0;
    }

    b_A[r2] = B[r2] / B[r1];
    b_A[r3] /= b_A[r1];
    b_A[3 + r2] -= b_A[r2] * b_A[3 + r1];
    b_A[3 + r3] -= b_A[r3] * b_A[3 + r1];
    b_A[6 + r2] -= b_A[r2] * b_A[6 + r1];
    b_A[6 + r3] -= b_A[r3] * b_A[6 + r1];
    if (fabs(b_A[3 + r3]) > fabs(b_A[3 + r2])) {
        rtemp = r2;
        r2 = r3;
        r3 = rtemp;
    }

    b_A[3 + r3] /= b_A[3 + r2];
    b_A[6 + r3] -= b_A[3 + r3] * b_A[6 + r2];
    for (rtemp = 0; rtemp < 4; rtemp++) {
        y[rtemp + (r1 << 2)] = A[rtemp] / b_A[r1];
        y[rtemp + (r2 << 2)] = A[4 + rtemp] - y[rtemp + (r1 << 2)] * b_A[3 + r1];
        y[rtemp + (r3 << 2)] = A[8 + rtemp] - y[rtemp + (r1 << 2)] * b_A[6 + r1];
        y[rtemp + (r2 << 2)] /= b_A[3 + r2];
        y[rtemp + (r3 << 2)] -= y[rtemp + (r2 << 2)] * b_A[6 + r2];
        y[rtemp + (r3 << 2)] /= b_A[6 + r3];
        y[rtemp + (r2 << 2)] -= y[rtemp + (r3 << 2)] * b_A[3 + r3];
        y[rtemp + (r1 << 2)] -= y[rtemp + (r3 << 2)] * b_A[r3];
        y[rtemp + (r1 << 2)] -= y[rtemp + (r2 << 2)] * b_A[r2];
    }
}

static double norm(const double x[3])
{
    double y;
    double scale;
    int k;
    double absxk;
    double t;
    y = 0.0F;
    scale = 1.17549435E-38F;
    for (k = 0; k < 3; k++) {
        absxk = fabs(x[k]);
        if (absxk > scale) {
            t = scale / absxk;
            y = 1.0F + y * t * t;
            scale = absxk;
        }
        else {
            t = absxk / scale;
            y += t * t;
        }
    }

    return scale * sqrt(y);
}

void IMU_EKF2(double P[16], double q[4], const double Cov_info[3], const double
    omega[3], double accel[3], double mag[3], double dt, bool initilized,
    bool use_mag)
{
    int i;
    double y;
    double Bk[12];
    static const signed char iv0[16] = { 10, 0, 0, 0, 0, 10, 0, 0, 0, 0, 10, 0, 0,
      0, 0, 10 };

    double Hk[12];
    int i0;
    double b_Bk[12];
    int i1;
    static const signed char iv1[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

    double S[9];
    double fv4[3];
    double b_accel[3];
    double qe[4];
    double c_Bk[16];
    double b_P[16];
    double scale;
    double absxk;
    double t;
    if (!initilized) {
        for (i = 0; i < 16; i++) {
            P[i] = iv0[i];
        }

        for (i = 0; i < 4; i++) {
            q[i] = 0.0F;
        }

        q[0] = 1.0F;
    }

    y = dt / 2.0F;
    Bk[0] = y * -q[1];
    Bk[4] = y * -q[2];
    Bk[8] = y * -q[3];
    Bk[1] = y * q[0];
    Bk[5] = y * -q[3];
    Bk[9] = y * q[2];
    Bk[2] = y * q[3];
    Bk[6] = y * q[0];
    Bk[10] = y * -q[1];
    Bk[3] = y * -q[2];
    Bk[7] = y * q[1];
    Bk[11] = y * q[0];
    for (i = 0; i < 4; i++) {
        y = 0.0F;
        for (i0 = 0; i0 < 3; i0++) {
            y += Bk[i + (i0 << 2)] * omega[i0];
            b_Bk[i + (i0 << 2)] = 0.0F;
            for (i1 = 0; i1 < 3; i1++) {
                b_Bk[i + (i0 << 2)] += Bk[i + (i1 << 2)] * (Cov_info[0] * iv1[i1
                    + 3 * i0]);
            }
        }

        q[i] += y;
        for (i0 = 0; i0 < 4; i0++) {
            y = 0.0F;
            for (i1 = 0; i1 < 3; i1++) {
                y += b_Bk[i + (i1 << 2)] * Bk[i0 + (i1 << 2)];
            }

            P[i + (i0 << 2)] += y;
        }
    }

    y = norm(accel);
    Hk[0] = -2.0F * q[2];
    Hk[3] = 2.0F * q[3];
    Hk[6] = -2.0F * q[0];
    Hk[9] = 2.0F * q[1];
    Hk[1] = 2.0F * q[1];
    Hk[4] = 2.0F * q[0];
    Hk[7] = 2.0F * q[3];
    Hk[10] = 2.0F * q[2];
    Hk[2] = 2.0F * q[0];
    Hk[5] = -2.0F * q[1];
    Hk[8] = -2.0F * q[2];
    Hk[11] = 2.0F * q[3];
    for (i = 0; i < 3; i++) {
        for (i0 = 0; i0 < 4; i0++) {
            b_Bk[i + 3 * i0] = 0.0F;
            for (i1 = 0; i1 < 4; i1++) {
                b_Bk[i + 3 * i0] += Hk[i + 3 * i1] * P[i1 + (i0 << 2)];
            }
        }

        for (i0 = 0; i0 < 3; i0++) {
            S[i + 3 * i0] = 0.0F;
            for (i1 = 0; i1 < 4; i1++) {
                S[i + 3 * i0] += b_Bk[i + 3 * i1] * Hk[i0 + 3 * i1];
            }
        }

        accel[i] /= y;
    }

    for (i = 0; i < 3; i++) {
        S[i + 3 * i] += Cov_info[1];
    }

    for (i = 0; i < 4; i++) {
        for (i0 = 0; i0 < 3; i0++) {
            b_Bk[i + (i0 << 2)] = 0.0F;
            for (i1 = 0; i1 < 4; i1++) {
                b_Bk[i + (i0 << 2)] += P[i + (i1 << 2)] * Hk[i0 + 3 * i1];
            }
        }
    }

    mrdivide(b_Bk, S, Bk);
    fv4[0] = 2.0F * q[1] * q[3] - 2.0F * q[0] * q[2];
    fv4[1] = 2.0F * q[0] * q[1] + 2.0F * q[2] * q[3];
    fv4[2] = ((q[0] * q[0] - q[1] * q[1]) - q[2] * q[2]) + q[3] * q[3];
    for (i = 0; i < 3; i++) {
        b_accel[i] = accel[i] - fv4[i];
    }

    for (i = 0; i < 4; i++) {
        qe[i] = 0.0F;
        for (i0 = 0; i0 < 3; i0++) {
            qe[i] += Bk[i + (i0 << 2)] * b_accel[i0];
        }
    }

    qe[3] = 0.0F;
    for (i = 0; i < 4; i++) {
        q[i] += qe[i];
        for (i0 = 0; i0 < 4; i0++) {
            c_Bk[i + (i0 << 2)] = 0.0F;
            for (i1 = 0; i1 < 3; i1++) {
                c_Bk[i + (i0 << 2)] += Bk[i + (i1 << 2)] * Hk[i1 + 3 * i0];
            }
        }

        for (i0 = 0; i0 < 4; i0++) {
            y = 0.0F;
            for (i1 = 0; i1 < 4; i1++) {
                y += c_Bk[i + (i1 << 2)] * P[i1 + (i0 << 2)];
            }

            b_P[i + (i0 << 2)] = P[i + (i0 << 2)] - y;
        }
    }

    for (i = 0; i < 4; i++) {
        for (i0 = 0; i0 < 4; i0++) {
            P[i0 + (i << 2)] = b_P[i0 + (i << 2)];
        }
    }

    if (use_mag == 1) {
        y = norm(mag);
        Hk[0] = 2.0F * q[3];
        Hk[3] = 2.0F * q[2];
        Hk[6] = 2.0F * q[1];
        Hk[9] = 2.0F * q[0];
        Hk[1] = 2.0F * q[0];
        Hk[4] = -2.0F * q[1];
        Hk[7] = -2.0F * q[2];
        Hk[10] = -2.0F * q[3];
        Hk[2] = -2.0F * q[1];
        Hk[5] = -2.0F * q[0];
        Hk[8] = 2.0F * q[3];
        Hk[11] = 2.0F * q[2];
        for (i = 0; i < 3; i++) {
            for (i0 = 0; i0 < 4; i0++) {
                b_Bk[i + 3 * i0] = 0.0F;
                for (i1 = 0; i1 < 4; i1++) {
                    b_Bk[i + 3 * i0] += Hk[i + 3 * i1] * P[i1 + (i0 << 2)];
                }
            }

            for (i0 = 0; i0 < 3; i0++) {
                S[i + 3 * i0] = 0.0F;
                for (i1 = 0; i1 < 4; i1++) {
                    S[i + 3 * i0] += b_Bk[i + 3 * i1] * Hk[i0 + 3 * i1];
                }
            }

            mag[i] /= y;
        }

        for (i = 0; i < 3; i++) {
            S[i + 3 * i] += Cov_info[2];
        }

        for (i = 0; i < 4; i++) {
            for (i0 = 0; i0 < 3; i0++) {
                b_Bk[i + (i0 << 2)] = 0.0F;
                for (i1 = 0; i1 < 4; i1++) {
                    b_Bk[i + (i0 << 2)] += P[i + (i1 << 2)] * Hk[i0 + 3 * i1];
                }
            }
        }

        mrdivide(b_Bk, S, Bk);
        fv4[0] = 2.0F * q[1] * q[2] + 2.0F * q[0] * q[3];
        fv4[1] = ((q[0] * q[0] - q[1] * q[1]) - q[2] * q[2]) - q[3] * q[3];
        fv4[2] = 2.0F * q[2] * q[3] - 2.0F * q[0] * q[1];
        for (i = 0; i < 3; i++) {
            b_accel[i] = mag[i] - fv4[i];
        }

        for (i = 0; i < 4; i++) {
            qe[i] = 0.0F;
            for (i0 = 0; i0 < 3; i0++) {
                qe[i] += Bk[i + (i0 << 2)] * b_accel[i0];
            }
        }

        qe[1] = 0.0F;
        qe[2] = 0.0F;
        for (i = 0; i < 4; i++) {
            q[i] += qe[i];
            for (i0 = 0; i0 < 4; i0++) {
                c_Bk[i + (i0 << 2)] = 0.0F;
                for (i1 = 0; i1 < 3; i1++) {
                    c_Bk[i + (i0 << 2)] += Bk[i + (i1 << 2)] * Hk[i1 + 3 * i0];
                }
            }

            for (i0 = 0; i0 < 4; i0++) {
                y = 0.0F;
                for (i1 = 0; i1 < 4; i1++) {
                    y += c_Bk[i + (i1 << 2)] * P[i1 + (i0 << 2)];
                }

                b_P[i + (i0 << 2)] = P[i + (i0 << 2)] - y;
            }
        }

        for (i = 0; i < 4; i++) {
            for (i0 = 0; i0 < 4; i0++) {
                P[i0 + (i << 2)] = b_P[i0 + (i << 2)];
            }
        }
    }

    y = 0.0F;
    scale = 1.17549435E-38F;
    for (i = 0; i < 4; i++) {
        absxk = fabs(q[i]);
        if (absxk > scale) {
            t = scale / absxk;
            y = 1.0F + y * t * t;
            scale = absxk;
        }
        else {
            t = absxk / scale;
            y += t * t;
        }
    }

    y = scale * sqrt(y);
    for (i = 0; i < 4; i++) {
        q[i] /= y;
    }
}
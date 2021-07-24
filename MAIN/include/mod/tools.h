/**
 * @file tools.h
 */

#ifndef TOOLS_H
#define TOOLS_H

#include <rc/time.h> // for nanos
#include <inttypes.h> // for PRIu64
#include <math.h>
#include <stdio.h> //for fscanf

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

	/**
	 * @brief		finds time elapsed in seconds
	 *
	 * @return		time in seconds
	 */
	double finddt_s(uint64_t ti);

	/**
	 * @brief		scans data from a text file
	 *
	 * @return		0 on success, -1 on failure
	 */
	int scan_file_d(void);


	/**
	 * @brief		this function multiplies two square 3x3 matrices
	 *
	 * @return		0 on success, -1 on failure
	 */
	 //void multiply(double mat1[3][3], double mat2[3][3], double res[3][3])


	 /**
	  * @brief		applies a coordinate tranformation from body to inetrial frame
	  *				using 1-2-3 psi-theta-phi rotation
	  *
	  * @return		0 on success, -1 on failure
	  */
	  //int rotate_i2b(double** vec);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* TOOLS_H */


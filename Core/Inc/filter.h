/*
 * filter.h
 *
 *  Created on: May 4, 2024
 *      Author: siyoums
 */

#ifndef INC_FILTER_H_
#define INC_FILTER_H_
#include <stdint.h>


#define FIR_FILTER_LENGTH 10

typedef struct {
	float coeff[2];
	float out[2];

} LPFilter_t;

typedef struct {
	float buf[FIR_FILTER_LENGTH];
	uint8_t buf_index; // circular buf pointer
	float out;

} FIRFilter_t;

// rc low pass
void lp_filter_init(LPFilter_t *filt, float cutoff_freq, float sample_time_s);
float lp_filter_update(LPFilter_t *filt, float inp);

// FIR
void fir_filter_init(FIRFilter_t *filt);
float fir_filter_update(FIRFilter_t *filt, float inp);



#endif /* INC_FILTER_H_ */

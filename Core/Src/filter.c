/*
 * filter.c
 *
 *  Created on: May 4, 2024
 *      Author: siyoums
 */
#include "filter.h"


void lp_filter_init(LPFilter_t *filt, float cutoff_freq, float sample_time_s) {
	float rc = 1.0f / (6.28318530718f * cutoff_freq);

	filt->coeff[0] = sample_time_s / (sample_time_s + rc);
	filt->coeff[1] = rc / (sample_time_s + rc);

	filt->out[0] = 0.0f;
	filt->out[1] = 0.0f;

}

float lp_filter_update(LPFilter_t *filt, float inp) {
	// shift the new sample to the old one evey time an update happens
	filt->out[1] = filt->out[0];

	// difference equation in discrete time (from backwards euler)
	filt->out[0] = (filt->coeff[0] * inp) + (filt->coeff[1] * filt->out[1]);

	return (filt->out[0]);
}

// fir

float fir_filter_response[]= {-0.0032906f,-0.0052635f,-0.0068811f,0.0000000f,0.0254209f,0.0724719f,0.1311260f,0.1805961f,0.2000000f,0.1805961f,0.1311260f,0.0724719f,0.0254209f,0.0000000f,-0.0068811f,-0.0052635f};

void fir_filter_init(FIRFilter_t *filt) {
	// clear filter buffer
	for (uint8_t i = 0; i < FIR_FILTER_LENGTH; i++) {
		filt->buf[i] = 0.0f;
	}

	filt->buf_index = 0;

	filt->out = 0.0f;
}

float fir_filter_update(FIRFilter_t *filt, float inp) {
	// store last sample in buffer
	filt->buf[filt->buf_index] = inp;

	// incremet buffer index and wrap around when buf index exceeds fir length
	filt->buf_index++;
	if (filt->buf_index > FIR_FILTER_LENGTH) {
		filt->buf_index = 0;
	}

	//compute new output sample (convolution)
	filt->out = 0.0f;

	uint8_t sum_index = filt->buf_index;

	for (uint8_t n = 0; n < FIR_FILTER_LENGTH; n++) {
		//decremet index and wrap
		if (sum_index > 0) {
			sum_index--;
		} else {
			sum_index = FIR_FILTER_LENGTH - 1;
		}

		// multiply impulse response with the shifted input sample and add to output
		filt->out += fir_filter_response[n] * filt->buf[sum_index];


	}

	return filt->out;
}


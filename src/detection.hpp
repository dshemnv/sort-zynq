#ifndef DETECTION_H
#define DETECTION_H

typedef struct
{
	float top_right;
	float bottom_left;
	const char *label;
	float probability;
} detectionprops;

#endif
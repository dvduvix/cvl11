// camshift_wrapper.h - by Robin Hewitt, 2007
// http://www.cognotics.com/opencv/downloads/camshift_wrapper
// This is free software. See License.txt, in the download
// package, for details.
//
//
// Public interface for the Simple Camshift Wrapper

#ifndef __SIMPLE_CAMSHIFT_WRAPPER_H
#define __SIMPLE_CAMSHIFT_WRAPPER_H


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <glib-2.0/glib/gmacros.h>

// Main Control functions
int     createTracker(const IplImage * pImg);
void    releaseTracker();
void    startTracking(IplImage * pImg, CvRect * pRect);
CvBox2D track(IplImage *);


// Parameter settings
void setVmin(int vmin);
void setSmin(int smin);

#endif

// facedet.h - by Robin Hewitt, 2007
// http://www.cognotics.com/opencv/downloads/camshift_wrapper
// This is free software. See License.txt, in the download
// package, for details.
//
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/objdetect/objdetect.hpp>

// Public interface for face detection
int      initFaceDet(const char * haarCascadePath);
void     closeFaceDet();
CvRect * detectFace(IplImage * pImg);

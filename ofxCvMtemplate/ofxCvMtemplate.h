/*
* ofxCvMtemplate.h
*
/// Carles Gutierrez
/// 2 Marzo 2010
/// Useed findMTemplate to get % of success result between two images with the same size and same ROI images
*
*/

#ifndef OFX_CV_MATCH_TEMPLATE
#define OFX_CV_MATCH_TEMPLATE

#include "ofxCvConstants.h"
#include "ofxCvBlob.h"
#include "ofxCvGrayscaleImage.h"
#include "ofxCvColorImage.h"
#include <algorithm>

class ofxCvMtemplate : public ofBaseDraws {

  public:

    ofxCvMtemplate();
    virtual  ~ofxCvMtemplate();

	virtual float getWidth() { return input_width; };    //set after first findContours call
	virtual float getHeight() { return input_height; };  //set after first findContours call

    virtual int  findMTemplate( ofxCvGrayscaleImage& input,
                               ofxCvGrayscaleImage& tp1);
                               // approximation = don't do points for all points
                               // of the contour, if the contour runs
                               // along a straight line, for example...
    virtual void  draw() { draw(0,0, input_width, input_height); };
    virtual void  draw( float x, float y ) { draw(x,y, input_width, input_height); };
    virtual void  draw( float x, float y, float w, float h );


  protected:

    int  input_width;
    int  input_height;

    int  template_width;
    int  template_height;

    int  res_width;
    int  res_height;

	CvPoint		minloc, maxloc;
	double		minval, maxval;

    ofxCvColorImage         resMtemplate;
    ofxCvGrayscaleImage     inputCopy;
    ofxCvGrayscaleImage     templateCopy;

    virtual void reset();

};



#endif

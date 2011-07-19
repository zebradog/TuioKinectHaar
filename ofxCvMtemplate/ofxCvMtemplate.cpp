/// Carles Gutierrez
/// 2 Marzo 2010
/// Use MatchTemplate to get % of success result between two images with the same size and same ROI images


#include "ofxCvMtemplate.h"

//--------------------------------------------------------------------------------
ofxCvMtemplate::ofxCvMtemplate() {
    input_width = 0;
    input_height = 0;

    res_width = 0;
    res_height = 0;

    template_width = 0;
    template_height = 0;

	reset();
}

//--------------------------------------------------------------------------------
ofxCvMtemplate::~ofxCvMtemplate() {

}

//--------------------------------------------------------------------------------
void ofxCvMtemplate::reset() {

}

//--------------------------------------------------------------------------------
int ofxCvMtemplate::findMTemplate( ofxCvGrayscaleImage&  input,
                                      ofxCvGrayscaleImage&  OF_imgTemplate) {
    //Calculate New Result Dimensions
	res_width  = input.getWidth() - OF_imgTemplate.getWidth() + 1;
	res_height = input.getWidth() - OF_imgTemplate.getWidth() + 1;

    /// allocate the res image
    IplImage	*res;
	/* create new image for template matching computation */
	res = cvCreateImage( cvSize( res_width, res_height ), IPL_DEPTH_32F, 1 );


    //////////////////////////////////
    // Could be fine optimize and use the others methods
    // Just for testing I've used the first method -> CV_TM_SQDIFF_NORMED
    // and printed to the console the matching results

    /* choose template matching method to be used */
	//cvMatchTemplate( inputCopy.getCvImage(), templateCopy.getCvImage(), resMtemplate.getCvImage(), CV_TM_SQDIFF );
	/*cvMatchTemplate( img, tpl, res, CV_TM_SQDIFF_NORMED );
	cvMatchTemplate( img, tpl, res, CV_TM_CCORR );
	cvMatchTemplate( img, tpl, res, CV_TM_CCORR_NORMED );
	cvMatchTemplate( img, tpl, res, CV_TM_CCOEFF );
	cvMatchTemplate( img, tpl, res, CV_TM_CCOEFF_NORMED );*/

    bool MTverbose = true;
    if(MTverbose)
    {
        printf("OF_imgTemplate ROI height = %f\n", OF_imgTemplate.getROI().height);
        printf("OF_imgTemplate ROI width = %f\n", OF_imgTemplate.getROI().width);
        //input.getROI()
        printf("input.getROI()  height = %f\n", input.getROI().height);
        printf("input.getROI()  width = %f\n", input.getROI().width);
    }

    if(OF_imgTemplate.getROI().height == input.getROI().height && OF_imgTemplate.getROI().width == input.getROI().width)
    {

        cvMatchTemplate( input.getCvImage(), OF_imgTemplate.getCvImage(), res, CV_TM_CCORR_NORMED ); //CV_TM_CCORR_NORMED

        if(MTverbose)
        {
            printf("cvMatchTemplate returned\n");

            printf("cnumber of channels of iplimage: %d\n", res->nChannels);
            printf("depth of iplimage: %d\n", res->depth);

            printf("cvMinMaxLoc\n");
        }

        cvMinMaxLoc( res, &minval, &maxval, &minloc, &maxloc, 0 );

        if(MTverbose)
        {
            printf("minval=[%e]\n",minval);
            printf("maxval=[%e]\n",maxval);

            printf("minlocX=[%i]\n",minloc.x);
            printf("minlocY=[%i]\n",minloc.y);

            printf("maxlocX=[%i]\n",maxloc.x);
            printf("maxlocY=[%i]\n",maxloc.y);
        }

    }

    ///FREE OPENCV MEMORY
    cvReleaseImage( & res);

}


//--------------------------------------------------------------------------------
void ofxCvMtemplate::draw( float x, float y, float w, float h ) {

}


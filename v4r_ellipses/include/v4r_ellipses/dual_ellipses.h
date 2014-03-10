
bool DualEllipse(double dDualEllipseMinGradient,
                                         int iDualEllipseBorder,
                                         IplImage *pSobelDx, 
                                         IplImage *pSobelDy,
                                         std::vector<cv::Point2f> &pEllPoints, 
                                         CzEllipse *pEllipse)
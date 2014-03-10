
////////////////////////////////////////////////////////////////////////////////
// Calculate exact ellipse parameters of the given ellipse hypothesis using gradient images
bool DualEllipse(double dDualEllipseMinGradient,  int iDualEllipseBorder,  IplImage *pSobelDx,  IplImage *pSobelDy, std::vector<cv::Point2f> &contour,  CzEllipse *pEllipse)
{
	// get region around the ellipse edgels where to compute the dual ellipse
	// that means, pixels where the gradient is higher than the given threshold and the distance
	// to the ellipse borders along the line joining the pixel and the ellipse center
	// is less than a certain radius (3px for example)

	double minx=10000, maxx=0, miny=10000, maxy=0;
	double minGradPoints=10000,maxGradPoints=0;
	for(unsigned i=0; i<contour.size();i++) {
		if(contour[i].x < minx) minx=contour[i].x;
		if(contour[i].y < miny) miny=contour[i].y;
		if(contour[i].x > maxx) maxx=contour[i].x;
		if(contour[i].y > maxy) maxy=contour[i].y;
			
		float grad = (float)(sqrt(Sqr((float)GetPx16SC1(pSobelDx, (short)contour[i].x,(short)contour[i].y)) + Sqr((float)GetPx16SC1(pSobelDy, (short)contour[i].x,(short)contour[i].y))));
		if(minGradPoints > grad) minGradPoints = grad;
		if(grad > maxGradPoints) maxGradPoints = grad; 
	}

	double gradThreshold = (minGradPoints + (maxGradPoints - minGradPoints)/2)/2;
	gradThreshold=dDualEllipseMinGradient;// minimum gradient to use a pixel
   // it does not make sense to use different values for ellipseRand and rand
	int ellipseRand=iDualEllipseBorder; // maximum distance to given hypothesis
	int rand=iDualEllipseBorder; // window border size to search for border points (hypothesis parameter A+rand X B+rand)
	CzArray<CzVector2> pointsToUse;
	double sumx=0,sumy=0;
	// searching within the window for points to use
	short min_y = (short)(miny-rand);
	short max_y = (short)(maxy+rand);
	short min_x = (short)(minx-rand);
	short max_x = (short)(maxx+rand);
	if (min_y < 0) min_y = 0;
	if (max_y >= pSobelDx->height) max_y = (short)pSobelDx->height-1;
	if (min_x < 0) min_x = 0;
	if (max_x >= pSobelDx->width) max_x = (short)pSobelDx->width-1;
	for(short j=min_y; j < max_y; j++) { 
		for(short i=min_x; i < max_x; i++) {
			float grad = (float)(sqrt(Sqr((float)GetPx16SC1(pSobelDx, i,j)) + Sqr((float)GetPx16SC1(pSobelDy, i,j))));
         if(gradThreshold < grad &&	pEllipse->InsideEllipse(pEllipse->dA+ellipseRand,pEllipse->dB+ellipseRand,pEllipse->dX,pEllipse->dY, pEllipse->dPhi, i, j) && 
                                                           !pEllipse->InsideEllipse(pEllipse->dA-ellipseRand,pEllipse->dB-ellipseRand,pEllipse->dX,pEllipse->dY, pEllipse->dPhi, i, j))	{
				pointsToUse.PushBack(CzVector2(i,j));
				sumx+=i;
				sumy+=j;
			} 
		}
	}

	// now the pointsToUse array exists (points with a distance smaller than ellipseRand from the hypothesis)

	// normalisation (to ensure numeric stability)

	double mx=sumx/pointsToUse.Size();
	double my=sumy/pointsToUse.Size();

	// compute scaling factors

	double lengths=0;
	for(unsigned i=0; i<pointsToUse.Size();i++) {
		lengths+= sqrt(Sqr(pointsToUse[i].x - mx) + Sqr(pointsToUse[i].y - my));
	}

	double ss=sqrt(2.) / (lengths/pointsToUse.Size());
	double Hdouble[] = {	ss,  0,  -ss*mx,
	             					0,  ss,  -ss*my,
	             					0, 0, 1};		

	CvMat H = cvMat(3, 3, CV_64FC1, Hdouble);
	CvMat *HT = cvCreateMat(3, 3, CV_64FC1);
	CvMat *Hinv = cvCreateMat(3, 3, CV_64FC1);
	CvMat *HinvT = cvCreateMat(3, 3, CV_64FC1);
   if ((HT == 0) || (Hinv == 0) || (HinvT == 0))
      throw CzExcept(__HERE__, "Could not create instances of CvMat! Out of memory?");

	cvInvert(&H, Hinv, CV_LU);
	cvTranspose(&H, HT);
	cvTranspose(Hinv,HinvT);

	// transformation finished
	/*
	
	Establish the least-square-system.

	In the following, [] means the definition of a vector or matrix and [x]^T its
	the transpose of [x] 

	A point xi = [ui,vi,1]^T (in homogenous coordinates) lies on the conic C iff
	it satisfies xi^T*C*xi=0
	C is:
	A 		B/2		D/2
	B/2	 	C	  	E/2
	D/2		E/2		 F
	The ConicParameters = [A,B,C,D,E,F] of the conic C can be obtained by least square techniques
	by minimizing f(ConicParameters of C):

	f(ConicParameters of C) = sum_{iterate points}(w_i*(x_i^T*C*x_i)^2)	

	Following part is based in the duality relationship of projective geometry where the role of
	homogenous points and lines can be interchanged.
	
	In the dual space, a line l_i = [a_i,b_i,c_i]^T is tangent to the dual of the conic C*
	iff it satisfies l_i^T*(C*)*l_i=0 where C* = inv(C)
	As in the point space, the DualConicParameters = [A*,B*,C*,D*,E*,F*] of the dual conic C*
	can be obtained similarly by finding f where f(DualConicParameters of C) reaches a minimum:

	f(DualConicParameters of C) = sum_{iterate lines}(w_i*(l_i^T*(C*)*l_i)^2) (Eq 1)

	li = [a_i,b_i,c_i]^T are obtained directly from the image gradient as follows:
	a_i = dx_i
	b_i = dy_i
	c_i = -[dx_i,dy_i]^T*[x_i,y_i]

	when the image gradient at [x_i,y_i] is not null, it defines the normal orientation of a line
	passing through [x_i,y_i]

	Operating in Eq 1, we obtain the following normal equations:
	[sum{iterate lines}(sqr(w_i)*Ki*Ki^T)][DualConicParameters] = 0

	where Ki = [sqr(a_i),a_i*b_i, sqr(b_i), a_i*c_i, b_i*c_i, c_i*2]^T

	By using the constraint F* = 1 to solve the system and adding the ellipse discriminant 4*A*C - sqr(B) = 1,
	we obtain the final system to find DualConicParameters' = [A'*,B'*,C'*,D'*,E'*,F'*]:

	sum{iterate lines}(-sqr(w_i)*K'i*sqr(c_i)) = 0

	which is the final system solved in the following code.
	Further references to the method can be found in:
	http://vision.gel.ulaval.ca/~hebert/pdf/ouellet07Ellipses.pdf

	*/

	CvMat *M = cvCreateMat(5, 5, CV_64FC1);
	CvMat *E = cvCreateMat(5, 1, CV_64FC1);
   if ((E == 0) || (M == 0))
      throw CzExcept(__HERE__, "Could not create instances of CvMat! Out of memory?");
	cvZero(E);
	cvZero(M);

	CvMat *Kprima = cvCreateMat(5, 1, CV_64FC1);
	CvMat *res1 = cvCreateMat(5, 5, CV_64FC1);
	CvMat *KprimaTrans = cvCreateMat(1, 5, CV_64FC1);
   if ((Kprima == 0) || (res1 == 0) || (KprimaTrans == 0))
      throw CzExcept(__HERE__, "Could not create instances of CvMat! Out of memory?");

   CzArray<double[3]> tangentLines((unsigned)pointsToUse.Size());
/*   double **tangentLines = new double*[pointsToUse.Size()];
   for (unsigned i=0; i<pointsToUse.Size(); i++)
      tangentLines[i] = new double[3];*/


   for (unsigned i=0; i<pointsToUse.Size();i++) {
      cvZero(Kprima);
      cvZero(res1);
      cvZero(KprimaTrans);
		
      float grad = (float)(sqrt(Sqr((float)GetPx16SC1(pSobelDx, (short)pointsToUse[i].x,(short)pointsToUse[i].y)) + 
                                Sqr((float)GetPx16SC1(pSobelDy, (short)pointsToUse[i].x,(short)pointsToUse[i].y))));

		// compute tangent lines to the dual conic directly from the image gradient
		double LiD[] = {(double)GetPx16SC1(pSobelDx, (short)pointsToUse[i].x,(short)pointsToUse[i].y),
										(double)GetPx16SC1(pSobelDy, (short)pointsToUse[i].x,(short)pointsToUse[i].y),
									  -((double)GetPx16SC1(pSobelDx, (short)pointsToUse[i].x,(short)pointsToUse[i].y)*(pointsToUse[i].x) + 
										(double)GetPx16SC1(pSobelDy, (short)pointsToUse[i].x,(short)pointsToUse[i].y)*(pointsToUse[i].y))};
		CvMat li = cvMat(3, 1, CV_64FC1, LiD);
		// scale line such norm(a_i,b_i)=1	
		li.data.db[0] /= grad;
		li.data.db[1] /= grad;
		li.data.db[2] /= grad;

		// normalize lines like L'=H^-T * L (-T is the inverse transposed)
		cvMatMul(HinvT,&li,&li);

		//save tangent line
		tangentLines[i][0] = li.data.db[0];
		tangentLines[i][1] = li.data.db[1];
		tangentLines[i][2] = li.data.db[2];

      double weight = Sqr(grad);

		Kprima->data.db[0] = Sqr(li.data.db[0]);
		Kprima->data.db[1] = li.data.db[0] * li.data.db[1];
		Kprima->data.db[2] = Sqr(li.data.db[1]);
		Kprima->data.db[3] = li.data.db[0] * li.data.db[2];
		Kprima->data.db[4] = li.data.db[1] * li.data.db[2];

		cvTranspose(Kprima,KprimaTrans);
		
		cvGEMM(Kprima, KprimaTrans, weight, NULL, 1, res1, 0 );
		cvAdd(M,res1,M);

		Kprima->data.db[0] *= -weight*Sqr(li.data.db[2]);
		Kprima->data.db[1] *= -weight*Sqr(li.data.db[2]);
		Kprima->data.db[2] *= -weight*Sqr(li.data.db[2]);
		Kprima->data.db[3] *= -weight*Sqr(li.data.db[2]);
		Kprima->data.db[4] *= -weight*Sqr(li.data.db[2]);

		cvAdd(E, Kprima, E);
	}

	// solve the least square system using the pseudo inverse technique
	// params = inv(M^T*M)*M^T*E

	CvMat *MT = cvCreateMat(5, 5, CV_64FC1);
   if (MT == 0)
      throw CzExcept(__HERE__, "Could not create instance of CvMat! Out of memory?");
	cvTranspose(M, MT);

	CvMat *MPerMT = cvCreateMat(5, 5, CV_64FC1);
	CvMat *MPerMTinv = cvCreateMat(5, 5, CV_64FC1);
   if ((MPerMT == 0) || (MPerMTinv == 0))
      throw CzExcept(__HERE__, "Could not create instance of CvMat! Out of memory?");
	
	cvMatMul(MT,M,MPerMT);
	cvInvert(MPerMT,MPerMTinv);
	cvMatMul(MPerMTinv,MT,MPerMT);
	
	CvMat *params = cvCreateMat(5, 1, CV_64FC1);
   if (params == 0)
      throw CzExcept(__HERE__, "Could not create instance of CvMat! Out of memory?");
	cvMatMul(MPerMT,E,params);

	double CNorm[] = {params->data.db[0],    params->data.db[1]/2., params->data.db[3]/2., 
                     params->data.db[1]/2., params->data.db[2],    params->data.db[4]/2.,
                     params->data.db[3]/2., params->data.db[4]/2., 1};

	//unnormalize params
	CvMat CNormMat = cvMat(3,3,CV_64FC1,CNorm);
	CvMat *CNormMatMalHInv = cvCreateMat(3,3,CV_64FC1);
   if (CNormMatMalHInv == 0)
      throw CzExcept(__HERE__, "Could not create instance of CvMat! Out of memory?");
	cvMatMul(Hinv, &CNormMat, CNormMatMalHInv);
	cvMatMul(CNormMatMalHInv, HinvT, CNormMatMalHInv);

	//compute error in the dual space
	//the error is proportional to distance between a tangent li and its pole xi=(C*)*li

	double dSumDistance=0;
	CvMat line;
	CvMat *pole = cvCreateMat(3,1,CV_64FC1);
   if (pole == 0)
      throw CzExcept(__HERE__, "Could not create instance of CvMat! Out of memory?");
	
	for (unsigned i=0; i<pointsToUse.Size(); i++) {
		//distance between li and pole
		line = cvMat(3,1,CV_64FC1,(tangentLines)[i]);
		cvMatMul(&CNormMat, &line, pole);

		double x0 = pole->data.db[0]/pole->data.db[2];
		double y0 = pole->data.db[1]/pole->data.db[2];

		double d = abs(tangentLines[i][0] * x0 + tangentLines[i][1] * y0 + tangentLines[i][2]) / sqrt(Sqr(tangentLines[i][0]) + Sqr(tangentLines[i][1]));
		dSumDistance += d;
	}

	cvReleaseMat(&pole);

//	pEllipse->dFitError = dSumDistance / pointsToUse.Size();

/*   for (unsigned i=0; i<pointsToUse.Size(); i++)
      delete tangentLines[i];
   delete tangentLines;*/

	// by inverting the dual conic matrix C* we obtain the original
	// conic parameters we were seeking for

	CvMat *CMat = cvCreateMat(3,3,CV_64FC1);
   if (CMat == 0)
      throw CzExcept(__HERE__, "Could not create instance of CvMat! Out of memory?");
	cvInvert(CNormMatMalHInv, CMat);

	double Ac,Bc,Cc,Dc,Ec,Fc;
	Ac=CMat->data.db[0];
	Bc=CMat->data.db[1];
	Cc=CMat->data.db[4];
	Dc=CMat->data.db[6];
	Ec=CMat->data.db[5];
	Fc=CMat->data.db[8];

   // calculate ellipse parameters x/y/A/B/phi from conic equation Ax^2+Bxy+Cy^2....+F=0
//	bool ell_ok = pEllipse->computeAndSetGeomFromConic(Ac,Bc,Cc,Dc,Ec,Fc);
//	cout << "x:" << Dp/2. << " y:" << Ep/2. << " " << pEllipse->dX << " " << pEllipse->dY << " " << pEllipse->dA << " " << pEllipse->dB << " " << pEllipse->dPhi << endl;
//	cout << "x:" << pEllipse->dX << " y:" << pEllipse->dY << " supp:" << pEllipse->dSupport << " fit_err:" << pEllipse->dFitError << " phi:" << pEllipse->dPhi << endl;

	cvReleaseMat(&CNormMatMalHInv);
	cvReleaseMat(&CMat);
	cvReleaseMat(&params);
	cvReleaseMat(&MPerMT);
	cvReleaseMat(&MPerMTinv);
	cvReleaseMat(&MT);
	cvReleaseMat(&res1);
	cvReleaseMat(&Kprima);
	cvReleaseMat(&KprimaTrans);
	cvReleaseMat(&M);
	cvReleaseMat(&E);
	cvReleaseMat(&HT);
	cvReleaseMat(&Hinv);
	cvReleaseMat(&HinvT);

	return ell_ok;
}

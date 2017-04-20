#include "surf.hpp"

static const int   SURF_ORI_SEARCH_INC = 5;
static const float SURF_ORI_SIGMA      = 2.5f;
static const float SURF_DESC_SIGMA     = 3.3f;

static int pointCount;

// Wavelet size at first layer of first octave.
static const int SURF_HAAR_SIZE0 = 9;

// Wavelet size increment between layers. This should be an even number,
// such that the wavelet sizes in an octave are either all even or all odd.
// This ensures that when looking for the neighbours of a sample, the layers
// above and below are aligned correctly.
static const int SURF_HAAR_SIZE_INC = 6;

/*
 * Calculate the determinant and trace of the Hessian for a layer of the
 * scale-space pyramid
 */

struct SurfHF
{
    int p0, p1, p2, p3;
    float w;
};

inline float calcHaarPattern( const int* origin, const SurfHF* f, int n )
{
    double d = 0;
    for( int k = 0; k < n; k++ )
        d += (origin[f[k].p0] + origin[f[k].p3] - origin[f[k].p1] - origin[f[k].p2])*f[k].w;
    return (float)d;
}

static void
resizeHaarPattern( const int src[][5], SurfHF* dst, int n, int oldSize, int newSize, int widthStep )
{
    float ratio = (float)newSize/oldSize;
    for( int k = 0; k < n; k++ )
    {
        int dx1 = cvRound( ratio*src[k][0] );
        int dy1 = cvRound( ratio*src[k][1] );
        int dx2 = cvRound( ratio*src[k][2] );
        int dy2 = cvRound( ratio*src[k][3] );
        dst[k].p0 = dy1*widthStep + dx1;
        dst[k].p1 = dy2*widthStep + dx1;
        dst[k].p2 = dy1*widthStep + dx2;
        dst[k].p3 = dy2*widthStep + dx2;
        dst[k].w = src[k][4]/((float)(dx2-dx1)*(dy2-dy1));
    }
}

static void calcLayerDetAndTrace( const Mat& sum, int size, int sampleStep,
                                  Mat& det, Mat& trace )
{
    const int NX=3, NY=3, NXY=4;
    const int dx_s[NX][5] = { {0, 2, 3, 7, 1}, {3, 2, 6, 7, -2}, {6, 2, 9, 7, 1} };
    const int dy_s[NY][5] = { {2, 0, 7, 3, 1}, {2, 3, 7, 6, -2}, {2, 6, 7, 9, 1} };
    const int dxy_s[NXY][5] = { {1, 1, 4, 4, 1}, {5, 1, 8, 4, -1}, {1, 5, 4, 8, -1}, {5, 5, 8, 8, 1} };

    SurfHF Dx[NX], Dy[NY], Dxy[NXY];

    if( size > sum.rows-1 || size > sum.cols-1 )
       return;

    resizeHaarPattern( dx_s , Dx , NX , 9, size, sum.cols );
    resizeHaarPattern( dy_s , Dy , NY , 9, size, sum.cols );
    resizeHaarPattern( dxy_s, Dxy, NXY, 9, size, sum.cols );

    /* The integral image 'sum' is one pixel bigger than the source image */
    int samples_i = 1+(sum.rows-1-size)/sampleStep;
    int samples_j = 1+(sum.cols-1-size)/sampleStep;

    /* Ignore pixels where some of the kernel is outside the image */
    int margin = (size/2)/sampleStep;

    for( int i = 0; i < samples_i; i++ )
    {
        const int* sum_ptr = sum.ptr<int>(i*sampleStep);
        float* det_ptr = &det.at<float>(i+margin, margin);
        float* trace_ptr = &trace.at<float>(i+margin, margin);
        for( int j = 0; j < samples_j; j++ )
        {
            float dx  = calcHaarPattern( sum_ptr, Dx , 3 );
            float dy  = calcHaarPattern( sum_ptr, Dy , 3 );
            float dxy = calcHaarPattern( sum_ptr, Dxy, 4 );
            sum_ptr += sampleStep;
            det_ptr[j] = dx*dy - 0.81f*dxy*dxy;
            trace_ptr[j] = dx + dy;
        }
    }
}

// Multi-threaded construction of the scale-space pyramid
struct SURFBuildInvoker
{
    SURFBuildInvoker( const Mat& _sum, const vector<int>& _sizes,
                      const vector<int>& _sampleSteps,
                      vector<Mat>& _dets, vector<Mat>& _traces )
    {
        sum = &_sum;
        sizes = &_sizes;
        sampleSteps = &_sampleSteps;
        dets = &_dets;
        traces = &_traces;
    }

    void operator()(const BlockedRange& range) const
    {
        for( int i=range.begin(); i<range.end(); i++ )
            calcLayerDetAndTrace( *sum, (*sizes)[i], (*sampleSteps)[i], (*dets)[i], (*traces)[i] );
    }

    const Mat *sum;
    const vector<int> *sizes;
    const vector<int> *sampleSteps;
    vector<Mat>* dets;
    vector<Mat>* traces;
};

/*
 * Maxima location interpolation as described in "Invariant Features from
 * Interest Point Groups" by Matthew Brown and David Lowe. This is performed by
 * fitting a 3D quadratic to a set of neighbouring samples.
 *
 * The gradient vector and Hessian matrix at the initial keypoint location are
 * approximated using central differences. The linear system Ax = b is then
 * solved, where A is the Hessian, b is the negative gradient, and x is the
 * offset of the interpolated maxima coordinates from the initial estimate.
 * This is equivalent to an iteration of Netwon's optimisation algorithm.
 *
 * N9 contains the samples in the 3x3x3 neighbourhood of the maxima
 * dx is the sampling step in x
 * dy is the sampling step in y
 * ds is the sampling step in size
 * point contains the keypoint coordinates and scale to be modified
 *
 * Return value is 1 if interpolation was successful, 0 on failure.
 */
static int
interpolateKeypoint( float N9[3][9], int dx, int dy, int ds, KeyPoint& kpt )
{
    Matx31f b(-(N9[1][5]-N9[1][3])/2,  // Negative 1st deriv with respect to x
              -(N9[1][7]-N9[1][1])/2,  // Negative 1st deriv with respect to y
              -(N9[2][4]-N9[0][4])/2); // Negative 1st deriv with respect to s

    Matx33f A(
        N9[1][3]-2*N9[1][4]+N9[1][5],            // 2nd deriv x, x
        (N9[1][8]-N9[1][6]-N9[1][2]+N9[1][0])/4, // 2nd deriv x, y
        (N9[2][5]-N9[2][3]-N9[0][5]+N9[0][3])/4, // 2nd deriv x, s
        (N9[1][8]-N9[1][6]-N9[1][2]+N9[1][0])/4, // 2nd deriv x, y
        N9[1][1]-2*N9[1][4]+N9[1][7],            // 2nd deriv y, y
        (N9[2][7]-N9[2][1]-N9[0][7]+N9[0][1])/4, // 2nd deriv y, s
        (N9[2][5]-N9[2][3]-N9[0][5]+N9[0][3])/4, // 2nd deriv x, s
        (N9[2][7]-N9[2][1]-N9[0][7]+N9[0][1])/4, // 2nd deriv y, s
        N9[0][4]-2*N9[1][4]+N9[2][4]);           // 2nd deriv s, s

    Matx31f x = A.solve<1>(b, DECOMP_LU);

    bool ok = (x(0,0) != 0 || x(1,0) != 0 || x(2,0) != 0) &&
        std::abs(x(0,0)) <= 1 && std::abs(x(1,0)) <= 1 && std::abs(x(2,0)) <= 1;

    if( ok )
    {
        kpt.pt.x += x(0,0)*dx;
        kpt.pt.y += x(1,0)*dy;
        kpt.size = (float)cvRound( kpt.size + x(2,0)*ds );
    }
    return ok;
}

/*
 * Find the maxima in the determinant of the Hessian in a layer of the
 * scale-space pyramid
 */
static void
findMaximaInLayer( const Mat& sum, const Mat& mask_sum,
                   const vector<Mat>& dets, const vector<Mat>& traces,
                   const vector<int>& sizes, vector<KeyPoint>& keypoints,
                   int octave, int layer, float hessianThreshold, int sampleStep )
{
	if(keypoints.size() >= pointCount){
		return;
	}

    // Wavelet Data
    const int NM=1;
    const int dm[NM][5] = { {0, 0, 9, 9, 1} };
    SurfHF Dm;

    int size = sizes[layer];

    // The integral image 'sum' is one pixel bigger than the source image
    int layer_rows = (sum.rows-1)/sampleStep;
    int layer_cols = (sum.cols-1)/sampleStep;

    // Ignore pixels without a 3x3x3 neighbourhood in the layer above
    int margin = (sizes[layer+1]/2)/sampleStep+1;

    if( !mask_sum.empty() )
       resizeHaarPattern( dm, &Dm, NM, 9, size, mask_sum.cols );

    int step = (int)(dets[layer].step/dets[layer].elemSize());

    for( int i = margin; i < layer_rows - margin; i++ )
    {
        const float* det_ptr = dets[layer].ptr<float>(i);
        const float* trace_ptr = traces[layer].ptr<float>(i);
        for( int j = margin; j < layer_cols-margin; j++ )
        {
            float val0 = det_ptr[j];
            if( val0 > hessianThreshold )
            {
                /* Coordinates for the start of the wavelet in the sum image. There
                   is some integer division involved, so don't try to simplify this
                   (cancel out sampleStep) without checking the result is the same */
                int sum_i = sampleStep*(i-(size/2)/sampleStep);
                int sum_j = sampleStep*(j-(size/2)/sampleStep);

                /* The 3x3x3 neighbouring samples around the maxima.
                   The maxima is included at N9[1][4] */

                const float *det1 = &dets[layer-1].at<float>(i, j);
                const float *det2 = &dets[layer].at<float>(i, j);
                const float *det3 = &dets[layer+1].at<float>(i, j);
                float N9[3][9] = { { det1[-step-1], det1[-step], det1[-step+1],
                                     det1[-1]  , det1[0] , det1[1],
                                     det1[step-1] , det1[step] , det1[step+1]  },
                                   { det2[-step-1], det2[-step], det2[-step+1],
                                     det2[-1]  , det2[0] , det2[1],
                                     det2[step-1] , det2[step] , det2[step+1]  },
                                   { det3[-step-1], det3[-step], det3[-step+1],
                                     det3[-1]  , det3[0] , det3[1],
                                     det3[step-1] , det3[step] , det3[step+1]  } };

                /* Check the mask - why not just check the mask at the center of the wavelet? */
                if( !mask_sum.empty() )
                {
                    const int* mask_ptr = &mask_sum.at<int>(sum_i, sum_j);
                    float mval = calcHaarPattern( mask_ptr, &Dm, 1 );
                    if( mval < 0.5 )
                        continue;
                }

                /* Non-maxima suppression. val0 is at N9[1][4]*/
                if( val0 > N9[0][0] && val0 > N9[0][1] && val0 > N9[0][2] &&
                    val0 > N9[0][3] && val0 > N9[0][4] && val0 > N9[0][5] &&
                    val0 > N9[0][6] && val0 > N9[0][7] && val0 > N9[0][8] &&
                    val0 > N9[1][0] && val0 > N9[1][1] && val0 > N9[1][2] &&
                    val0 > N9[1][3]                    && val0 > N9[1][5] &&
                    val0 > N9[1][6] && val0 > N9[1][7] && val0 > N9[1][8] &&
                    val0 > N9[2][0] && val0 > N9[2][1] && val0 > N9[2][2] &&
                    val0 > N9[2][3] && val0 > N9[2][4] && val0 > N9[2][5] &&
                    val0 > N9[2][6] && val0 > N9[2][7] && val0 > N9[2][8] )
                {
                    /* Calculate the wavelet center coordinates for the maxima */
                    float center_i = sum_i + (size-1)*0.5f;
                    float center_j = sum_j + (size-1)*0.5f;

                    KeyPoint kpt( center_j, center_i, (float)sizes[layer],
                                  -1, val0, octave, CV_SIGN(trace_ptr[j]) );

                    /* Interpolate maxima location within the 3x3x3 neighbourhood  */
                    int ds = size - sizes[layer-1];
                    int interp_ok = interpolateKeypoint( N9, sampleStep, sampleStep, ds, kpt );

                    /* Sometimes the interpolation step gives a negative size etc. */
                    if( interp_ok  )
                    {
                        /*printf( "KeyPoint %f %f %d\n", point.pt.x, point.pt.y, point.size );*/
                    #ifdef HAVE_TBB
                        static tbb::mutex m;
                        tbb::mutex::scoped_lock lock(m);
                    #endif


                        if(keypoints.size() < pointCount){
                        	keypoints.push_back(kpt);
                        }else{
                        	return;
                        }

                    }
                }
            }
        }
    }
}

// Multi-threaded search of the scale-space pyramid for keypoints
struct SURFFindInvoker
{
    SURFFindInvoker( const Mat& _sum, const Mat& _mask_sum,
                     const vector<Mat>& _dets, const vector<Mat>& _traces,
                     const vector<int>& _sizes, const vector<int>& _sampleSteps,
                     const vector<int>& _middleIndices, vector<KeyPoint>& _keypoints,
                     int _nOctaveLayers, float _hessianThreshold )
    {
        sum = &_sum;
        mask_sum = &_mask_sum;
        dets = &_dets;
        traces = &_traces;
        sizes = &_sizes;
        sampleSteps = &_sampleSteps;
        middleIndices = &_middleIndices;
        keypoints = &_keypoints;
        nOctaveLayers = _nOctaveLayers;
        hessianThreshold = _hessianThreshold;
    }

    void operator()(const BlockedRange& range) const
    {
        for( int i=range.begin(); i<range.end(); i++ )
        {
            int layer = (*middleIndices)[i];
            int octave = i % nOctaveLayers;
            findMaximaInLayer( *sum, *mask_sum, *dets, *traces, *sizes,
                               *keypoints, octave, layer, hessianThreshold,
                               (*sampleSteps)[layer] );

            if((*keypoints).size() >= pointCount){
            	return;
            }
        }
    }

    const Mat *sum;
    const Mat *mask_sum;
    const vector<Mat>* dets;
    const vector<Mat>* traces;
    const vector<int>* sizes;
    const vector<int>* sampleSteps;
    const vector<int>* middleIndices;
    vector<KeyPoint>* keypoints;
    int nOctaveLayers;
    float hessianThreshold;
};


static void fastHessianDetector( const Mat& sum, const Mat& mask_sum, vector<KeyPoint>& keypoints,
                                 int nOctaves, int nOctaveLayers, float hessianThreshold )
{
    /* Sampling step along image x and y axes at first octave. This is doubled
       for each additional octave. WARNING: Increasing this improves speed,
       however keypoint extraction becomes unreliable. */
    const int SAMPLE_STEP0 = 1;

    int nTotalLayers = (nOctaveLayers+2)*nOctaves;
    int nMiddleLayers = nOctaveLayers*nOctaves;

    vector<Mat> dets(nTotalLayers);
    vector<Mat> traces(nTotalLayers);
    vector<int> sizes(nTotalLayers);
    vector<int> sampleSteps(nTotalLayers);
    vector<int> middleIndices(nMiddleLayers);

    // Allocate space and calculate properties of each layer
    int index = 0, middleIndex = 0, step = SAMPLE_STEP0;

    for( int octave = 0; octave < nOctaves; octave++ )
    {
        for( int layer = 0; layer < nOctaveLayers+2; layer++ )
        {
            /* The integral image sum is one pixel bigger than the source image*/
            dets[index].create( (sum.rows-1)/step, (sum.cols-1)/step, CV_32F );
            traces[index].create( (sum.rows-1)/step, (sum.cols-1)/step, CV_32F );
            sizes[index] = (SURF_HAAR_SIZE0 + SURF_HAAR_SIZE_INC*layer) << octave;
            sampleSteps[index] = step;

            if( 0 < layer && layer <= nOctaveLayers )
                middleIndices[middleIndex++] = index;
            index++;
        }
        step *= 2;
    }

    // Calculate hessian determinant and trace samples in each layer
    parallel_for( BlockedRange(0, nTotalLayers),
                      SURFBuildInvoker(sum, sizes, sampleSteps, dets, traces) );

    // Find maxima in the determinant of the hessian
    parallel_for( BlockedRange(0, nMiddleLayers),
                      SURFFindInvoker(sum, mask_sum, dets, traces, sizes,
                                      sampleSteps, middleIndices, keypoints,
                                      nOctaveLayers, hessianThreshold) );
}

struct SURFInvoker
{
    enum { ORI_RADIUS = 6, ORI_WIN = 60, PATCH_SZ = 20 };

    SURFInvoker( const Mat& _img, const Mat& _sum,
                 vector<KeyPoint>& _keypoints, Mat& _descriptors,
                 bool _extended, bool _upright )
    {
        keypoints = &_keypoints;
        descriptors = &_descriptors;
        img = &_img;
        sum = &_sum;
        extended = _extended;
        upright = _upright;

        // Simple bound for number of grid points in circle of radius ORI_RADIUS
        const int nOriSampleBound = (2*ORI_RADIUS+1)*(2*ORI_RADIUS+1);

        // Allocate arrays
        apt.resize(nOriSampleBound);
        aptw.resize(nOriSampleBound);
        DW.resize(PATCH_SZ*PATCH_SZ);

        /* Coordinates and weights of samples used to calculate orientation */
        Mat G_ori = getGaussianKernel( 2*ORI_RADIUS+1, SURF_ORI_SIGMA, CV_32F );
        nOriSamples = 0;
        for( int i = -ORI_RADIUS; i <= ORI_RADIUS; i++ )
        {
            for( int j = -ORI_RADIUS; j <= ORI_RADIUS; j++ )
            {
                if( i*i + j*j <= ORI_RADIUS*ORI_RADIUS )
                {
                    apt[nOriSamples] = cvPoint(i,j);
                    aptw[nOriSamples++] = G_ori.at<float>(i+ORI_RADIUS,0) * G_ori.at<float>(j+ORI_RADIUS,0);
                }
            }
        }
        CV_Assert( nOriSamples <= nOriSampleBound );

        /* Gaussian used to weight descriptor samples */
        Mat G_desc = getGaussianKernel( PATCH_SZ, SURF_DESC_SIGMA, CV_32F );
        for( int i = 0; i < PATCH_SZ; i++ )
        {
            for( int j = 0; j < PATCH_SZ; j++ )
                DW[i*PATCH_SZ+j] = G_desc.at<float>(i,0) * G_desc.at<float>(j,0);
        }
    }

    void operator()(const BlockedRange& range) const
    {
        /* X and Y gradient wavelet data */
        const int NX=2, NY=2;
        const int dx_s[NX][5] = {{0, 0, 2, 4, -1}, {2, 0, 4, 4, 1}};
        const int dy_s[NY][5] = {{0, 0, 4, 2, 1}, {0, 2, 4, 4, -1}};

        // Optimisation is better using nOriSampleBound than nOriSamples for
        // array lengths.  Maybe because it is a constant known at compile time
        const int nOriSampleBound =(2*ORI_RADIUS+1)*(2*ORI_RADIUS+1);

        float X[nOriSampleBound], Y[nOriSampleBound], angle[nOriSampleBound];
        uchar PATCH[PATCH_SZ+1][PATCH_SZ+1];
        float DX[PATCH_SZ][PATCH_SZ], DY[PATCH_SZ][PATCH_SZ];
        CvMat matX = cvMat(1, nOriSampleBound, CV_32F, X);
        CvMat matY = cvMat(1, nOriSampleBound, CV_32F, Y);
        CvMat _angle = cvMat(1, nOriSampleBound, CV_32F, angle);
        Mat _patch(PATCH_SZ+1, PATCH_SZ+1, CV_8U, PATCH);

        int dsize = extended ? 128 : 64;

        int k, k1 = range.begin(), k2 = range.end();
        float maxSize = 0;
        for( k = k1; k < k2; k++ )
        {
            maxSize = std::max(maxSize, (*keypoints)[k].size);
        }
        int imaxSize = std::max(cvCeil((PATCH_SZ+1)*maxSize*1.2f/9.0f), 1);
        Ptr<CvMat> winbuf = cvCreateMat( 1, imaxSize*imaxSize, CV_8U );
        for( k = k1; k < k2; k++ )
        {
            int i, j, kk, x, y, nangle;
            float* vec;
            SurfHF dx_t[NX], dy_t[NY];
            KeyPoint& kp = (*keypoints)[k];
            float size = kp.size;
            Point2f center = kp.pt;
            /* The sampling intervals and wavelet sized for selecting an orientation
             and building the keypoint descriptor are defined relative to 's' */
            float s = size*1.2f/9.0f;
            /* To find the dominant orientation, the gradients in x and y are
             sampled in a circle of radius 6s using wavelets of size 4s.
             We ensure the gradient wavelet size is even to ensure the
             wavelet pattern is balanced and symmetric around its center */
            int grad_wav_size = 2*cvRound( 2*s );
            if( sum->rows < grad_wav_size || sum->cols < grad_wav_size )
            {
                /* when grad_wav_size is too big,
                 * the sampling of gradient will be meaningless
                 * mark keypoint for deletion. */
                kp.size = -1;
                continue;
            }

            float descriptor_dir = 90.f;
            if (upright == 0)
            {
                resizeHaarPattern( dx_s, dx_t, NX, 4, grad_wav_size, sum->cols );
                resizeHaarPattern( dy_s, dy_t, NY, 4, grad_wav_size, sum->cols );
                for( kk = 0, nangle = 0; kk < nOriSamples; kk++ )
                {
                    x = cvRound( center.x + apt[kk].x*s - (float)(grad_wav_size-1)/2 );
                    y = cvRound( center.y + apt[kk].y*s - (float)(grad_wav_size-1)/2 );
                    if( y < 0 || y >= sum->rows - grad_wav_size ||
                        x < 0 || x >= sum->cols - grad_wav_size )
                        continue;
                    const int* ptr = &sum->at<int>(y, x);
                    float vx = calcHaarPattern( ptr, dx_t, 2 );
                    float vy = calcHaarPattern( ptr, dy_t, 2 );
                    X[nangle] = vx*aptw[kk];
                    Y[nangle] = vy*aptw[kk];
                    nangle++;
                }
                if( nangle == 0 )
                {
                    // No gradient could be sampled because the keypoint is too
                    // near too one or more of the sides of the image. As we
                    // therefore cannot find a dominant direction, we skip this
                    // keypoint and mark it for later deletion from the sequence.
                    kp.size = -1;
                    continue;
                }
                matX.cols = matY.cols = _angle.cols = nangle;
                cvCartToPolar( &matX, &matY, 0, &_angle, 1 );

                float bestx = 0, besty = 0, descriptor_mod = 0;
                for( i = 0; i < 360; i += SURF_ORI_SEARCH_INC )
                {
                    float sumx = 0, sumy = 0, temp_mod;
                    for( j = 0; j < nangle; j++ )
                    {
                        int d = std::abs(cvRound(angle[j]) - i);
                        if( d < ORI_WIN/2 || d > 360-ORI_WIN/2 )
                        {
                            sumx += X[j];
                            sumy += Y[j];
                        }
                    }
                    temp_mod = sumx*sumx + sumy*sumy;
                    if( temp_mod > descriptor_mod )
                    {
                        descriptor_mod = temp_mod;
                        bestx = sumx;
                        besty = sumy;
                    }
                }
                descriptor_dir = fastAtan2( besty, bestx );
            }
            kp.angle = descriptor_dir;
            if( !descriptors || !descriptors->data )
                continue;

            /* Extract a window of pixels around the keypoint of size 20s */
            int win_size = (int)((PATCH_SZ+1)*s);
            CV_Assert( winbuf->cols >= win_size*win_size );
            Mat win(win_size, win_size, CV_8U, winbuf->data.ptr);

            if( !upright )
            {
            	descriptor_dir *= (float)(CV_PI/180);
                float sin_dir = std::sin(descriptor_dir);
                float cos_dir = std::cos(descriptor_dir);

                /* Subpixel interpolation version (slower). Subpixel not required since
                the pixels will all get averaged when we scale down to 20 pixels */
                /*
                float w[] = { cos_dir, sin_dir, center.x,
                -sin_dir, cos_dir , center.y };
                CvMat W = cvMat(2, 3, CV_32F, w);
                cvGetQuadrangleSubPix( img, &win, &W );
                */

                // Nearest neighbour version (faster)
                float win_offset = -(float)(win_size-1)/2;
                float start_x = center.x + win_offset*cos_dir + win_offset*sin_dir;
                float start_y = center.y - win_offset*sin_dir + win_offset*cos_dir;
                uchar* WIN = win.data;
                for( i = 0; i < win_size; i++, start_x += sin_dir, start_y += cos_dir )
                {
                    float pixel_x = start_x;
                    float pixel_y = start_y;
                    for( j = 0; j < win_size; j++, pixel_x += cos_dir, pixel_y -= sin_dir )
                    {
                        int x = std::min(std::max(cvRound(pixel_x), 0), img->cols-1);
                        int y = std::min(std::max(cvRound(pixel_y), 0), img->rows-1);
                        WIN[i*win_size + j] = img->at<uchar>(y, x);
                    }
                }
            }
            else
            {
                // extract rect - slightly optimized version of the code above
                // TODO: find faster code, as this is simply an extract rect operation,
                //       e.g. by using cvGetSubRect, problem is the border processing
                // descriptor_dir == 90 grad
                // sin_dir == 1
                // cos_dir == 0

                float win_offset = -(float)(win_size-1)/2;
                int start_x = cvRound(center.x + win_offset);
                int start_y = cvRound(center.y - win_offset);
                uchar* WIN = win.data;
                for( i = 0; i < win_size; i++, start_x++ )
                {
                    int pixel_x = start_x;
                    int pixel_y = start_y;
                    for( j = 0; j < win_size; j++, pixel_y-- )
                    {
                        x = MAX( pixel_x, 0 );
                        y = MAX( pixel_y, 0 );
                        x = MIN( x, img->cols-1 );
                        y = MIN( y, img->rows-1 );
                        WIN[i*win_size + j] = img->at<uchar>(y, x);
                    }
                }
            }
            // Scale the window to size PATCH_SZ so each pixel's size is s. This
            // makes calculating the gradients with wavelets of size 2s easy
            resize(win, _patch, _patch.size(), 0, 0, INTER_AREA);

            // Calculate gradients in x and y with wavelets of size 2s
            for( i = 0; i < PATCH_SZ; i++ )
                for( j = 0; j < PATCH_SZ; j++ )
                {
                    float dw = DW[i*PATCH_SZ + j];
                    float vx = (PATCH[i][j+1] - PATCH[i][j] + PATCH[i+1][j+1] - PATCH[i+1][j])*dw;
                    float vy = (PATCH[i+1][j] - PATCH[i][j] + PATCH[i+1][j+1] - PATCH[i][j+1])*dw;
                    DX[i][j] = vx;
                    DY[i][j] = vy;
                }

            // Construct the descriptor
            vec = descriptors->ptr<float>(k);
            for( kk = 0; kk < dsize; kk++ )
                vec[kk] = 0;
            double square_mag = 0;
            if( extended )
            {
                // 128-bin descriptor
                for( i = 0; i < 4; i++ )
                    for( j = 0; j < 4; j++ )
                    {
                        for( y = i*5; y < i*5+5; y++ )
                        {
                            for( x = j*5; x < j*5+5; x++ )
                            {
                                float tx = DX[y][x], ty = DY[y][x];
                                if( ty >= 0 )
                                {
                                    vec[0] += tx;
                                    vec[1] += (float)fabs(tx);
                                } else {
                                    vec[2] += tx;
                                    vec[3] += (float)fabs(tx);
                                }
                                if ( tx >= 0 )
                                {
                                    vec[4] += ty;
                                    vec[5] += (float)fabs(ty);
                                } else {
                                    vec[6] += ty;
                                    vec[7] += (float)fabs(ty);
                                }
                            }
                        }
                        for( kk = 0; kk < 8; kk++ )
                            square_mag += vec[kk]*vec[kk];
                        vec += 8;
                    }
            }
            else
            {
                // 64-bin descriptor
                for( i = 0; i < 4; i++ )
                    for( j = 0; j < 4; j++ )
                    {
                        for( y = i*5; y < i*5+5; y++ )
                        {
                            for( x = j*5; x < j*5+5; x++ )
                            {
                                float tx = DX[y][x], ty = DY[y][x];
                                vec[0] += tx; vec[1] += ty;
                                vec[2] += (float)fabs(tx); vec[3] += (float)fabs(ty);
                            }
                        }
                        for( kk = 0; kk < 4; kk++ )
                            square_mag += vec[kk]*vec[kk];
                        vec+=4;
                    }
            }

            // unit vector is essential for contrast invariance
            vec = descriptors->ptr<float>(k);
            float scale = (float)(1./(sqrt(square_mag) + DBL_EPSILON));
            for( kk = 0; kk < dsize; kk++ )
                vec[kk] *= scale;
        }
    }

    // Parameters
    const Mat* img;
    const Mat* sum;
    vector<KeyPoint>* keypoints;
    Mat* descriptors;
    bool extended;
    bool upright;

    // Pre-calculated values
    int nOriSamples;
    vector<Point> apt;
    vector<float> aptw;
    vector<float> DW;
};

SURFX::SURFX()
{
    hessianThreshold = 100;
    extended = true;
    upright = false;
    nOctaves = 4;
    nOctaveLayers = 2;
}

SURFX::SURFX(double _threshold, bool _extended, bool _upright, int _nOctaves, int _nOctaveLayers, int _pointCount)
{
    hessianThreshold = _threshold;
    extended = _extended;
    upright = _upright;
    nOctaves = _nOctaves;
    nOctaveLayers = _nOctaveLayers;
    pointCount = _pointCount;
}

int SURFX::descriptorSize() const { return extended ? 128 : 64; }
int SURFX::descriptorType() const { return CV_32F; }

void SURFX::operator()(InputArray imgarg, InputArray maskarg, vector<KeyPoint>& keypoints) const
{
	(*this)(imgarg, maskarg, keypoints, noArray(), false);
}

void SURFX::operator()(InputArray _img, InputArray _mask,
                      vector<KeyPoint>& keypoints,
                      OutputArray _descriptors,
                      bool useProvidedKeypoints) const
{
    Mat img = _img.getMat(), mask = _mask.getMat(), mask1, sum, msum;
    bool doDescriptors = _descriptors.needed();

    if( img.channels() > 1 )
        cvtColor(img, img, COLOR_BGR2GRAY);

    integral(img, sum, CV_32S);

    // Compute keypoints only if we are not asked for evaluating the descriptors are some given locations:
    if( !useProvidedKeypoints )
    {
        if( !mask.empty() )
        {
            cv::min(mask, 1, mask1);
            integral(mask1, msum, CV_32S);
        }
        fastHessianDetector( sum, msum, keypoints, nOctaves, nOctaveLayers, (float)hessianThreshold );
    }

    int i, j, N = (int)keypoints.size();
    if( N > 0 )
    {
        Mat descriptors;
        if( doDescriptors )
        {
            _descriptors.create((int)keypoints.size(), (extended ? 128 : 64), CV_32F);
            descriptors = _descriptors.getMat();
        }

        parallel_for(BlockedRange(0, N), SURFInvoker(img, sum, keypoints, descriptors, extended, upright) );

        size_t dsize = descriptors.cols*descriptors.elemSize();

        // remove keypoints that were marked for deletion
        for( i = j = 0; i < N; i++ )
        {
            if( keypoints[i].size > 0 )
            {
                if( i > j )
                {
                    keypoints[j] = keypoints[i];
                    if( doDescriptors )
                        memcpy( descriptors.ptr(j), descriptors.ptr(i), dsize);
                }
                j++;
            }
        }
        if( N > j )
        {
            N = j;
            keypoints.resize(N);
            if( doDescriptors )
            {
                Mat d = descriptors.rowRange(0, N);
                d.copyTo(_descriptors);
            }
        }
    }

}

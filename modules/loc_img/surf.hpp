#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv2/core/internal.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;

class SURFX
{
public:
    //! the default constructor
    SURFX();
    //! the full constructor taking all the necessary parameters
    explicit SURFX(double _hessianThreshold,
                  bool _extended=true, bool _upright=false,
                  int _nOctaves=4, int _nOctaveLayers=2, int _pointCount=100);

    //! returns the descriptor size in float's (64 or 128)
    int descriptorSize() const;

    //! returns the descriptor type
    int descriptorType() const;

    //! finds the keypoints using fast hessian detector used in SURF
    void operator()(InputArray img, InputArray mask,
                    vector<KeyPoint>& keypoints) const;

    //! finds the keypoints and computes their descriptors. Optionally it can compute descriptors for the user-provided keypoints
    void operator()(InputArray img, InputArray mask,
                    vector<KeyPoint>& keypoints,
                    OutputArray descriptors,
                    bool useProvidedKeypoints=false) const;


    double hessianThreshold;
    int nOctaves;
    int nOctaveLayers;
    bool extended;
    bool upright;



};

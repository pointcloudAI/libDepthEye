#ifndef POINTCLOUD_GLOBALSETTING_H
#define POINTCLOUD_GLOBALSETTING_H

#define PHASE_MAX_VALUE       4096
#define AMPTITUDE_MAX_VALUE   4096
#define MAX_NONLINEARITY_SIZE 16
#define SPEED_OF_LIGHT        299792458.0

#define MAX_PHASE_VALUE           0x0FFF
#define MAX_PHASE_RANGE           0x1000
#define TOF_ANGLE_TO_PHASE_FACTOR 4096 / (2 * M_PI)

#define UNAMBIGUOUS_RANGE "unambiguous_range"
#define NEAR_DISTANCE     "near_distance"
#define MEASURE_MODE      "measure_mode"
#define OUTPUT_MODE       "output_mode"
#define FILTER_EN         "filter_en"
#define AMP_THREHOLD      "amp_threhold"

#define HDR_EN     "hdr_en"
#define INTG_SCALE "intg_scale"
#define INTG_TIME  "intg_time"

#define AE_ENABLE                     "ae_enable"
#define AE_THRESHOLD                  "ae_threshold"
#define AE_STEP                       "ae_step"
#define TILLUM                        "tillum"
#define TSENSOR                       "tsensor"
#define MOD_F1                        "mod_freq1"
#define MOD_F2                        "mod_freq2"
#define DLYCNT                        "dlycnt"
#define ANADLYCNT                     "anadlycnt"
#define DLYCNTSF                      "dlycntsf"
#define TSENSOR_INDIVIDUAL_COMPATIBLE "tsensor_individual_compatible"

//--------------------------
#define PARAM_PHASE_OFFSETS          "phaseOffsets"
#define PARAM_FRAME_WIDTH            "frameWidth"
#define PARAM_FRAME_HEIGHT           "frameHeight"
#define PARAM_CALIB_SENSOR           "calibSensor"
#define PARAM_CALIB_ILLUM            "calibIllum"
#define PARAM_COEFF_SENSOR           "coeffSensor"
#define PARAM_COEFF_ILLUM            "coeffIllum"
#define PARAM_PHASE_MAX_CORR         "maxPhaseCorr"
#define PARAM_PHASE_MIN_CORR         "minPhaseCorr"
#define PARAM_MEASURE_MODE           "measureMode"
#define PARAM_OUTPUT_MODE            "outputMode"
#define PARAM_MAX_FREQ               "maxFreq"
#define PARAM_MIN_FREQ               "minFreq"
#define PARAM_MAX_NONLINEARITY_COEFF "phaseMaxLinCoeff"
#define PARAM_MIN_NONLINEARITY_COEFF "phaseMinLinCoeff"
#define PARAM_PIXCELWISE_ENABLE      "pixelwiseCalibEnable"
#define PARAM_CALIB_DISABLE          "calib_disable"
#define PARAM_NONLINEAR_ENABLE       "nonlinearityCalibEnable"
#define PARAM_COMMON_PHASE_ENABLE    "commonPhaseCalibEnable"

#define PARAM_FILTER_ENABLE     "filter_en"
#define PARAM_UNAMBIGUOUS_RANGE "unambiguous_range"
#define PARAM_NEAR_DISTANCE     "near_distance"

#define PARAM_AMPLITUDE_SCALING_FACTOR "amplitudeScalingFactor"
#define PARAM_DEPTH_SCALING_FACTOR     "depthScalingFactor"

#define PARAM_FISHEYE_ENABLE   "fisheye_enable"
#define PARAM_ROI_X            "roiX"
#define PARAM_ROI_Y            "roiY"
#define PARAM_ROI_WIDTH        "roiWidth"
#define PARAM_ROI_HEIGHT       "roiHeight"
#define PARAM_ROWS_TO_MERGE    "rowsToMerge"
#define PARAM_COLUMNS_TO_MERGE "columnsToMerge"
#define PARAM_AMP_THREHOLD     "ampThrehold"
#define PARAM_TRANS_THRESHOLD  "transThreshold"
#define PARAM_MAX_RANGE        "maxRange"
#define PARAM_CX               "cx"
#define PARAM_CY               "cy"
#define PARAM_FX               "fx"
#define PARAM_FY               "fy"
#define PARAM_K1               "k1"
#define PARAM_K2               "k2"
#define PARAM_K3               "k3"
#define PARAM_K4               "k4"
#define PARAM_P1               "p1"
#define PARAM_P2               "p2"
//----------------------
#define GAUSSIAN_KERNEL             "gaussian_kernel"
#define BTL_AB_THRESHOLD            "btl_ab_threshold"
#define BTL_MAX_EDGE                "btl_max_edge"
#define BTL_EXP                     "btl_exp"
#define INDIVIDUAL_AB_THRESHOLD     "individual_ab_threshold"
#define AB_THRESHOLD                "ab_threshold"
#define AB_CONFIDENCE_SLOPE         "ab_confidence_slope"
#define AB_CONFIDENCE_OFFSET        "ab_confidence_offset"
#define MIN_DEALIAS_CONFIDENCE      "min_dealias_confidence"
#define MAX_DEALIAS_CONFIDENCE      "max_dealias_confidence"
#define EDGE_AB_AVG_MIN_VALUE       "edge_ab_avg_min_value"
#define EDGE_AB_STD_DEV_THRESHOLD   "edge_ab_std_dev_threshold"
#define EDGE_CLOSE_DELTA_THRESHOLD  "edge_close_delta_threshold"
#define EDGE_FAR_DELTA_THRESHOLD    "edge_far_delta_threshold"
#define EDGE_MAX_DELTA_THRESHOLD    "edge_max_delta_threshold"
#define EDGE_AVG_DELTA_THRESHOLD    "edge_avg_delta_threshold"
#define MAX_EDGE_COUNT              "max_edge_count"
#define KDE_SIGMA_SQR               "kde_sigma_sqr"
#define UNWRAPPING_LIKELIHOOD_SCALE "unwrapping_likelihood_scale"
#define PHASE_CONFIDENCE_SCALE      "phase_confidence_scale"
#define KDE_THRESHOLD               "kde_threshold"
#define KDE_NEIGBORHOOD_SIZE        "kde_neigborhood_size"
#define NUM_HYPS                    "num_hyps"
//-------------------------
#define CALIB_SECT_LENS                          "lens"
#define CALIB_SECT_LENS_ID                       0
#define ToF_CALIB_SECT_FREQUENCY_ID              0
#define ToF_CALIB_SECT_CROSS_TALK_ID             1
#define ToF_CALIB_SECT_NON_LINEARITY_ID          2
#define ToF_CALIB_SECT_TEMPERATURE_ID            3
#define ToF_CALIB_SECT_COMMON_PHASE_OFFSET_ID    4
#define ToF_CALIB_SECT_PIXELWISE_PHASE_OFFSET_ID 5

#define DUAL_CDK_VENDOR_ID  0x1d6bU
#define DUAL_CDK_PRODUCT_ID 0x0102U

namespace PointCloud {
enum OutputMode {
    AMP_PHA = 0,
    II_IQ,
    A_B,
    AandB,
    AplusB,
    RGB
};

enum MeasureMode {
    USE_MOD_F1  = 1,
    USE_MOD_F2  = 2,
    USE_DEALIAS = 3
};

enum FreqIndex {
    MAX_FREQ = 0,
    MIN_FREQ = 1
};

enum ChipType {
    UNKNOW_TYPE = 0,
    MLX75027    = 1,
    MLX75026    = 2,
    IMX556      = 3,
    IMX570      = 4
};

struct LensCalib {
    float fx              = 0.0;
    float fy              = 0.0;
    float cx              = 0.0;
    float cy              = 0.0;
    float k1              = 0.0;
    float k2              = 0.0;
    float k3              = 0.0;
    float k4              = 0.0;
    float p1              = 0.0;
    float p2              = 0.0;
    float pha_scale       = 0.0;
    float trans_threshold = 1e-6;
};

struct FilterParam {
    float edge_ab_avg_min_value      = 50.0f;
    float edge_ab_std_dev_threshold  = 0.05f;
    float edge_close_delta_threshold = 50.0f;
    float edge_far_delta_threshold   = 30.0f;
    float edge_max_delta_threshold   = 100.0f;
    float edge_avg_delta_threshold   = 30.0f;
    float max_edge_count             = 3;

    float joint_bilateral_ab_threshold = 3.0f / 0.6666667f;
    float joint_bilateral_max_edge     = 2.5f;
    float joint_bilateral_exp          = 5.0f;
    float gaussian_kernel[9]           = {
        0.1069973f,
        0.1131098f,
        0.1069973f,
        0.1131098f,
        0.1195716f,
        0.1131098f,
        0.1069973f,
        0.1131098f,
        0.1069973f};

    float individual_ab_threshold = 3.0f;
    float ab_threshold            = 10.0f;
    float ab_confidence_slope     = -0.5330578f;
    float ab_confidence_offset    = 0.7694894f;
    float min_dealias_confidence  = 0.3490659f;
    float max_dealias_confidence  = 0.6108653f;

    // joint_bilateral_ab_threshold = 3.0f;
    /*
     * These are parameters for the method described in "Efficient Phase Unwrapping
     * using Kernel Density Estimation", ECCV 2016, Felix Järemo Lawin, Per-Erik Forssen and
     * Hannes Ovren, see http://www.cvl.isy.liu.se/research/datasets/kinect2-dataset/.
     */
    /*
    kde_sigma_sqr = 0.0239282226563f; //the scale of the kernel in the KDE, h in eq (13).
    unwrapping_likelihood_scale = 2.0f; //scale parameter for the unwrapping likelihood, s_1^2 in eq (15).
    phase_confidence_scale = 3.0f; //scale parameter for the phase likelihood, s_2^2 in eq (23)
    kde_threshold = 0.5f; //threshold on the KDE output in eq (25), defines the inlier/outlier rate trade-off

    kde_neigborhood_size = 5; //spatial support of the KDE, defines a filter size of (2*kde_neigborhood_size+1 x 2*kde_neigborhood_size+1)
    num_hyps = 2; //number of phase unwrapping hypothesis considered by the KDE in each pixel. Implemented values are 2 and 3.
    //a large kde_neigborhood_size improves performance but may remove fine structures and makes the processing slower.
    //setting num_hyp to 3 improves the performance slightly but makes processing slower
     */
};
struct FilterParams {
    float btl_ab_threshold   = 3.0f / 0.6666667f;
    float btl_max_edge       = 2.5f;
    float btl_exp            = 100.0f;  // 5.0f;
    float gaussian_kernel[9] = {
        0.1069973f,
        0.1131098f,
        0.1069973f,
        0.1131098f,
        0.1195716f,
        0.1131098f,
        0.1069973f,
        0.1131098f,
        0.1069973f};
    float kde_sigma_sqr           = 0.0239282226563f;  // the scale of the kernel in the KDE, h in eq (13).
    float individual_ab_threshold = 3.0f;
    float ab_threshold            = 10.0f;
    float ab_confidence_slope     = -0.5330578f;
    float ab_confidence_offset    = 0.7694894f;
    float min_dealias_confidence  = 0.3490659f;
    float max_dealias_confidence  = 0.6108653f;

    float edge_ab_avg_min_value      = 50.0f;
    float edge_ab_std_dev_threshold  = 0.05f;
    float edge_close_delta_threshold = 50.0f;
    float edge_far_delta_threshold   = 30.0f;
    float edge_max_delta_threshold   = 100.0f;
    float edge_avg_delta_threshold   = 30.0f;  // for test 0.0f;
    float max_edge_count             = 3;      // for test 5.0f;

    float unwrapping_likelihood_scale = 2.0f;  // scale parameter for the unwrapping likelihood, s_1^2 in eq (15).
    float phase_confidence_scale      = 3.0f;  // scale parameter for the phase likelihood, s_2^2 in eq (23)
    float kde_threshold               = 0.5f;  // threshold on the KDE output in eq (25), defines the inlier/outlier rate trade-off

    size_t kde_neigborhood_size = 5;           // spatial support of the KDE, defines a filter size of (2*kde_neigborhood_size+1 x 2*kde_neigborhood_size+1)
    size_t num_hyps             = 2;
};

struct TofConfSetting {
    uint16_t              version                = 1;
    uint16_t              firmware_version       = 1;
    uint32_t              left                   = 0;
    uint32_t              top                    = 0;
    uint32_t              rowsToMerge            = 0;
    uint32_t              columnsToMerge         = 0;
    uint32_t              amp_threhold           = 0;
    float                 max_range              = 0;
    uint16_t              width                  = 640;
    uint16_t              height                 = 480;
    bool                  disableOffsetCorr      = true;
    int16_t               maxPhaseCorr           = 0;
    int16_t               minPhaseCorr           = 0;
    uint8_t               maxFreq                = 100;
    uint8_t               minFreq                = 20;
    float                 amplitudeScalingFactor = 0.0;
    float                 depthScalingFactor     = 0.0;
    float                 tillumCalib            = 0;
    float                 tsensorCalib           = 0;
    float                 coeffIllum             = 0;
    float                 coeffSensor            = 0;
    bool                  aeEnable               = false;
    float                 aeStep                 = 0;
    int                   aeThreshold            = 0;
    bool                  fishEyeEnable          = false;
    MeasureMode           measure_mode           = USE_DEALIAS;
    std::vector<uint16_t> maxNLPhaseCorr;
    std::vector<uint16_t> minNLPhaseCorr;
    std::vector<float>    maxNLPhaseCorrRate;
    std::vector<float>    minNLPhaseCorrRate;
    double                maxNLCurveCoeffA;
    double                maxNLCurveCoeffB;
    double                maxNLCurveCoeffC;
    double                maxNLCurveCoeffD;
    double                minNLCurveCoeffA;
    double                minNLCurveCoeffB;
    double                minNLCurveCoeffC;
    double                minNLCurveCoeffD;
    std::vector<uint16_t> maxNLPhaseCorrLut;
    std::vector<uint16_t> minNLPhaseCorrLut;
    std::vector<int16_t>  pixelwiseCorr;
    int                   phasePeriod    = 128;
    int                   phase_duration = 2048;
    OutputMode            output_mode    = AMP_PHA;
    LensCalib             lens_calib;
    int                   calib_disable;
    bool                  pixelwiseCalibEnable = false;
    bool                  nonlinearityCalibEnable;
    bool                  useNonlinearityLut = false;
    bool                  commonPhaseCalibEnable;
    bool                  tempCorrCalibEnable;
    bool                  compat_data_en              = false;
    bool                  filterEnable                = false;
    float                 unambiguousRange            = 0;
    float                 near_distance               = 0;
    float                 calibTempOffset             = 0;
    bool                  tsensorIndividualCompatible = false;
    FilterParam           filterParam;
    bool                  closePhaseReversalEnable = false;
};
}  // namespace PointCloud
#endif

//These are all opencv type defines...
opencv.attr("CV_8U") = CV_8U;
opencv.attr("CV_8S") = CV_8S;
opencv.attr("CV_16U") = CV_16U;
opencv.attr("CV_16S") = CV_16S;
opencv.attr("CV_32S") = CV_32S;
opencv.attr("CV_32F") = CV_32F;
opencv.attr("CV_64F") = CV_64F;

opencv.attr("CV_8UC1") = CV_8UC1;
opencv.attr("CV_8UC2") = CV_8UC2;
opencv.attr("CV_8UC3") = CV_8UC3;
opencv.attr("CV_8UC4") = CV_8UC4;

opencv.attr("CV_8SC1") = CV_8SC1;
opencv.attr("CV_8SC2") = CV_8SC2;
opencv.attr("CV_8SC3") = CV_8SC3;
opencv.attr("CV_8SC4") = CV_8SC4;

opencv.attr("CV_16UC1") = CV_16UC1;
opencv.attr("CV_16UC2") = CV_16UC2;
opencv.attr("CV_16UC3") = CV_16UC3;
opencv.attr("CV_16UC4") = CV_16UC4;

opencv.attr("CV_16SC1") = CV_16SC1;
opencv.attr("CV_16SC2") = CV_16SC2;
opencv.attr("CV_16SC3") = CV_16SC3;
opencv.attr("CV_16SC4") = CV_16SC4;

opencv.attr("CV_32SC1") = CV_32SC1;
opencv.attr("CV_32SC2") = CV_32SC2;
opencv.attr("CV_32SC3") = CV_32SC3;
opencv.attr("CV_32SC4") = CV_32SC4;

opencv.attr("CV_32FC1") = CV_32FC1;
opencv.attr("CV_32FC2") = CV_32FC2;
opencv.attr("CV_32FC3") = CV_32FC3;
opencv.attr("CV_32FC4") = CV_32FC4;

opencv.attr("CV_64FC1") = CV_64FC1;
opencv.attr("CV_64FC2") = CV_64FC2;
opencv.attr("CV_64FC3") = CV_64FC3;
opencv.attr("CV_64FC4") = CV_64FC4;

// matrix decomposition types
enum
{
  DECOMP_LU = 0, DECOMP_SVD = 1, DECOMP_EIG = 2, DECOMP_CHOLESKY = 3, DECOMP_QR = 4, DECOMP_NORMAL = 16
};
enum
{
  NORM_INF = 1, NORM_L1 = 2, NORM_L2 = 4, NORM_TYPE_MASK = 7, NORM_RELATIVE = 8, NORM_MINMAX = 32
};
enum
{
  CMP_EQ = 0, CMP_GT = 1, CMP_GE = 2, CMP_LT = 3, CMP_LE = 4, CMP_NE = 5
};
enum
{
  GEMM_1_T = 1, GEMM_2_T = 2, GEMM_3_T = 4
};
enum
{
  DFT_INVERSE = 1, DFT_SCALE = 2, DFT_ROWS = 4, DFT_COMPLEX_OUTPUT = 16, DFT_REAL_OUTPUT = 32,
  DCT_INVERSE = DFT_INVERSE, DCT_ROWS = DFT_ROWS
};


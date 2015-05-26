#include "solvep3p.h"

SolveP3P::SolveP3P()
{
    threeImageCoord = cv::Mat(3,1,cv::DataType<cv::Point2f>::type);
    threeWorldCoord = cv::Mat(3,1,cv::DataType<cv::Point3f>::type);
}

Matrix4d SolveP3P::solveP3P(cv::Mat worldCoord, cv::Mat imageCoord, cv::Mat cameraMatrix, cv::Mat distCoeffs, cv::Mat rvec, cv::Mat tvec, cv::Mat img)
{
    this->img = img;
    this->imageCoord = imageCoord;

    // only use three image coord, and the forth for finding the best solution
    // sort image points to be:
    // 0 4        A D
    // 1 2   or   B C
    threeImageCoord.at<cv::Point2f>(0) = imageCoord.at<cv::Point2f>(0);
    threeImageCoord.at<cv::Point2f>(1) = imageCoord.at<cv::Point2f>(1);
    threeImageCoord.at<cv::Point2f>(2) = imageCoord.at<cv::Point2f>(2);
////    imageCoord.at<cv::Point2f>(0) = (cv::Point2f){270,153};
////    imageCoord.at<cv::Point2f>(1) = (cv::Point2f){320,240};
////    imageCoord.at<cv::Point2f>(2) = (cv::Point2f){420,240};
////    imageCoord.at<cv::Point2f>(0) = (cv::Point2f){320,140};
////    imageCoord.at<cv::Point2f>(1) = (cv::Point2f){320,240};
////    imageCoord.at<cv::Point2f>(2) = (cv::Point2f){420,240};

    // only use three world coord, and the forth for finding the best solution
    threeWorldCoord.at<cv::Point3f>(0) = worldCoord.at<cv::Point3f>(0);
    threeWorldCoord.at<cv::Point3f>(1) = worldCoord.at<cv::Point3f>(1);
    threeWorldCoord.at<cv::Point3f>(2) = worldCoord.at<cv::Point3f>(2);
////    worldCoord.at<cv::Point3f>(0) = (cv::Point3f){-0.05,0.0866,0};
////    worldCoord.at<cv::Point3f>(1) = (cv::Point3f){0,0,0};
////    worldCoord.at<cv::Point3f>(2) = (cv::Point3f){0.1,0,0};
////    worldCoord.at<cv::Point3f>(0) = (cv::Point3f){0,0.1,0};
////    worldCoord.at<cv::Point3f>(1) = (cv::Point3f){0,0,0};
////    worldCoord.at<cv::Point3f>(2) = (cv::Point3f){0.1,0,0};
////    if(DEBUG) cout << "3worldCoord: " << worldCoord << endl;

    normalizeImagePoints(cameraMatrix, distCoeffs);
    setUpP3PEquationSystem();

    Matrix4d T = chooseBestSolution();

//    cv::Mat estimatedWorldCoord = cv::Mat(3,1,cv::DataType<cv::Point3f>::type);
//    estimatedWorldCoord = solveP3PEquationSystem();
////    if(DEBUG) cout << "estimatedWorldCoord: " << estimatedWorldCoord << endl;

//    Matrix4d T = rigidTransform(worldCoord, estimatedWorldCoord);

//    if(DEBUG) cout << "T: " << T << endl;
    rvec = rvecFromT(T);
    tvec = tvecFromT(T);

    return T;
}

// parameters:
// imageCoord: cv::Mat of cv::Point2f
// size: number of points in imageCoord
void SolveP3P::normalizeImagePoints(cv::Mat cameraMatrix, cv::Mat distCoeffs)
{
    // set camera parameters
    this->cameraMatrix = cameraMatrix;
    this->distCoeffs = distCoeffs;
    fx = cameraMatrix.at<double>(0,0);
    fy = cameraMatrix.at<double>(1,1);
    cx = cameraMatrix.at<double>(0,2);
    cy = cameraMatrix.at<double>(1,2);

    int size = *threeImageCoord.size;

    normalizedCoord = cv::Mat(size,1,cv::DataType<cv::Point3f>::type);
    for (int i = 0; i < size; i++)
    {
        double x = (threeImageCoord.at<cv::Point2f>(i).x - cx) / fx;
        double y = (threeImageCoord.at<cv::Point2f>(i).y - cy) / fy;
        double z = 1;

        double N = sqrt(x*x + y*y + z*z);

        normalizedCoord.at<cv::Point3f>(i).x = (float) x / N;
        normalizedCoord.at<cv::Point3f>(i).y = (float) y / N;
        normalizedCoord.at<cv::Point3f>(i).z = (float) z / N;

//        if(DEBUG) cout << "threeImageCoord: " << threeImageCoord.at<cv::Point2f>(i).x << ", " << threeImageCoord.at<cv::Point2f>(i).y << endl;
//        if(DEBUG) cout << "normalizedCoord: " << normalizedCoord.at<cv::Point3f>(i).x << ", " << normalizedCoord.at<cv::Point3f>(i).y << ", " << normalizedCoord.at<cv::Point3f>(i).z << endl;
    }
}

void SolveP3P::setUpP3PEquationSystem()
{
    // TODO can't assume right triangles, must use law of cosines! **********************
    // calculate angles to points on image plane
    u = normalizedCoord.at<cv::Point3f>(0);
    v = normalizedCoord.at<cv::Point3f>(1);
    w = normalizedCoord.at<cv::Point3f>(2);
    double v_u = sqrt((v.x-u.x)*(v.x-u.x)+(v.y-u.y)*(v.y-u.y)+(v.z-u.z)*(v.z-u.z));
    double w_v = sqrt((w.x-v.x)*(w.x-v.x)+(w.y-v.y)*(w.y-v.y)+(w.z-v.z)*(w.z-v.z));
    double w_u = sqrt((w.x-u.x)*(w.x-u.x)+(w.y-u.y)*(w.y-u.y)+(w.z-u.z)*(w.z-u.z));
    double p_u = sqrt(pow(u.x,2) + pow(u.y,2) + pow(u.z,2));
    double p_v = sqrt(pow(v.x,2) + pow(v.y,2) + pow(v.z,2));
    double p_w = sqrt(pow(w.x,2) + pow(w.y,2) + pow(w.z,2));
//    uv = acos(u.x*v.x + u.y*v.y + u.z*v.z);
//    uw = acos(u.x*w.x + u.y*w.y + u.z*w.z);
//    vw = acos(v.x*w.x + v.y*w.y + v.z*w.z);
    uv = acos((-pow(v_u,2) + pow(p_v,2) + pow(p_u,2)) / (2*p_v*p_u));
    uw = acos((-pow(w_u,2) + pow(p_w,2) + pow(p_u,2)) / (2*p_w*p_u));
    vw = acos((-pow(w_v,2) + pow(p_w,2) + pow(p_v,2)) / (2*p_w*p_v));
//    uv = 0.197;
//    uw = 0.34;
//    vw = 0.197;

    if(DEBUG) cout << "angles: " << uv << ", " << uw << ", " << vw << endl;

    // calculate distances AB, BC, and CA
//    // These are actually distances uv vw wu but aa and bb below are only the ratio so it doesn't matter
//    AB = sqrt((v.x-u.x)*(v.x-u.x)+(v.y-u.y)*(v.y-u.y)+(v.z-u.z)*(v.z-u.z));
//    BC = sqrt((w.x-v.x)*(w.x-v.x)+(w.y-v.y)*(w.y-v.y)+(w.z-v.z)*(w.z-v.z));
//    CA = sqrt((w.x-u.x)*(w.x-u.x)+(w.y-u.y)*(w.y-u.y)+(w.z-u.z)*(w.z-u.z));
    cv::Point3f A = threeWorldCoord.at<cv::Point3f>(0);
    cv::Point3f B = threeWorldCoord.at<cv::Point3f>(1);
    cv::Point3f C = threeWorldCoord.at<cv::Point3f>(2);
    AB = sqrt(pow(A.x-B.x,2) + pow(A.y-B.y,2) + pow(A.z-B.z,2));
    BC = sqrt(pow(B.x-C.x,2) + pow(B.y-C.y,2) + pow(B.z-C.z,2));
    CA = sqrt(pow(A.x-C.x,2) + pow(A.y-C.y,2) + pow(A.z-C.z,2));

//    if(DEBUG) cout << "AB, BC, CA: " << AB << ", " << BC << ", " << CA << endl;

    //calculate coefficients
    aa = (BC*BC) / (AB*AB);
    bb = (CA*CA) / (AB*AB);

    if(DEBUG) cout << "aa bb : " << aa << ", " << bb << endl;
}

cv::Mat SolveP3P::solveP3PEquationSystem()
{
    // use Wu Ritt's zero composition to get coefficients a4, a3, a2, a1, a0, b1 and b0
    double p = 2*cos(vw);
    double q = 2*cos(uw);
    double r = 2*cos(uv);

    if(DEBUG) cout << "pqr: " << p << ", " << q << ", " << r << endl;

    // **** TS1 ****
    // Solving the general case
    // a4*x^4+a3*x^3+a2*x^2+a1*x^1+a0=0
    double a5 = aa*p*r + 2*q*aa - r*p*bb + 2*bb*q - 2*q - aa*pow(r,2)*q + p*r;
    a4 = -2*bb + pow(bb,2) + pow(aa,2) + 1 - bb*pow(r,2)*aa +  2*bb*aa - 2*aa;
    a3 = -2*bb*q*aa - 2*pow(aa,2)*q + bb*pow(r,2)*q*aa - 2*q + 2*bb*q + 4*aa*q + p*bb*r + bb*r*p*aa - pow(bb,2)*r*p;
    a2 = pow(q,2) + pow(bb,2)*pow(r,2) - bb*pow(p,2) - q*p*bb*r + pow(bb,2)*pow(p,2) - bb*pow(r,2)*aa +
            2 - 2*pow(bb,2) - aa*bb*r*p*q + 2*pow(aa,2) - 4*aa - 2*pow(q,2)*aa + pow(q,2)*pow(aa,2);
    a1 = -pow(bb,2)*r*p + bb*r*p*aa - 2*pow(aa,2)*q + q*pow(p,2)*bb + 2*bb*q*aa + 4*aa*q + p*bb*r - 2*bb*q - 2*q;
    a0 = 1 - 2*aa + 2*bb + pow(bb,2) - bb*pow(p,2) + pow(aa,2) - 2*bb*aa;

    // SolveP4 return value
    // return 4: 4 real roots x[0], x[1], x[2], x[3], possible multiple roots
    // return 2: 2 real roots x[0], x[1] and complex x[2]i*x[3],
    // return 0: two pair of complex roots: x[0]i*x[1],  x[2]i*x[3],
    double X [4];
    int ret = SolveP4(X, a3/a4, a2/a4, a1/a4, a0/a4);
    if(DEBUG) cout << "ret: " << ret << endl;
    if(DEBUG) cout << "X: " << X[0] << ", " << X[1] << ", " << X[2] << ", " << X[3] << endl;

    double x = X[xSolu];
//    x = 1;
    if(DEBUG) cout << "x: " << x << endl;
/*
//    double b2 = bb*(-4*aa*pow(r,3) + 4*pow(r,3) + aa*pow(r,5) - 2*pow(p,3)*q  + 4*r*pow(p,2) - 6*p*q*pow(r,2) - 4*r*pow(p,2)*bb - 4*pow(p,2)*aa*r + 6*p*aa*pow(r,2)*q +
//                    2*pow(p,2)*r*pow(q,2) + 2*pow(p,2)*aa*pow(r,3) + 2*pow(p,3)*bb*q + 2*pow(p,3)*q*aa + pow(p,4)*aa*r + pow(p,2)*aa*pow(r,3)*pow(q,2) - 2*pow(p,2)*r*pow(q,2)*aa - pow(p,2)*r*bb*pow(q,2) -
//                    2*pow(p,3)*aa*pow(r,2)*q - 2*p*aa*pow(r,4)*q + 2*p*pow(r,2)*bb*q);
//    b1 = bb*pow((pow(p,2)*aa - pow(p,2) + bb*pow(p,2) + p*q*r - q*aa*r*p + aa*pow(r,2) - pow(r,2) - bb*pow(r,2)),2);
//    b0 = ((1-aa-bb)*pow(x,2) + (q*aa-q)*x + 1 - aa + bb)*((pow(aa,2)*pow(r,3) + 2*bb*pow(r,3)*aa - bb*pow(r,5)*aa - 2*aa*pow(r,3) + pow(r,3) + pow(bb,2)*pow(r,3) - 2*pow(r,3)*bb)*pow(x,3) +
//            (p*pow(r,2) + p*pow(aa,2)*pow(r,2) - 2*bb*pow(r,3)*q*aa + 2*pow(r,3)*bb*q + 2*pow(r,3)*q - 2*p*aa*pow(r,2) - 2*p*pow(r,2)*bb + pow(r,4)*p*bb + 4*aa*pow(r,3)*q + bb*q*aa*pow(r,5) - 2*pow(r,3)*pow(aa,2)*q +
//             2*pow(r,2)*p*bb*aa + pow(bb,2)*pow(r,2)*p - pow(r,4)*p*pow(bb,2))*pow(x,2) + (pow(r,3)*pow(q,2) + pow(r,5)*pow(bb,2) + r*pow(p,2)*pow(bb,2) - 4*aa*pow(r,3) - 2*aa*pow(r,3)*pow(q,2) + pow(r,3)*pow(q,2)*pow(aa,2) +
//             2*pow(aa,2)*pow(r,3) - 2*pow(bb,2)*pow(r,3) - 2*pow(p,2)*bb*r + 4*p*aa*pow(r,2)*q + 2*aa*pow(p,2)*r*bb - 2*aa*pow(r,2)*q*bb*p - 2*pow(p,2)*aa*r + r*pow(p,2) - bb*pow(r,5)*aa + 2*p*pow(r,2)*bb*q +
//             r*pow(p,2)*pow(aa,2) - 2*p*q*pow(r,2) + 2*pow(r,3) - 2*pow(r,2)*p*pow(aa,2)*q - pow(r,4)*q*bb*p)*x + 4*aa*pow(r,3)*q + p*pow(r,2)*pow(q,2) + 2*pow(p,3)*bb*aa - 4*p*aa*pow(r,2) +
//             -2*pow(r,3)*bb*q - 2*pow(p,2)*q*r - 2*pow(bb,2)*pow(r,2)*p + pow(r,4)*p*bb + 2*p*pow(aa,2)*pow(r,2) - 2*pow(r,3)*pow(aa,2)*q - 2*pow(p,3)*aa + pow(p,3)*pow(aa,2) + 2+p+pow(r,2) + pow(p,3) + 2*bb*pow(r,3)*q*aa +
//             2*q*pow(p,2)*bb*r + 4*q*aa*r*pow(p,2) - 2*p*aa*pow(r,2)*pow(q,2) - 2*pow(p,2)*pow(aa,2)*r*q + p*pow(aa,2)*pow(r,2)*pow(q,2) - 2*pow(r,3)*q - 2*pow(p,3)*bb + pow(p,3)*pow(bb,2) - 2*pow(p,2)*bb*r*q*aa);

//    double I0 = pow(p,2) + pow(q,2) + pow(r,2) - p*q*r - 1;
//    double I1 = a0;
//    double I2 = b0;
//    double I3 = a5;
//    double I4 = b2;
//    double I5 = r;
//    double I6 = r*p - 4*p*q + r*pow(q,2);
//    double I7 = p;
//    double I8 = (pow(p,2) + pow(q,2))*bb - pow(p,2);

//    double T [10];
//    T[1] = I0*I1*I2;
//    T[2] = I0*I2*I3*I4;
//    T[3] = I0*I2*I4*I5*I6*I7;
//    T[4] = I0*I2*I4*I6*I7*I8;
//    T[5] = I0*I2*I4*I6*I7;
//    T[6] = I0*I2*I4*I5*I7;
//    T[7] = I0*I2*I3;
//    T[8] = T[7];
//    T[9] = I0*I1;
//    T[10] = T[9];
//    T[11] = T[9];
//    for(int i = 1; i < 12; i++)
//    {
//        if(DEBUG) cout << "T" << i << ": " << T[i] << endl;
//    }


//    // **** TS2 ****
//    double ts2 = pow(aa,2) + (2 + 2*bb - bb*pow(r,2))*aa - 2*bb + pow(bb,2) + 1;
//    if(DEBUG) cout << "TS2: " << ts2 << endl;

//    // ***** TS3 ****
//    double ts3a = (-4*pow(p,2) + 4*p*q*r + pow(r,2)*pow(p,2) + pow(r,2)*pow(q,2) - pow(r,3)*p*q - 4*pow(q,2))*aa + pow(r,2)*pow(p,2) - 4*p*q*r + 4*pow(q,2);
//    double ts3b = (-4*pow(p,2) + 4*p*q*r + pow(r,2)*pow(p,2) + pow(r,2)*pow(q,2) - pow(r,3)*p*q - 4*pow(q,2))*bb + pow(r,2)*pow(q,2) + 4*pow(p,2) - 4*p*q*r;
//    if(DEBUG) cout << "TS3a: " << ts3a << endl;
//    if(DEBUG) cout << "TS3b: " << ts3b << endl;

//    // **** TS4 ****
//    // Solving the right angle case
//    double ts4a = aa + bb - 1;
//    double ts4b = r;
//    if(DEBUG) cout << "TS4a: " << ts4a << endl;
//    if(DEBUG) cout << "TS4b: " << ts4b << endl;

////    double X_ [2];
////    double a_ = pow(p,2)*bb + pow(q,2)*bb - pow(p,2);
////    double b_ = -4*bb*q + pow(p,2)*q;
////    double c_ = 4*bb - pow(p,2);
////    if(DEBUG) cout << "a_b_c_: " << a_ << ", " << b_ << ", " << c_ << endl;
////    if(DEBUG) cout << "sqrt: " << pow(b_,2) - 4*a_*c_ << endl;
////    X_[0] = (-b_ + sqrt(pow(b_,2) - 4*a_*c_)) / (2*a_);
////    X_[1] = (-b_ - sqrt(pow(b_,2) - 4*a_*c_)) / (2*a_);
////    if(DEBUG) cout << "X_: " << X_[0] << ", " << X_[1] << endl;

//    // **** TS5 ****
//    double ts5a = (pow(p,2) + pow(q,2))*aa - pow(q,2);
//    double ts5b = (pow(p,2) + pow(q,2))*bb - pow(p,2);
//    double ts5c = r;
//    if(DEBUG) cout << "TS5a: " << ts5a << endl;
//    if(DEBUG) cout << "TS5b: " << ts5b << endl;
//    if(DEBUG) cout << "TS5c: " << ts5c << endl;

//    // **** TS6 ****
//    double ts6a = (pow(p,4) - 2*pow(p,2)*pow(q,2) + pow(q,4))*aa -  pow(p,2)*pow(q,2) - pow(q,4);
//    double ts6b = (pow(p,4) - 2*pow(p,2)*pow(q,2) + pow(q,4))*bb -  pow(p,2)*pow(q,2) - pow(p,4);
//    double ts6c = (pow(p,2) + pow(q,2))*r - 4*p*q;
//    if(DEBUG) cout << "TS6a: " << ts6a << endl;
//    if(DEBUG) cout << "TS6b: " << ts6b << endl;
//    if(DEBUG) cout << "TS6c: " << ts6c << endl;

//    // **** TS7 ****
//    double ts7 = (4*pow(r,2) + pow(p,2)*pow(q,2) + pow(p,4) - pow(r,4) - pow(p,3)*q*r + p*pow(r,3)*q - 4*q*p*r)*bb + 2*p*pow(r,3)*q - 2*pow(p,2)*pow(r,2) + 2*pow(p,3)*q*r - pow(p,2)*pow(q,2)*pow(r,2) - pow(p,4) - pow(r,4);
//    if(DEBUG) cout << "TS7: " << ts7 << endl;

//    // **** TS8 ****
////    double ts8a = -pow(p,2)*pow(q,2)*pow(r,2) + 2*p*pow(r,3)*q + 2*pow(p,3)*q*r - pow(p,4) - pow(r,4) - 2*pow(p,2)*pow(r,2) - 4*q*bb*p*r + pow(q,2)*bb*pow(r,2) + 4*bb*pow(p,2);
////    double ts8b = (-q*p*r + pow(p,2) + pow(r,2))*y + p*q*

//    // **** TS9 ****
//    double ts9a = p;
//    double ts9b = r;
//    if(DEBUG) cout << "TS9a: " << ts9a << endl;
//    if(DEBUG) cout << "TS9b: " << ts9b << endl;

//    // **** TS10 ****
//    double ts10a = (-p*q*pow(r,3) + pow(r,4) + r*pow(p,3)*q - 4*pow(r,2) - pow(p,2)*pow(q,2) + 4*r*p*q - pow(p,4))*aa + pow(p,2)*pow(q,2) - 4*r*p*q + 4*pow(r,2);
//    double ts10b = (-p*q*pow(r,3) + pow(r,4) + r*pow(p,3)*q - 4*pow(r,2) - pow(p,2)*pow(q,2) + 4*r*p*q - pow(p,4))*bb + pow(p,4) + pow(r,4) + 2*pow(r,2)*pow(p,2) + pow(p,2)*pow(r,2)*pow(q,2) - 2*r*pow(p,3)*q - 2*p*q*pow(r,3);
//    if(DEBUG) cout << "TS10a: " << ts10a << endl;
//    if(DEBUG) cout << "TS10b: " << ts10b << endl;

//    // **** TS11 ****
//    double ts11a = (-p*q*pow(r,3) + pow(r,4) + r*pow(p,3)*q - 4*pow(r,2) - pow(p,2)*pow(q,2) + 4*r*p*q - pow(p,4))*aa + pow(p,2)*pow(q,2) - 4*r*p*q + 4*pow(r,2);
//    double ts11b = (-p*q*pow(r,3) + pow(r,4) + r*pow(p,3)*q - 4*pow(r,2) - pow(p,2)*pow(q,2) + 4*r*p*q - pow(p,4))*bb + pow(p,4) + pow(r,4) + 2*pow(r,2)*pow(p,2) + pow(p,2)*pow(r,2)*pow(q,2) - 2*r*pow(p,3)*q - 2*p*q*pow(r,3);
//    if(DEBUG) cout << "TS11a: " << ts11a << endl;
//    if(DEBUG) cout << "TS11b: " << ts11b << endl;
*/


    // Solve quadratic for y
    double j = 1-aa;
    double k = 2*aa*x*cos(uv) - 2*cos(vw);
    double l = 1-aa*pow(x,2);
    if(DEBUG) cout << "jkl: " << j << ", " << k << ", " << l << endl;
    if(DEBUG) cout << "sqrt: " << pow(k,2) - 4*j*l << endl;

    double Y [3];
    Y[0] = (-k + sqrt(pow(k,2) - 4*j*l)) / (2*j);
    Y[1] = (-k - sqrt(pow(k,2) - 4*j*l)) / (2*j);
    Y[2] = -l/k;
    if(DEBUG) cout << "y0: " << Y[0] << endl;
    if(DEBUG) cout << "y1: " << Y[1] << endl;
    if(DEBUG) cout << "y2: " << Y[2] << endl;

    double y;
//    y = 0.98;
    y = Y[ySolu];
    if(DEBUG) cout << "y: " << y << endl;

    // plug into P3P equation system to check x and y, should equal 0
    double eq0 = (1-aa)*pow(y,2) - aa*pow(x,2) - 2*cos(vw)*y + 2*aa*cos(uv)*x*y + 1;
    double eq1 = (1-bb)*pow(x,2) - bb*pow(y,2) - 2*cos(uw)*x + 2*bb*cos(uv)*x*y + 1;
    if(DEBUG) cout << "eq0: " << eq0 << endl;
    if(DEBUG) cout << "eq1: " << eq1 << endl;

    double V = pow(x,2) + pow(y,2) - 2*x*y*cos(uv);
//    PC = sqrt(pow(AB,2)/V);  // using v = AB2/PC2
    double markerSpacing = 0.105;
    PC = sqrt(pow(markerSpacing,2)/V);  // using v = AB2/PC2
    PB = y*PC; // using y = PB/PC
    PA = x*PC; // using x = PA/PC

//     PC = 0.51;
//    PB = 0.5;
//    PA = 0.51;

    if(DEBUG) cout << "PA PB PC: " << PA << ", " << PB << ", " << PC << endl;

    // scale points to find estimated projected points
    AA.x = u.x * PA;
    AA.y = u.y * PA;
    AA.z = u.z * PA;
    BB.x = v.x * PB;
    BB.y = v.y * PB;
    BB.z = v.z * PB;
    CC.x = w.x * PC;
    CC.y = w.y * PC;
    CC.z = w.z * PC;

    // estimate DD, the forth point, using the other 3
    // Note: This will only work for 4 points in a square in a plane ****
    cv::Point3f DD; // forth point estimate
    cv::Point3f EE; // center of marker
    EE.x = AA.x + ((CC.x - AA.x) / 2);
    EE.y = AA.y + ((CC.y - AA.y) / 2);
    EE.z = AA.z + ((CC.z - AA.z) / 2);
    DD.x = EE.x + (EE.x - BB.x);
    DD.y = EE.y + (EE.y - BB.y);
    DD.z = EE.z + (EE.z - BB.z);

    if(DEBUG) cout << "estimated world coord:" << endl;
    if(DEBUG) cout << "AA: " << AA << endl;
    if(DEBUG) cout << "BB: " << BB << endl;
    if(DEBUG) cout << "CC: " << CC << endl;
    if(DEBUG) cout << "DD: " << DD << endl;
    if(DEBUG) cout << "EE: " << EE << endl;

    // return vector of points
    cv::Mat estimatedWorldPoints = cv::Mat(4,1,cv::DataType<cv::Point3f>::type);
    estimatedWorldPoints.at<cv::Point3f>(0) = AA;
    estimatedWorldPoints.at<cv::Point3f>(1) = BB;
    estimatedWorldPoints.at<cv::Point3f>(2) = CC;
    estimatedWorldPoints.at<cv::Point3f>(3) = DD;

    return estimatedWorldPoints;
}

Matrix4d SolveP3P::chooseBestSolution()
{
    int xSolutions = 3;
    int ySolutions = 2;
    // transform to return
    Matrix4d T;
    // vector to store all Ts
    vector<Matrix4d> allT;
    // vector to store all errors
    vector<double> errors;
    for (xSolu = 0; xSolu < xSolutions; xSolu++)
    {
        for (ySolu = 0; ySolu < ySolutions; ySolu++)
        {
            int index = xSolu * xSolutions + ySolu;
            if(DEBUG) cout << "Index: " << index << endl;

            cv::Mat estimatedWorldCoord = cv::Mat(4,1,cv::DataType<cv::Point3f>::type);
            estimatedWorldCoord = solveP3PEquationSystem();

            cv::Mat threeEstimatedWorldCoord = cv::Mat(3,1,cv::DataType<cv::Point3f>::type);
            threeEstimatedWorldCoord.at<cv::Point3f>(0) = estimatedWorldCoord.at<cv::Point3f>(0);
            threeEstimatedWorldCoord.at<cv::Point3f>(1) = estimatedWorldCoord.at<cv::Point3f>(1);
            threeEstimatedWorldCoord.at<cv::Point3f>(2) = estimatedWorldCoord.at<cv::Point3f>(2);

            Matrix4d transform = rigidTransform(threeWorldCoord, threeEstimatedWorldCoord);

            allT.push_back(transform);
//            T = transform;   // ************

            // get rvec and tvec
            cv::Mat rmat(3,3,cv::DataType<double>::type);
            cv::Mat rvec(3,1,cv::DataType<double>::type);
            cv::Mat tvec(3,1,cv::DataType<double>::type);
            rmat = eigenToCvMat(transform.topLeftCorner(3,3),3,3);
            cv::Rodrigues(rmat, rvec);
            tvec = eigenToCvMat(transform.topRightCorner(3,1),3,1);

//            // project axis ****
//            cv::Mat axis = cv::Mat(4,1,cv::DataType<cv::Point3f>::type); // 1 marker
//            std::vector<cv::Point2f> projectedAxis;
//            axis.at<cv::Point3f>(0) = (cv::Point3f){0,0,0};
//            axis.at<cv::Point3f>(1) = (cv::Point3f){0.1,0,0};
//            axis.at<cv::Point3f>(2) = (cv::Point3f){0,0.1,0};
//            axis.at<cv::Point3f>(3) = (cv::Point3f){0,0,0.1};

//            cv::projectPoints(axis, rvec, tvec, cameraMatrix, distCoeffs, projectedAxis);

//            cv::line(img, projectedAxis[0], projectedAxis[1], cv::Scalar(0,0,250), 2);
//            cv::line(img, projectedAxis[0], projectedAxis[2], cv::Scalar(0,250,0), 2);
//            cv::line(img, projectedAxis[0], projectedAxis[3], cv::Scalar(255,255,0), 2);
//            // ****

            // project forth image point
            vector<cv::Point2f> projectedMarkerPoints;
            vector<cv::Point3f> markerPoints;
//            forthPointVector.push_back(estimatedWorldCoord.at<cv::Point3f>(3));
            markerPoints.push_back((cv::Point3f){-0.05,-0.05,0});
            markerPoints.push_back((cv::Point3f){0.05,-0.05,0});
            markerPoints.push_back((cv::Point3f){-0.05,0.05,0});
            markerPoints.push_back((cv::Point3f){0.05,0.05,0});
            cv::projectPoints(markerPoints, rvec, tvec, cameraMatrix, distCoeffs, projectedMarkerPoints);
//            cv::circle(img,projectedMarkerPoints.at(0),2,cv::Scalar(0,255,255),-1);
//            cv::circle(img,projectedMarkerPoints.at(1),2,cv::Scalar(0,255,255),-1);
//            cv::circle(img,projectedMarkerPoints.at(2),2,cv::Scalar(0,255,255),-1);
//            cv::circle(img,projectedMarkerPoints.at(3),2,cv::Scalar(255,255,255),-1);

            vector<cv::Point2f> trying;
            for(int i = 0; i < 4; i ++)
            {
                trying.push_back(projectedMarkerPoints.at(i));
            }

            // if the first points are not close to each other rotate one set of points
            for(int i = 0; i < 4; i ++)
            {
                int radius = 40;
                int j = 0;
                while (sqrt(pow(abs(projectedMarkerPoints.at(i).x - imageCoord.at<cv::Point2f>(i).x),2) + pow(abs(projectedMarkerPoints.at(i).y - imageCoord.at<cv::Point2f>(i).y),2)) > radius && j < 4)
                {
                    projectedMarkerPoints.at(i) = trying.at(j);
                    j++;
                }
            }


            // compute error
            double error;
            for (int i = 0; i < 4; i++)
            {
                error += abs(projectedMarkerPoints.at(i).x - imageCoord.at<cv::Point2f>(i).x) + abs(projectedMarkerPoints.at(i).y - imageCoord.at<cv::Point2f>(i).y);
                cv::line(img, projectedMarkerPoints.at(i),imageCoord.at<cv::Point2f>(i), cv::Scalar(0,255,255), 2);

//                cv::circle(img,imageCoord.at<cv::Point2f>(i),5,cv::Scalar(50*i,50*i,50*i),-1);
//                cv::circle(img,projectedMarkerPoints.at(i),5,cv::Scalar(50*i,50*i,50*i),-1);
            }

            // store error
            errors.push_back(error);

            cout << "error: " << error << endl;
        }
    }

    // Loop through all errors to find the minimum then return the cooresponding T
    int minErrorIndex = 0;
    double minError = errors.at(0);
    for (int i = 0; i < errors.size(); i++)
    {
        if (errors.at(i) < minError)
        {
            minError = errors.at(i);
            minErrorIndex = i;
        }
    }
    cout << "minError: " << minError << endl;

    return allT.at(minErrorIndex);
}

// parameters N and M should be a Nx1 Matrix of cv::Point3f
Matrix4d SolveP3P::rigidTransform(cv::Mat N, cv::Mat M)
{
//    if(DEBUG) cout << "N: " << N << endl;
//    if(DEBUG) cout << "M: " << M << endl;

    // find centroid of n and m
    float x = 0;
    float y = 0;
    float z = 0;
    float  num = N.rows; // number of points, should be the same for both

    if (N.rows != M.rows)
    {
        if(DEBUG) cout << "Error: Number of elements should match" << endl;
    }
    for (int i = 0; i < num; i++)
    {
        x = x + N.at<cv::Point3f>(i).x;
        y = y + N.at<cv::Point3f>(i).y;
        z = z + N.at<cv::Point3f>(i).z;
    }
    cv::Mat nCenter = cv::Mat(3,1,cv::DataType<float>::type);
    nCenter.at<float>(0) = x/num;
    nCenter.at<float>(1) = y/num;
    nCenter.at<float>(2) = z/num;
    x = 0;
    y = 0;
    z = 0;
    for (int i = 0; i < num; i++)
    {
        x = x + M.at<cv::Point3f>(i).x;
        y = y + M.at<cv::Point3f>(i).y;
        z = z + M.at<cv::Point3f>(i).z;
    }
    cv::Mat mCenter = cv::Mat(3,1,cv::DataType<float>::type);
    mCenter.at<float>(0) = x/num;
    mCenter.at<float>(1) = y/num;
    mCenter.at<float>(2) = z/num;

    //********
    // overwrite nCenter and mCenter to be the middle point as opposed to the centroid of all three points
    nCenter.at<float>(0) = N.at<cv::Point3f>(1).x;
    nCenter.at<float>(1) = N.at<cv::Point3f>(1).y;
    nCenter.at<float>(2) = N.at<cv::Point3f>(1).z;
    mCenter.at<float>(0) = M.at<cv::Point3f>(1).x;
    mCenter.at<float>(1) = M.at<cv::Point3f>(1).y;
    mCenter.at<float>(2) = M.at<cv::Point3f>(1).z;
    // *****

//    if(DEBUG) cout << "nCenter: " << nCenter << endl;
//    if(DEBUG) cout << "mCenter: " << mCenter << endl;

    // convert cv::Mat of Point3f to vector of cv::Mat for matrix manipulation
//    cv::Mat H = cv::Mat(3,3,cv::DataType<float>::type, 0);
    cv::Mat H = cv::Mat::zeros(3,3,cv::DataType<float>::type);
//    if(DEBUG) cout << "H: " << H << endl;
    for (int i = 0; i < num; i++)
    {
        cv::Mat nMat = cv::Mat(3,1,cv::DataType<float>::type);
        nMat.at<float>(0) = N.at<cv::Point3f>(i).x - nCenter.at<float>(0);
        nMat.at<float>(1) = N.at<cv::Point3f>(i).y - nCenter.at<float>(1);
        nMat.at<float>(2) = N.at<cv::Point3f>(i).z - nCenter.at<float>(2);

        cv::Mat mMat = cv::Mat(3,1,cv::DataType<float>::type);
        cv::Mat mMatT = cv::Mat(1,3,cv::DataType<float>::type);
        mMat.at<float>(0) = M.at<cv::Point3f>(i).x - mCenter.at<float>(0);
        mMat.at<float>(1) = M.at<cv::Point3f>(i).y - mCenter.at<float>(1);
        mMat.at<float>(2) = M.at<cv::Point3f>(i).z - mCenter.at<float>(2);
        cv::transpose(mMat, mMatT);

        cv::Mat h = cv::Mat(3,3,cv::DataType<cv::Point3f>::type);
        h = nMat * mMatT;
        cv::add(H,h,H);
    }
//    if(DEBUG) cout << "H: " << H << endl;

    cv::Mat w, u, vT;
    cv::SVD::compute(H,w,u,vT,cv::SVD::MODIFY_A);
    cv::Mat R = cv::Mat(3,3,cv::DataType<float>::type);
    cv::Mat uT;
    cv::transpose(u,uT);
    cv::Mat v;
    cv::transpose(vT,v);

    R = v*uT;

    if (cv::determinant(R) < 0)
    {
        if(DEBUG) cout << "reflection detected" << endl;
        R.at<float>(0,2) = -R.at<float>(0,2);
        R.at<float>(1,2) = -R.at<float>(1,2);
        R.at<float>(2,2) = -R.at<float>(2,2);
    }

//    if(DEBUG) cout << "vT: " << vT << endl;
//    if(DEBUG) cout << "u: " << u << endl;
//    if(DEBUG) cout << "uT: " << uT << endl;

    cv::Mat t;
    cv::add(-R*nCenter, mCenter, t);

//    if(DEBUG) cout << "R: " << R << endl;
//    if(DEBUG) cout << "t; " << t << endl;

    // use eigen to convert to T
    MatrixXd rMat(3,3);
    rMat = cvMatToEigen(R,3,3);
    MatrixXd tVec(3,1);
    tVec = cvMatToEigen(t,3,1);
    Matrix4d Tmat;
    Tmat.topLeftCorner(3,3) = rMat;
    Tmat.topRightCorner(3,1) = tVec;
    Tmat.bottomLeftCorner(1,4) << 0,0,0,1;

//    if(DEBUG) cout << "Tmat: " << Tmat << endl;

    cv::Mat T = cv::Mat(3,3,cv::DataType<double>::type);
    T = eigenToCvMat(Tmat,4,4);

    return Tmat;
}

// careful this takes cv::Mat of floats
MatrixXd SolveP3P::cvMatToEigen(cv::Mat input, int rows, int cols)
{
    MatrixXd output(rows, cols);
    for (int r = 0; r < rows; r++)
    {
        for (int c = 0; c < cols; c++)
        {
            output(r,c) = (double)input.at<float>(r,c);
        }
    }
    return output;
}

cv::Mat SolveP3P::eigenToCvMat(MatrixXd input, int rows, int cols)
{
    cv::Mat output = cv::Mat(rows,cols,cv::DataType<double>::type);
    for (int r = 0; r < rows; r++)
    {
        for (int c = 0; c < cols; c++)
        {
            output.at<double>(r,c) = (double)input(r,c);
        }
    }
    return output;
}

cv::Mat SolveP3P::rvecFromT(Matrix4d T)
{
    MatrixXd R(3,3);
    R = T.topLeftCorner(3,3);
    cv::Mat rmat = cv::Mat(3,3,cv::DataType<double>::type);
    rmat = eigenToCvMat(R,3,3);
    cv::Mat rvec = cv::Mat(3,1,cv::DataType<double>::type);
    cv::Rodrigues(rmat, rvec);

    return rvec;
}

cv::Mat SolveP3P::tvecFromT(Matrix4d T)
{
    MatrixXd t(3,1);
    t = T.topRightCorner(3,1);
    cv::Mat tvec = cv::Mat(3,1,cv::DataType<double>::type);
    tvec = eigenToCvMat(t,3,1);

    return tvec;
}



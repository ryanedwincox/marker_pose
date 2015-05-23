#include "solvep3p.h"

SolveP3P::SolveP3P()
{

}

Matrix4d SolveP3P::solveP3P(cv::Mat worldCoord, cv::Mat imageCoord, cv::Mat cameraMatrix, cv::Mat distCoeffs, cv::Mat rvec, cv::Mat tvec)
{
    // only use three image coord, and the forth for finding the best solution
    cv::Mat threeImageCoord = cv::Mat(3,1,cv::DataType<cv::Point2f>::type);
    threeImageCoord.at<cv::Point2f>(0) = imageCoord.at<cv::Point2f>(0);
    threeImageCoord.at<cv::Point2f>(1) = imageCoord.at<cv::Point2f>(1);
    threeImageCoord.at<cv::Point2f>(2) = imageCoord.at<cv::Point2f>(2);
//    threeImageCoord.at<cv::Point2f>(0) = (cv::Point2f){270,153};
//    threeImageCoord.at<cv::Point2f>(1) = (cv::Point2f){320,240};
//    threeImageCoord.at<cv::Point2f>(2) = (cv::Point2f){420,240};
//    threeImageCoord.at<cv::Point2f>(0) = (cv::Point2f){320,140};
//    threeImageCoord.at<cv::Point2f>(1) = (cv::Point2f){320,240};
//    threeImageCoord.at<cv::Point2f>(2) = (cv::Point2f){420,240};

    // only use three world coord, and the forth for finding the best solution
    cv::Mat threeWorldCoord = cv::Mat(3,1,cv::DataType<cv::Point3f>::type);
    threeWorldCoord.at<cv::Point3f>(0) = worldCoord.at<cv::Point3f>(0);
    threeWorldCoord.at<cv::Point3f>(1) = worldCoord.at<cv::Point3f>(1);
    threeWorldCoord.at<cv::Point3f>(2) = worldCoord.at<cv::Point3f>(2);
//    threeWorldCoord.at<cv::Point3f>(0) = (cv::Point3f){-0.05,0.0866,0};
//    threeWorldCoord.at<cv::Point3f>(1) = (cv::Point3f){0,0,0};
//    threeWorldCoord.at<cv::Point3f>(2) = (cv::Point3f){0.1,0,0};
//    threeWorldCoord.at<cv::Point3f>(0) = (cv::Point3f){0,0.1,0};
//    threeWorldCoord.at<cv::Point3f>(1) = (cv::Point3f){0,0,0};
//    threeWorldCoord.at<cv::Point3f>(2) = (cv::Point3f){0.1,0,0};
//    cout << "3worldCoord: " << threeWorldCoord << endl;

    normalizeImagePoints(threeImageCoord, cameraMatrix, distCoeffs);
    setUpP3PEquationSystem(threeWorldCoord);

    cv::Mat estimatedWorldCoord = cv::Mat(3,1,cv::DataType<cv::Point3f>::type);
    estimatedWorldCoord = solveP3PEquationSystem();
//    cout << "estimatedWorldCoord: " << estimatedWorldCoord << endl;

    Matrix4d T = rigidTransform(threeWorldCoord, estimatedWorldCoord);

//    cout << "T: " << T << endl;
    rvec = rvecFromT(T);
    tvec = tvecFromT(T);

    return T;
}

// parameters:
// imageCoord: cv::Mat of cv::Point2f
// size: number of points in imageCoord
void SolveP3P::normalizeImagePoints(cv::Mat imageCoord, cv::Mat cameraMatrix, cv::Mat distCoeffs)
{
    // set camera parameters
    this->cameraMatrix = cameraMatrix;
    this->distCoeffs = distCoeffs;
    fx = cameraMatrix.at<double>(0,0);
    fy = cameraMatrix.at<double>(1,1);
    cx = cameraMatrix.at<double>(0,2);
    cy = cameraMatrix.at<double>(1,2);

    int size = *imageCoord.size;

    normalizedCoord = cv::Mat(size,1,cv::DataType<cv::Point3f>::type);
    for (int i = 0; i < size; i++)
    {
        double x = (imageCoord.at<cv::Point2f>(i).x - cx) / fx;
        double y = (imageCoord.at<cv::Point2f>(i).y - cy) / fy;
        double z = 1;

        double N = sqrt(x*x + y*y + z*z);

        normalizedCoord.at<cv::Point3f>(i).x = (float) x / N;
        normalizedCoord.at<cv::Point3f>(i).y = (float) y / N;
        normalizedCoord.at<cv::Point3f>(i).z = (float) z / N;

//        cout << "ImageCoord: " << imageCoord.at<cv::Point2f>(i).x << ", " << imageCoord.at<cv::Point2f>(i).y << endl;
//        cout << "normalizedCoord: " << normalizedCoord.at<cv::Point3f>(i).x << ", " << normalizedCoord.at<cv::Point3f>(i).y << ", " << normalizedCoord.at<cv::Point3f>(i).z << endl;
    }
}

void SolveP3P::setUpP3PEquationSystem(cv::Mat threeWorldCoord)
{
    // TODO can't assume right triangles, must use law of cosines! **********************
    // calculate angles to points on image plane
    u = normalizedCoord.at<cv::Point3f>(0);
    v = normalizedCoord.at<cv::Point3f>(1);
    w = normalizedCoord.at<cv::Point3f>(2);
    uv = acos(u.x*v.x + u.y*v.y + u.z*v.z);
    uw = acos(u.x*w.x + u.y*w.y + u.z*w.z);
    vw = acos(v.x*w.x + v.y*w.y + v.z*w.z);
//    uv = 0.197;
//    uw = 0.34;
//    vw = 0.197;

    cout << "angles: " << uv << ", " << uw << ", " << vw << endl;

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

    cout << "AB, BC, CA: " << AB << ", " << BC << ", " << CA << endl;

    //calculate coefficients
    aa = (BC*BC) / (AB*AB);
    bb = (CA*CA) / (AB*AB);

    cout << "aa bb : " << aa << ", " << bb << endl;
}

cv::Mat SolveP3P::solveP3PEquationSystem()
{
    // use Wu Ritt's zero composition to get coefficients a4, a3, a2, a1, a0, b1 and b0
    double p = 2*cos(vw);
    double q = 2*cos(uw);
    double r = 2*cos(uv);

    cout << "pqr: " << p << ", " << q << ", " << r << endl;

    // a4*x^4+a3*x^3+a2*x^2+a1*x^1+a0=0
//    a4 = pow(aa,2) + pow(bb,2) - 2*aa - 2*bb + 2*(1-pow(r,2))*bb*aa + 1;
//    a3 = -2*q*pow(aa,2) - r*p*pow(bb,2) + 4*q*aa + (2*q+p*r)*bb + (pow(r,2)*q-2*q+r*p)*aa*bb - 2*q;
//    a2 = (2+pow(q,2))*pow(aa,2) + (pow(p,2)+pow(r,2)-2)*pow(bb,2) - (4+2*pow(q,2))*aa - (p*q*r+pow(p,2))*bb - (p*q*r+pow(r,2))*aa*bb + pow(q,2) + 2;
//    a1 = -2*q*pow(aa,2) - r*p*pow(bb,2) + 4*q*aa + (p*r+q*pow(p,2)-2*q)*bb + (r*p+2*q)*aa*bb - 2*q;
//    a0 = pow(aa,2)+pow(bb,2) - 2*aa + (2-pow(p,2))*bb - 2*aa*bb + 1;
//    cout << "a4: " << a4 << endl;
//    cout << "a3: " << a3 << endl;
//    cout << "a2: " << a2 << endl;
//    cout << "a1: " << a1 << endl;
//    cout << "a0: " << a0 << endl;

    a4 = -2*bb + pow(bb,2) + pow(aa,2) + 1 - bb*pow(r,2)*aa +  2*bb*aa - 2*aa;
    a3 = -2*bb*q*aa - 2*pow(aa,2)*q + bb*pow(r,2)*q*aa - 2*q + 2*bb*q + 4*aa*q + p*bb*r + bb*r*p*aa - pow(bb,2)*r*p;
    a2 = pow(q,2) + pow(bb,2)*pow(r,2) - bb*pow(p,2) - q*p*bb*r + pow(bb,2)*pow(p,2) - bb*pow(r,2)*aa +
            2 - 2*pow(bb,2) - aa*bb*r*p*q + 2*pow(aa,2) - 4*aa - 2*pow(q,2)*aa + pow(q,2)*pow(aa,2);
    a1 = -pow(bb,2)*r*p + bb*r*p*aa - 2*pow(aa,2)*q + q*pow(p,2)*bb + 2*bb*q*aa + 4*aa*q + p*bb*r - 2*bb*q - 2*q;
    a0 = 1 - 2*aa + 2*bb + pow(bb,2) - bb*pow(p,2) + pow(aa,2) - 2*bb*aa;
    cout << "a4: " << a4 << endl;
    cout << "a3: " << a3 << endl;
    cout << "a2: " << a2 << endl;
    cout << "a1: " << a1 << endl;
    cout << "a0: " << a0 << endl;

    // SolveP4 return value
    // return 4: 4 real roots x[0], x[1], x[2], x[3], possible multiple roots
    // return 2: 2 real roots x[0], x[1] and complex x[2]i*x[3],
    // return 0: two pair of complex roots: x[0]i*x[1],  x[2]i*x[3],
    double X [4];
    int ret = SolveP4(X, a3/a4, a2/a4, a1/a4, a0/a4);
    cout << "ret: " << ret << endl;
    cout << "X: " << X[0] << ", " << X[1] << ", " << X[2] << ", " << X[3] << endl;

    double x = X[0];
//    x = 1;
    cout << "x: " << x << endl;

//    // b1*y - b0 = 0
//    b1 = bb*pow(((pow(p,2)-p*q*r+pow(r,2))*aa+(pow(p,2)-pow(r,2))*bb-pow(p,2)+p*q*r-pow(r,2)),2);
//    b0 = ((1-aa-bb)*pow(x,2)+(aa-1)*q*x-aa+bb+1)*
//            (pow(r,3)*(pow(aa,2)+pow(bb,2)-2*aa-2*bb+(2-pow(r,2))*aa*bb+1)*pow(x,3)+
//            pow(r,2)*(p+p*pow(aa,2)-2*r*q*aa*bb+2*r*q*bb-2*r*q-2*p*aa-2*p*bb+p*pow(r,2)*bb+4*r*q*aa+q*pow(r,3)*aa*bb-2*r*q*pow(aa,2)+2*p*aa*bb+p*pow(bb,2)-pow(r,2)*p*pow(bb,2))*pow(x,2)+
//            (pow(r,5)*(pow(bb,2)-aa*bb)-pow(r,4)*p*q*bb+pow(r,3)*(pow(q,2)-4*aa-2*pow(q,2)*aa+pow(q,2)*pow(aa,2)+2*pow(aa,2)-2*pow(bb,2)+2)+pow(r,2)*(4*p*q*aa-2*p*q*aa*bb+2*p*q*bb-2*p*q-2*p*q*pow(aa,2)+
//            r*(pow(p,2)*pow(bb,2)-2*pow(p,2)*bb+2*pow(p,2)*aa*bb-2*pow(p,2)*aa+pow(p,2)+pow(p,2)*pow(aa,2)))*x+
//            (2*p*pow(r,2)-2*pow(r,3)*q+pow(p,3)-2*pow(p,2)*q*r+p*pow(q,2)*pow(r,2))*pow(aa,2)+(pow(p,3)-2*p*pow(r,2))*pow(bb,2)+(4*q*pow(r,3)-4*p*pow(r,2)-2*pow(p,3)+4*pow(p,2)*q*r-2*p*pow(q,2)*pow(r,2))*aa+
//            (-2*q*pow(r,3)+p*pow(r,4)+2*pow(p,2)*q*r-2*pow(p,3))*bb+(2*pow(p,3)+2*q*pow(r,3)-2*pow(p,2)*q*r)*aa*bb+
//            p*pow(q,2)*pow(r,2)-2*pow(p,2)*q*r+2*p*pow(r,2)+pow(p,3)-2*pow(r,3)*q));

//    cout << "b1: " << b1 << endl;
//    cout << "b0: " << b0 << endl;

    b1 = bb*pow((pow(p,2)*aa - pow(p,2) + bb*pow(p,2) + p*q*r - q*aa*r*p + aa*pow(r,2) - pow(r,2) - bb*pow(r,2)),2);
    b0 = ((1-aa-bb)*pow(x,2) + (q*aa-q)*x + 1 - aa + bb)*((pow(aa,2)*pow(r,3) + 2*bb*pow(r,3)*aa - bb*pow(r,5)*aa - 2*aa*pow(r,3) + pow(r,3) + pow(bb,2)*pow(r,3) - 2*pow(r,3)*bb)*pow(x,3) +
            (p*pow(r,2) + p*pow(aa,2)*pow(r,2) - 2*bb*pow(r,3)*q*aa + 2*pow(r,3)*bb*q + 2*pow(r,3)*q - 2*p*aa*pow(r,2) - 2*p*pow(r,2)*bb + pow(r,4)*p*bb + 4*aa*pow(r,3)*q + bb*q*aa*pow(r,5) - 2*pow(r,3)*pow(aa,2)*q +
             2*pow(r,2)*p*bb*aa + pow(bb,2)*pow(r,2)*p - pow(r,4)*p*pow(bb,2))*pow(x,2) + (pow(r,3)*pow(q,2) + pow(r,5)*pow(bb,2) + r*pow(p,2)*pow(bb,2) - 4*aa*pow(r,3) - 2*aa*pow(r,3)*pow(q,2) + pow(r,3)*pow(q,2)*pow(aa,2) +
             2*pow(aa,2)*pow(r,3) - 2*pow(bb,2)*pow(r,3) - 2*pow(p,2)*bb*r + 4*p*aa*pow(r,2)*q + 2*aa*pow(p,2)*r*bb - 2*aa*pow(r,2)*q*bb*p - 2*pow(p,2)*aa*r + r*pow(p,2) - bb*pow(r,5)*aa + 2*p*pow(r,2)*bb*q +
             r*pow(p,2)*pow(aa,2) - 2*p*q*pow(r,2) + 2*pow(r,3) - 2*pow(r,2)*p*pow(aa,2)*q - pow(r,4)*q*bb*p)*x + 4*aa*pow(r,3)*q + p*pow(r,2)*pow(q,2) + 2*pow(p,3)*bb*aa - 4*p*aa*pow(r,2) +
             -2*pow(r,3)*bb*q - 2*pow(p,2)*q*r - 2*pow(bb,2)*pow(r,2)*p + pow(r,4)*p*bb + 2*p*pow(aa,2)*pow(r,2) - 2*pow(r,3)*pow(aa,2)*q - 2*pow(p,3)*aa + pow(p,3)*pow(aa,2) + 2+p+pow(r,2) + pow(p,3) + 2*bb*pow(r,3)*q*aa +
             2*q*pow(p,2)*bb*r + 4*q*aa*r*pow(p,2) - 2*p*aa*pow(r,2)*pow(q,2) - 2*pow(p,2)*pow(aa,2)*r*q + p*pow(aa,2)*pow(r,2)*pow(q,2) - 2*pow(r,3)*q - 2*pow(p,3)*bb + pow(p,3)*pow(bb,2) - 2*pow(p,2)*bb*r*q*aa);

    cout << "b1: " << b1 << endl;
    cout << "b0: " << b0 << endl;

    // Solve quadratic for y
    double j = 1-aa;
    double k = 2*aa*x*cos(uv) - 2*cos(vw);
    double l = 1-aa*pow(x,2);
    cout << "jkl: " << j << ", " << k << ", " << l << endl;
    cout << "sqrt: " << pow(k,2) - 4*j*l << endl;

    double y0 = -l/k;
    double y1 = (-k + sqrt(pow(k,2) - 4*j*l)) / (2*j);
    double y2 = (-k - sqrt(pow(k,2) - 4*j*l)) / (2*j);
    cout << "y0: " << y0 << endl;
    cout << "y1: " << y1 << endl;
    cout << "y2: " << y2 << endl;

    double y = b0/b1;
    y = 0.98;
    y = y2;
    cout << "y: " << y << endl;

    // plug into P3P equation system to check x and y, should equal 0
    double eq0 = (1-aa)*pow(y,2) - aa*pow(x,2) - 2*cos(vw)*y + 2*aa*cos(uv)*x*y + 1;
    double eq1 = (1-bb)*pow(x,2) - bb*pow(y,2) - 2*cos(uw)*x + 2*bb*cos(uv)*x*y + 1;
    cout << "eq0: " << eq0 << endl;
    cout << "eq1: " << eq1 << endl;

    double V = pow(x,2) + pow(y,2) - 2*x*y*cos(uv);
//    PC = sqrt(pow(AB,2)/V);  // using v = AB2/PC2
    double markerSpacing = 0.105;
    PC = sqrt(pow(markerSpacing,2)/V);  // using v = AB2/PC2
    PB = y*PC; // using y = PB/PC
    PA = x*PC; // using x = PA/PC

//    PC = 0.51;
//    PB = 0.5;
//    PA = 0.51;

    cout << "PA: " << PA << endl;
    cout << "PB: " << PB << endl;
    cout << "PC: " << PC << endl;

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

    cout << "estimated world coord:" << endl;
    cout << "AA: " << AA << endl;
    cout << "BB: " << BB << endl;
    cout << "CC: " << CC << endl;

    // return vector of points
    cv::Mat estimatedWorldPoints = cv::Mat(3,1,cv::DataType<cv::Point3f>::type);
    estimatedWorldPoints.at<cv::Point3f>(0) = AA;
    estimatedWorldPoints.at<cv::Point3f>(1) = BB;
    estimatedWorldPoints.at<cv::Point3f>(2) = CC;

    return estimatedWorldPoints;
}

// parameters N and M should be a Nx1 Matrix of cv::Point3f
Matrix4d SolveP3P::rigidTransform(cv::Mat N, cv::Mat M)
{
//    cout << "N: " << N << endl;
//    cout << "M: " << M << endl;

    // find centroid of n and m
    float x = 0;
    float y = 0;
    float z = 0;
    float  num = N.rows; // number of points, should be the same for both

    if (N.rows != M.rows)
    {
        cout << "Error: Number of elements should match" << endl;
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
    // overwrite nCenter and mCenter to be the middle point
    nCenter.at<float>(0) = N.at<cv::Point3f>(1).x;
    nCenter.at<float>(1) = N.at<cv::Point3f>(1).y;
    nCenter.at<float>(2) = N.at<cv::Point3f>(1).z;
    mCenter.at<float>(0) = M.at<cv::Point3f>(1).x;
    mCenter.at<float>(1) = M.at<cv::Point3f>(1).y;
    mCenter.at<float>(2) = M.at<cv::Point3f>(1).z;
    // *****

//    cout << "nCenter: " << nCenter << endl;
//    cout << "mCenter: " << mCenter << endl;

    // convert cv::Mat of Point3f to vector of cv::Mat for matrix manipulation
//    cv::Mat H = cv::Mat(3,3,cv::DataType<float>::type, 0);
    cv::Mat H = cv::Mat::zeros(3,3,cv::DataType<float>::type);
//    cout << "H: " << H << endl;
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
//    cout << "H: " << H << endl;

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
        cout << "reflection detected" << endl;
        R.at<float>(0,2) = -R.at<float>(0,2);
        R.at<float>(1,2) = -R.at<float>(1,2);
        R.at<float>(2,2) = -R.at<float>(2,2);
    }

//    cout << "vT: " << vT << endl;
//    cout << "u: " << u << endl;
//    cout << "uT: " << uT << endl;

    cv::Mat t;
    cv::add(-R*nCenter, mCenter, t);

//    cout << "R: " << R << endl;
//    cout << "t; " << t << endl;

    // use eigen to convert to T
    MatrixXd rMat(3,3);
    rMat = cvMatToEigen(R,3,3);
    MatrixXd tVec(3,1);
    tVec = cvMatToEigen(t,3,1);
    Matrix4d Tmat;
    Tmat.topLeftCorner(3,3) = rMat;
    Tmat.topRightCorner(3,1) = tVec;
    Tmat.bottomLeftCorner(1,4) << 0,0,0,1;

//    cout << "Tmat: " << Tmat << endl;

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



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
//    threeImageCoord.at<cv::Point2f>(0) = (cv::Point2f){200,200};
//    threeImageCoord.at<cv::Point2f>(1) = (cv::Point2f){200,300};
//    threeImageCoord.at<cv::Point2f>(2) = (cv::Point2f){300,300};

    normalizeImagePoints(threeImageCoord, cameraMatrix, distCoeffs);
    setUpP3PEquationSystem();

    cv::Mat estimatedWorldCoord = cv::Mat(3,1,cv::DataType<cv::Point3f>::type);
    estimatedWorldCoord = solveP3PEquationSystem();
    cout << "estimatedWorldCoord: " << estimatedWorldCoord << endl;

    // only use three world coord, and the forth for finding the best solution
    cv::Mat threeWorldCoord = cv::Mat(3,1,cv::DataType<cv::Point3f>::type);
    threeWorldCoord.at<cv::Point3f>(0) = worldCoord.at<cv::Point3f>(0);
    threeWorldCoord.at<cv::Point3f>(1) = worldCoord.at<cv::Point3f>(1);
    threeWorldCoord.at<cv::Point3f>(2) = worldCoord.at<cv::Point3f>(2);
//    threeWorldCoord.at<cv::Point3f>(0) = (cv::Point3f){0,0.1,0};
//    threeWorldCoord.at<cv::Point3f>(1) = (cv::Point3f){0,0,0};
//    threeWorldCoord.at<cv::Point3f>(2) = (cv::Point3f){0.1,0,0};
//    cout << "3worldCoord: " << threeWorldCoord << endl;

    Matrix4d T = rigidTransform(threeWorldCoord, estimatedWorldCoord);

    cout << "T: " << T << endl;
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

void SolveP3P::setUpP3PEquationSystem()
{
    // calculate angles to points on image plane
    u = normalizedCoord.at<cv::Point3f>(0);
    v = normalizedCoord.at<cv::Point3f>(1);
    w = normalizedCoord.at<cv::Point3f>(2);
    uv = acos(u.x*v.x + u.y*v.y + u.z*v.z);
    uw = acos(u.x*w.x + u.y*w.y + u.z*w.z);
    vw = acos(v.x*w.x + v.y*w.y + v.z*w.z);

    // calculate distances AB, BC, and CA
    AB = sqrt((v.x-u.x)*(v.x-u.x)+(v.y-u.y)*(v.y-u.y)+(v.z-u.z)*(v.z-u.z));
    BC = sqrt((w.x-v.x)*(w.x-v.x)+(w.y-v.y)*(w.y-v.y)+(w.z-v.z)*(w.z-v.z));
    CA = sqrt((w.x-u.x)*(w.x-u.x)+(w.y-u.y)*(w.y-u.y)+(w.z-u.z)*(w.z-u.z));

    //calculate coefficients
    aa = (BC*BC) / (AB*AB);
    bb = (CA*CA) / (AB*AB);
}

cv::Mat SolveP3P::solveP3PEquationSystem()
{
    // use Wu Ritt's zero composition to get coefficients a4, a3, a2, a1, a0, b1 and b0
    double p = 2*cos(vw);
    double q = 2*cos(uw);
    double r = 2*cos(uv);

    // a4*x^4+a3*x^3+a2*x^2+a1*x^1+a0=0
    a4 = pow(aa,2) + pow(bb,2) - 2*aa - 2*bb + 2*(1-pow(r,2))*bb*aa + 1;
    a3 = -2*q*pow(aa,2) - r*p*pow(bb,2) + 4*q*aa + (2*q+p*r)*bb + (pow(r,2)*q-2*q+r*p)*aa*bb - 2*q;
    a2 = (2+pow(q,2))*pow(aa,2) + (pow(p,2)+pow(r,2)-2)*pow(bb,2) - (4+2*pow(q,2))*aa - (p*q*r+pow(p,2))*bb - (p*q*r+pow(r,2))*aa*bb + pow(q,2) + 2;
    a1 = -2*q*pow(aa,2) - r*p*pow(bb,2) + 4*q*aa + (p*r+q*pow(p,2)-2*q)*bb + (r*p+2*q)*aa*bb - 2*q;
    a0 = pow(aa,2)+pow(bb,2) - 2*aa + (2-pow(p,2))*bb - 2*aa*bb + 1;
//    cout << "a4: " << a_4 << endl;
//    cout << "a3: " << a_3 << endl;
//    cout << "a2: " << a_2 << endl;
//    cout << "a1: " << a_1 << endl;
//    cout << "a0: " << a_0 << endl;

    double X [4];
    int ret = SolveP4(X, a3/a4, a2/a4, a1/a4, a0/a4);
    cout << "ret: " << ret << endl;

    double x = X[0];

    // b1*y - b0 = 0
    b1 = bb*pow(((pow(p,2)-p*q*r+pow(r,2))*aa+(pow(p,2)-pow(r,2))*bb-pow(p,2)+p*q*r-pow(r,2)),2);
    b0 = ((1-aa-bb)*pow(x,2)+(aa-1)*q*x-aa+bb+1)*
            (pow(r,3)*(pow(aa,2)+pow(bb,2)-2*aa-2*bb+(2-pow(r,2))*aa*bb+1)*pow(x,3)+
            pow(r,2)*(p+p*pow(aa,2)-2*r*q*aa*bb+2*r*q*bb-2*r*q-2*p*aa-2*p*bb+p*pow(r,2)*bb+4*r*q*aa+q*pow(r,3)*aa*bb-2*r*q*pow(aa,2)+2*p*aa*bb+p*pow(bb,2)-pow(r,2)*p*pow(bb,2))*pow(x,2)+
            (pow(r,5)*(pow(bb,2)-aa*bb)-pow(r,4)*p*q*bb+pow(r,3)*(pow(q,2)-4*aa-2*pow(q,2)*aa+pow(q,2)*pow(aa,2)+2*pow(aa,2)-2*pow(bb,2)+2)+pow(r,2)*(4*p*q*aa-2*p*q*aa*bb+2*p*q*bb-2*p*q-2*p*q*pow(aa,2)+
            r*(pow(p,2)*pow(bb,2)-2*pow(p,2)*bb+2*pow(p,2)*aa*bb-2*pow(p,2)*aa+pow(p,2)+pow(p,2)*pow(aa,2)))*x+
            (2*p*pow(r,2)-2*pow(r,3)*q+pow(p,3)-2*pow(p,2)*q*r+p*pow(q,2)*pow(r,2))*pow(aa,2)+(pow(p,3)-2*p*pow(r,2))*pow(bb,2)+(4*q*pow(r,3)-4*p*pow(r,2)-2*pow(p,3)+4*pow(p,2)*q*r-2*p*pow(q,2)*pow(r,2))*aa+
            (-2*q*pow(r,3)+p*pow(r,4)+2*pow(p,2)*q*r-2*pow(p,3))*bb+(2*pow(p,3)+2*q*pow(r,3)-2*pow(p,2)*q*r)*aa*bb+
            p*pow(q,2)*pow(r,2)-2*pow(p,2)*q*r+2*p*pow(r,2)+pow(p,3)-2*pow(r,3)*q));

//    cout << "b1: " << b1 << endl;
//    cout << "b0: " << b0 << endl;

    double y = b0/b1;

    double V = pow(x,2) + pow(y,2) - 2*x*y*cos(uv);
    PC = sqrt(pow(AB,2)/V);  // using v = AB2/PC2
    PB = y*PC; // using y = PB/PC
    PA = x*PC; // using x = PA/PC

    PC = 0.51;
    PB = 0.5;
    PA = 0.51;

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



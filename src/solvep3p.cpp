#include "solvep3p.h"

SolveP3P::SolveP3P()
{

}

void SolveP3P::solveP3P(cv::Mat imageCoord, cv::Mat worldCoord, cv::Mat cameraMatrix, cv::Mat distCoeffs)
{
    normalizeImagePoints(imageCoord, cameraMatrix, distCoeffs);
    setUpP3PEquationSystem();
    cv::Mat M = cv::Mat(3,1,cv::DataType<cv::Point3f>::type);
    M = solveP3PEquationSystem();
    cv::Mat tempWorldCoord = cv::Mat(3,1,cv::DataType<cv::Point3f>::type);
    tempWorldCoord.at<cv::Point3f>(0) = worldCoord.at<cv::Point3f>(0);
    tempWorldCoord.at<cv::Point3f>(1) = worldCoord.at<cv::Point3f>(1);
    tempWorldCoord.at<cv::Point3f>(2) = worldCoord.at<cv::Point3f>(2);
    rigidTransform(tempWorldCoord,M);
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
    a = (BC*BC) / (AB*AB);
    b = (CA*CA) / (AB*AB);
}

cv::Mat SolveP3P::solveP3PEquationSystem()
{
    // use Wu Ritt's zero composition to get coefficients a4, a3, a2, a1, a0, b1 and b0
    double p = 2*cos(vw);
    double q = 2*cos(uw);
    double r = 2*cos(uv);

    // a4*x^4+a3*x^3+a2*x^2+a1*x^1+a0=0
    a4 = pow(a,2) + pow(b,2) - 2*a - 2*b + 2*(1-pow(r,2))*b*a + 1;
    a3 = -2*q*pow(a,2) - r*p*pow(b,2) + 4*q*a + (2*q+p*r)*b + (pow(r,2)*q-2*q+r*p)*a*b - 2*q;
    a2 = (2+pow(q,2))*pow(a,2) + (pow(p,2)+pow(r,2)-2)*pow(b,2) - (4+2*pow(q,2))*a - (p*q*r+pow(p,2))*b - (p*q*r+pow(r,2))*a*b + pow(q,2) + 2;
    a1 = -2*q*pow(a,2) - r*p*pow(b,2) + 4*q*a + (p*r+q*pow(p,2)-2*q)*b + (r*p+2*q)*a*b - 2*q;
    a0 = pow(a,2)+pow(b,2) - 2*a + (2-pow(p,2))*b - 2*a*b + 1;
//    cout << "a4: " << a4 << endl;
//    cout << "a3: " << a3 << endl;
//    cout << "a2: " << a2 << endl;
//    cout << "a1: " << a1 << endl;
//    cout << "a0: " << a0 << endl;

    double X [4];
    int ret = SolveP4(X, a3/a4, a2/a4, a1/a4, a0/a4);
//    cout << "ret: " << ret << endl;

    double x = X[0];

    // b1*y - b0 = 0
    b1 = b*pow(((pow(p,2)-p*q*r+pow(r,2))*a+(pow(p,2)-pow(r,2))*b-pow(p,2)+p*q*r-pow(r,2)),2);
    b0 = ((1-a-b)*pow(x,2)+(a-1)*q*x-a+b+1)*
            (pow(r,3)*(pow(a,2)+pow(b,2)-2*a-2*b+(2-pow(r,2))*a*b+1)*pow(x,3)+
            pow(r,2)*(p+p*pow(a,2)-2*r*q*a*b+2*r*q*b-2*r*q-2*p*a-2*p*b+p*pow(r,2)*b+4*r*q*a+q*pow(r,3)*a*b-2*r*q*pow(a,2)+2*p*a*b+p*pow(b,2)-pow(r,2)*p*pow(b,2))*pow(x,2)+
            (pow(r,5)*(pow(b,2)-a*b)-pow(r,4)*p*q*b+pow(r,3)*(pow(q,2)-4*a-2*pow(q,2)*a+pow(q,2)*pow(a,2)+2*pow(a,2)-2*pow(b,2)+2)+pow(r,2)*(4*p*q*a-2*p*q*a*b+2*p*q*b-2*p*q-2*p*q*pow(a,2)+
            r*(pow(p,2)*pow(b,2)-2*pow(p,2)*b+2*pow(p,2)*a*b-2*pow(p,2)*a+pow(p,2)+pow(p,2)*pow(a,2)))*x+
            (2*p*pow(r,2)-2*pow(r,3)*q+pow(p,3)-2*pow(p,2)*q*r+p*pow(q,2)*pow(r,2))*pow(a,2)+(pow(p,3)-2*p*pow(r,2))*pow(b,2)+(4*q*pow(r,3)-4*p*pow(r,2)-2*pow(p,3)+4*pow(p,2)*q*r-2*p*pow(q,2)*pow(r,2))*a+
            (-2*q*pow(r,3)+p*pow(r,4)+2*pow(p,2)*q*r-2*pow(p,3))*b+(2*pow(p,3)+2*q*pow(r,3)-2*pow(p,2)*q*r)*a*b+
            p*pow(q,2)*pow(r,2)-2*pow(p,2)*q*r+2*p*pow(r,2)+pow(p,3)-2*pow(r,3)*q));

//    cout << "b1: " << b1 << endl;
//    cout << "b0: " << b0 << endl;

    double y = b0/b1;

    double V = pow(x,2) + pow(y,2) - 2*x*y*cos(uv);
    PC = sqrt(pow(AB,2)/V);  // using v = AB2/PC2
    PB = y*PC; // using y = PB/PC
    PA = x*PC; // using x = PA/PC

//    cout << "PA: " << PA << endl;
//    cout << "PB: " << PB << endl;
//    cout << "PC: " << PC << endl;

    // scale points to find estimated projected points
    A.x = u.x * PA;
    A.y = u.y * PA;
    A.z = u.z * PA;
    B.x = v.x * PB;
    B.y = v.y * PB;
    B.z = v.z * PB;
    C.x = w.x * PC;
    C.y = w.y * PC;
    C.z = w.z * PC;

    // ************
    // Temp for testing rigidTransform
    A = (cv::Point3f){1,1,1};
    B = (cv::Point3f){1,3,1};
    C = (cv::Point3f){3,3,1};
    // ***********

    // return vector of points
    cv::Mat estimatedWorldPoints = cv::Mat(3,1,cv::DataType<cv::Point3f>::type);
    estimatedWorldPoints.at<cv::Point3f>(0) = A;
    estimatedWorldPoints.at<cv::Point3f>(1) = B;
    estimatedWorldPoints.at<cv::Point3f>(2) = C;

    return estimatedWorldPoints;
}

void SolveP3P::rigidTransform(cv::Mat N, cv::Mat M)
{
    // find centroid of n and m
    float x = 0;
    float y = 0;
    float z = 0;
    float  num = N.rows; // number of points, should be the same for both
    cout << "rows: " << num << endl;
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

    cout << "nCenter: " << nCenter << endl;
    cout << "mCenter: " << mCenter << endl;

    // convert cv::Mat of Point3f to vector of cv::Mat for matrix manipulation
    vector<cv::Mat> nTemp = vector<cv::Mat> (num);
    vector<cv::Mat> mTemp = vector<cv::Mat> (num);
    cv::Mat H = cv::Mat(3,3,cv::DataType<float>::type);
    for (int i = 0; i < num; i++)
    {
        cv::Mat nMat = cv::Mat(3,1,cv::DataType<float>::type);
        nMat.at<float>(0,0) = N.at<cv::Point3f>(i).x - nCenter.at<float>(0,0);
        nMat.at<float>(1,0) = N.at<cv::Point3f>(i).y - nCenter.at<float>(0,1);
        nMat.at<float>(2,0) = N.at<cv::Point3f>(i).z - nCenter.at<float>(0,2);
        nTemp[i] = nMat;

        cv::Mat mMat = cv::Mat(3,1,cv::DataType<float>::type);
        cv::Mat mMatT = cv::Mat(1,3,cv::DataType<float>::type);
        mMat.at<float>(0,0) = M.at<cv::Point3f>(i).x - mCenter.at<float>(0,0);
        mMat.at<float>(1,0) = M.at<cv::Point3f>(i).y - mCenter.at<float>(0,1);
        mMat.at<float>(2,0) = M.at<cv::Point3f>(i).z - mCenter.at<float>(0,2);
        cv::transpose(mMat, mMatT);
        mTemp[i] = mMatT;

        cv::Mat h = cv::Mat(3,3,cv::DataType<cv::Point3f>::type);
        h = nMat * mMatT;
        cv::add(H,h,H);
    }
    cout << H << endl;

    cv::Mat w, u, vt;
    cv::SVD::compute(H,w,u,vt);
    cv::Mat R = cv::Mat(3,3,cv::DataType<float>::type);
    R = vt*u;

    cv::Mat t;

    cv::add(-R*nCenter, mCenter, t);

    cout << "R: " << R << endl;
    cout << "t: " << t << endl;

    // TODO: use eigen to convert to T
    // apply T to n should get m

    // TODO: return T
}

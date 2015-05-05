#include "solvep3p.h"

SolveP3P::SolveP3P()
{

}

void SolveP3P::solveP3P(cv::Mat imageCoord, cv::Mat cameraMatrix, cv::Mat distCoeffs)
{
    normalizeImagePoints(imageCoord, cameraMatrix, distCoeffs);
    setUpP3PEquationSystem();
    solveP3PEquationSystem();
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
    cv::Point3f u = normalizedCoord.at<cv::Point3f>(0);
    cv::Point3f v = normalizedCoord.at<cv::Point3f>(1);
    cv::Point3f w = normalizedCoord.at<cv::Point3f>(2);
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

void SolveP3P::solveP3PEquationSystem()
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
    cout << "a4: " << a4 << endl;
    cout << "a3: " << a3 << endl;
    cout << "a2: " << a2 << endl;
    cout << "a1: " << a1 << endl;
    cout << "a0: " << a0 << endl;

    double X [4];
    int ret = SolveP4(X, a3/a4, a2/a4, a1/a4, a0/a4);
    cout << "ret: " << ret << endl;

    double x = X[0];

    b1 = b*pow(((pow(p,2)-p*q*r+pow(r,2))*a+(pow(p,2)-pow(r,2))*b-pow(p,2)+p*q*r-pow(r,2)),2);
    b0 = ((1-a-b)*pow(x,2)+(a-1)*q*x-a+b+1)*
            (pow(r,3)*(pow(a,2)+pow(b,2)-2*a-2*b+(2-pow(r,2))*a*b+1)*pow(x,3)+
            pow(r,2)*(p+p*pow(a,2)-2*r*q*a*b+2*r*q*b-2*r*q-2*p*a-2*p*b+p*pow(r,2)*b+4*r*q*a+q*pow(r,3)*a*b-2*r*q*pow(a,2)+2*p*a*b+p*pow(b,2)-pow(r,2)*p*pow(b,2))*pow(x,2)+
            (pow(r,5)*(pow(b,2)-a*b)-pow(r,4)*p*q*b+pow(r,3)*(pow(q,2)-4*a-2*pow(q,2)*a+pow(q,2)*pow(a,2)+2*pow(a,2)-2*pow(b,2)+2)+pow(r,2)*(4*p*q*a-2*p*q*a*b+2*p*q*b-2*p*q-2*p*q*pow(a,2)+
            r*(pow(p,2)*pow(b,2)-2*pow(p,2)*b+2*pow(p,2)*a*b-2*pow(p,2)*a+pow(p,2)+pow(p,2)*pow(a,2)))*x+
            (2*p*pow(r,2)-2*pow(r,3)*q+pow(p,3)-2*pow(p,2)*q*r+p*pow(q,2)*pow(r,2))*pow(a,2)+(pow(p,3)-2*p*pow(r,2))*pow(b,2)+(4*q*pow(r,3)-4*p*pow(r,2)-2*pow(p,3)+4*pow(p,2)*q*r-2*p*pow(q,2)*pow(r,2))*a+
            (-2*q*pow(r,3)+p*pow(r,4)+2*pow(p,2)*q*r-2*pow(p,3))*b+(2*pow(p,3)+2*q*pow(r,3)-2*pow(p,2)*q*r)*a*b+
            p*pow(q,2)*pow(r,2)-2*pow(p,2)*q*r+2*p*pow(r,2)+pow(p,3)-2*pow(r,3)*q));



    cout << "b1: " << b1 << endl;
    cout << "b0: " << b0 << endl;

}

#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

long double len[] = {0.3415,0.39399990552675401,0.366,0.25029999994029001,0.299};
const long double pi = 3.1415926535897932384626433832795;
Vector<long double, 3> pos[10],a[10];
Vector<long double,7> theta;
Matrix<long double, 6, 7> J;
Vector<long double, 7> des;
Vector<long double, 6> delta;
Quaternion<long double> q_end;
Quaternion<long double> des_q(0.266747,0,0.899519,0.345994);
Vector<long double, 3> des_pos(-0.352357, 0.611308, 0.237226);
long long cnt;
long double alpha = 0.000005;

void getTransform(Matrix<long double,4,4> &T,int num)
{
    Matrix<long double,3,3> R;
    Vector<long double,3> b;
    if(num == 1)
    {
        R << cos(theta(0)),sin(theta(0)),0,
            -sin(theta(0)),cos(theta(0)),0,
            0,0,1;
        b << 0,0,0.3415;
    }
    else if(num & 1)
    {
        R << cos(theta(num - 1)),sin(theta(num - 1)),0,
            0,0,1,
            sin(theta(num - 1)),-cos(theta(num - 1)),0;
        b << 0,len[num >> 1],0;
    }
    else
    {
        R << cos(theta(num - 1)),-sin(theta(num - 1)),0,
             0,0,-1,
             sin(theta(num - 1)),cos(theta(num - 1)),0;
        b << 0,0,0;
    }
    if(num == 7) b(2) = 0.00038204393466717502;
    T << R,b,0,0,0,1;
}

Matrix<long double,4,4> update(int num)
{
    if(num == 0)    return Matrix<long double,4,4>::Identity();
    Matrix<long double,4,4> T0 = update(num - 1);
    Matrix<long double,4,4> T;
    Vector<long double, 4> p(0,0,0,1);
    getTransform(T,num);
    T = T0 * T,p = T * p, pos[num] = p.block(0,0,3,1);
    Matrix<long double,3,3> R = T.block(0,0,3,3);
    if(num & 1) a[num] << 0, 0, -1;
    else        a[num] << 0, 0, 1;
    a[num] = R * a[num];
    if(num == 7)    q_end = Quaternion<long double>(R);
    // cout << "num = " << num << ",a = " << a[num].transpose() << endl;
    return T;
}

void getJacobi(Matrix<long double,6,7> &Je)
{
    for(int i = 1; i <= 7; ++i)
        Je.block(0,i - 1,3,1) << a[i].cross(pos[7] - pos[i]),Je.block(3, i - 1, 3, 1) << a[i];
    // cout << Je << endl;
}

void getDeltaX()
{
    AngleAxis<long double> Ax(q_end.conjugate() * des_q);
    delta.block(0,0,3,1) = des_pos - pos[7];
    delta.block(3,0,3,1) = Ax.axis() * Ax.angle();
}

int main()
{
    for(int i = 0; i < 7; ++i)  theta(i) = 0.785398163;
    update(7);
    des_pos = pos[7],des_q = q_end;
    theta(2) = 1.570796327;
    update(7);
    getDeltaX();
    while(delta.norm() > 2e-4)
    {
        getJacobi(J);
        JacobiSVD< Matrix<long double,6,7> > svd(J,ComputeFullU | ComputeFullV);
        Vector<long double,7> q = svd.solve(delta * alpha);
        // Matrix<long double, 7, 6> Jp = J.transpose() * (J * J.transpose()).inverse();
        // Vector<long double,7> q = Jp * delta;
        theta += q,update(7);
        ++cnt,getDeltaX();
        if(cnt % 2000 == 0)
            cout << "cnt = " << cnt << ":\ntheta = " << theta.transpose() << "\ndelta = " << delta.transpose() << "\nnorm = " << delta.norm() << endl;
    }
    return 0;
}
//
// Created by gishr on 19-6-7.
//

#include "bresenham.h"

vector<Vector3f> Bresenham3D(Vector3f point_1, Vector3f point_2){


    int x1 = point_1[0];
    int y1 = point_1[1];
    int z1 = point_1[2];
    int x2 = point_2[0];
    int y2 = point_2[1];
    int z2 = point_2[2];

    cout<<"bresenham3d 1"<<endl;

    int i, dx, dy, dz, l, m, n, x_inc, y_inc, z_inc, err_1, err_2, dx2, dy2, dz2;

    Vector3f point;

    point[0] = x1;
    point[1] = y1;
    point[2] = z1;

    dx = x2 - x1;
    dy = y2 - y1;
    dz = z2 - z1;

    x_inc = (dx < 0) ? -1 : 1;

    l = abs(dx);

    y_inc = (dy < 0) ? -1 : 1;

    m = abs(dy);

    z_inc = (dz < 0) ? -1 : 1;

    n = abs(dz);

    dx2 = l << 1;

    dy2 = m << 1;

    dz2 = n << 1;

    vector<Vector3f> path;


    cout<<"bresenham3d 2"<<endl;

    if ((l >= m) && (l >= n)) {

        err_1 = dy2 - l;

        err_2 = dz2 - l;

        for (i = 0; i < l; i++) {


            path.emplace_back(point);

            if (err_1 > 0) {

                point[1] += y_inc;

                err_1 -= dx2;

            }

            if (err_2 > 0) {

                point[2] += z_inc;

                err_2 -= dx2;

            }

            err_1 += dy2;

            err_2 += dz2;

            point[0] += x_inc;

//            path.push_back(point);

        }


    } else if ((m >= l) && (m >= n)) {

        err_1 = dx2 - m;

        err_2 = dz2 - m;

        for (i = 0; i < m; i++) {

            path.emplace_back(point);
            if (err_1 > 0) {

                point[0] += x_inc;

                err_1 -= dy2;

            }

            if (err_2 > 0) {

                point[2] += z_inc;

                err_2 -= dy2;

            }

            err_1 += dx2;

            err_2 += dz2;

            point[1] += y_inc;

//            path.push_back(point);

        }


    } else {
        err_1 = dy2 - n;

        err_2 = dx2 - n;

        for (i = 0; i < n; i++) {

            path.emplace_back(point);

            if (err_1 > 0) {

                point[1] += y_inc;

                err_1 -= dz2;

            }

            if (err_2 > 0) {

                point[0] += x_inc;

                err_2 -= dz2;

            }

            err_1 += dy2;

            err_2 += dx2;

            point[2] += z_inc;

            path.push_back(point);
        }

    }

    cout<<"bresenham3d 3"<<endl;
    path.push_back(point);

    return path;
}
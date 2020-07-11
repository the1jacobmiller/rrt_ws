#ifndef POINT_H
#define POINT_H

struct point {
    double x;
    double y;

    point(double x_, double y_) {
        x=x_;
        y=y_;
    }

    point() {}
};

#endif

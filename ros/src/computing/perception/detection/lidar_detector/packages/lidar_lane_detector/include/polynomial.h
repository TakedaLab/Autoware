/*
 *  Example code for fitting a polynomial to sample data (using Eigen 3)
 *
 *  Copyright (C) 2014  RIEGL Research ForschungsGmbH
 *  Copyright (C) 2014  Clifford Wolf <clifford@clifford.at>
 *  Copyright (c) 2019, Nagoya University
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#ifndef ROS_POLYNOMIAL_H
#define ROS_POLYNOMIAL_H

#include <Eigen/QR>
#include <stdio.h>
#include <vector>
#include <iostream>

class Polynomial {

  public:
    Polynomial(int order);
    virtual ~Polynomial();
    void fit(const std::vector<double> &xv, const std::vector<double> &yv);
    void fit(const Eigen::VectorXd &xv, const Eigen::VectorXd &yv);
    void predict(const std::vector<double> &xv, std::vector<double> &yv);
    void predict(const Eigen::VectorXd &xv, Eigen::VectorXd &yv);
    double predict(const double x);
    float predict(const float x);
    void eval(const std::vector<double> &xv, const std::vector<double> &yv, std::vector<double> &ev);
    void eval(const Eigen::VectorXd &xv, const Eigen::VectorXd &yv, Eigen::VectorXd &ev);
    std::vector<double> getCoefficients();
    void setCoefficients(std::vector<double> coeffs);

  private:
    int order_;
    std::vector<double> coeffs_;

};

#endif //ROS_POLYNOMIAL_H
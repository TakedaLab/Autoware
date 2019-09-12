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

#include <polynomial.h>


Polynomial::Polynomial(int order)
{
  order_ = order;
}

Polynomial::~Polynomial()
{
  // deconstruct
}

std::vector<double> Polynomial::getCoefficients()
{
  return coeffs_;
}

void Polynomial::setCoefficients(std::vector<double> coeffs)
{
  assert(coeffs.size() == order_+1);
  coeffs_ = coeffs;
}

void Polynomial::fit(const std::vector<double> &xv, const std::vector<double> &yv)
{
  Eigen::VectorXd xv_mapped = Eigen::VectorXd::Map(&xv.front(), xv.size());
  Eigen::VectorXd yv_mapped = Eigen::VectorXd::Map(&yv.front(), yv.size());

  fit(xv_mapped, yv_mapped);
}


void Polynomial::fit(const Eigen::VectorXd &xv, const Eigen::VectorXd &yv)
{
  Eigen::MatrixXd A(xv.size(), order_+1);
  Eigen::VectorXd result;

  assert(xv.size() == yv.size());
  assert(xv.size() >= order_+1);

  // create matrix
  for (long i = 0; i < xv.size(); i++)
    for (int j = 0; j < order_+1; j++)
      A(i, j) = pow(xv(i), j);

  // solve for linear least squares fit
  result = A.householderQr().solve(yv);

  coeffs_.clear();
  coeffs_.resize(order_+1);
  for (int i = 0; i < order_+1; i++)
    coeffs_[i] = result[i];
}

void Polynomial::predict(const std::vector<double> &xv, std::vector<double> &yv)
{
  Eigen::VectorXd xv_mapped = Eigen::VectorXd::Map(&xv.front(), xv.size());
  Eigen::VectorXd yv_mapped;
  predict(xv_mapped, yv_mapped);
  yv.resize(yv_mapped.size());
  Eigen::VectorXd::Map(&yv[0], yv_mapped.size()) = yv_mapped;
}

void Polynomial::predict(const Eigen::VectorXd &xv, Eigen::VectorXd &yv)
{
  yv.resize(xv.size());

  double y;
  for (long i = 0; i < xv.size(); i++) {
    y = 0;
    for (int j = 0; j < order_ + 1; j++)
      y += coeffs_[j] * pow(xv(i), j);
    yv(i) = y;
  }
}

double Polynomial::predict(const double x)
{
  double y = 0;
  for (int j = 0; j < order_+1; j++)
    y += coeffs_[j] * pow(x, j);
  return y;
}

float Polynomial::predict(const float x)
{
  float y = 0;
  for (int j = 0; j < order_+1; j++)
    y += coeffs_[j] * pow(x, j);
  return y;
}

void Polynomial::eval(const std::vector<double> &xv, const std::vector<double> &yv, std::vector<double> &ev)
{
  Eigen::VectorXd xv_mapped = Eigen::VectorXd::Map(&xv.front(), xv.size());
  Eigen::VectorXd yv_mapped = Eigen::VectorXd::Map(&yv.front(), yv.size());
  Eigen::VectorXd ev_mapped;
  eval(xv_mapped, yv_mapped, ev_mapped);
  ev.resize(ev_mapped.size());
  Eigen::VectorXd::Map(&ev[0], ev_mapped.size()) = ev_mapped;
}

void Polynomial::eval(const Eigen::VectorXd &xv, const Eigen::VectorXd &yv, Eigen::VectorXd &ev)
{
  Eigen::VectorXd yv_predicted;
  predict(xv, yv_predicted);
  ev.resize(xv.size());
  ev = yv - yv_predicted;
}


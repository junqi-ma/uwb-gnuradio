// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

namespace openuwb::core {

// Ordinary least squares for y = intercept + slope * x. Callers select
// which points to fit, center them if needed and check the determinant in
// their own units before using the slope.
class LinearFit {
public:
    void add(double x, double y)
    {
        sx_ += x;
        sy_ += y;
        sxx_ += x * x;
        sxy_ += x * y;
        count_ += 1.0;
    }

    double determinant() const { return count_ * sxx_ - sx_ * sx_; }

    double slope(double determinant) const
    {
        return (count_ * sxy_ - sx_ * sy_) / determinant;
    }

    double intercept(double slope) const { return (sy_ - slope * sx_) / count_; }

private:
    double sx_ = 0.0;
    double sy_ = 0.0;
    double sxx_ = 0.0;
    double sxy_ = 0.0;
    double count_ = 0.0;
};

} // namespace openuwb::core

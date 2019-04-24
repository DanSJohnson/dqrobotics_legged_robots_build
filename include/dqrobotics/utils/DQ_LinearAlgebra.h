/**
(C) Copyright 2019 DQ Robotics Developers

This file is part of DQ Robotics.

    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.

Contributors:
- Murilo M. Marinho (murilo@nml.t.u-tokyo.ac.jp)
*/

#include<eigen3/Eigen/Dense>
using namespace Eigen;

namespace DQ_robotics
{

/**
 * @brief pinv Calculates the pseudo inverse of the input @p matrix using Singular Value Decomposition with
 * a given tolerance for small singular values.
 * http://www.mathworks.com/help/matlab/ref/pinv.html
 * @param matrix the input matrix
 * @return the pseudo-inverse of @p matrix such that pinv(matrix)*matrix is as close as possible to the identity matrix.
 */
MatrixXd pinv(const MatrixXd& matrix);

}
/*!
 \file      dtDynTypes.cpp
 \brief     Native dtDynamics type utility definitions.
 \author    Joonhee Jo, joonhee.jo@hyundai.com
 \date      2026. 07. 16
 \version   0.1.0
 \copyright RoboticsLab ART All rights reserved.
*/

#include "dtDynamics/dtDynTypes.h"

#include <cassert>

namespace dt::Dynamics
{

std::size_t ToSize(const dtDynIndex value)
{
    return value > 0 ? static_cast<std::size_t>(value) : static_cast<std::size_t>(0);
}

uint16_t ToDtMathDim(const dtDynIndex value)
{
    assert(value >= 0 && value <= static_cast<dtDynIndex>(std::numeric_limits<uint16_t>::max()));
    return static_cast<uint16_t>(value);
}

void ResizeDenseVector(dtDynDenseVector &vector, const dtDynIndex dim)
{
    const uint16_t rows = ToDtMathDim(dim);

    if (vector.GetDim() == 0)
    {
        vector.Release();
        vector.NewSize(rows);
        return;
    }

    vector.ReSize(rows);
}

void ResizeDenseMatrix(dtDynDenseMatrix &matrix, const dtDynIndex rows, const dtDynIndex cols)
{
    const uint16_t mathRows = ToDtMathDim(rows);
    const uint16_t mathCols = ToDtMathDim(cols);

    if (matrix.GetRowSize() == 0 && matrix.GetColSize() == 0)
    {
        matrix.Release();
        matrix.NewSize(mathRows, mathCols);
        return;
    }

    matrix.ReSize(mathRows, mathCols);
}

void EnsureDenseVectorSize(dtDynDenseVector &vector, const dtDynIndex dim)
{
    const uint16_t rows = ToDtMathDim(dim);
    if (vector.GetDim() != rows)
    {
        ResizeDenseVector(vector, dim);
    }
}

void EnsureDenseMatrixSize(dtDynDenseMatrix &matrix, const dtDynIndex rows, const dtDynIndex cols)
{
    const uint16_t mathRows = ToDtMathDim(rows);
    const uint16_t mathCols = ToDtMathDim(cols);
    if (matrix.GetRowSize() != mathRows || matrix.GetColSize() != mathCols)
    {
        ResizeDenseMatrix(matrix, rows, cols);
    }
}

dtDynJointDofSize GetJointDofSize(const dtDynJointType type)
{
    switch (type)
    {
    case dtDynJointType::Free:
        return {static_cast<dtDynIndex>(7), static_cast<dtDynIndex>(6)};
    case dtDynJointType::Ball:
        return {static_cast<dtDynIndex>(4), static_cast<dtDynIndex>(3)};
    case dtDynJointType::Slide:
    case dtDynJointType::Hinge:
        return {static_cast<dtDynIndex>(1), static_cast<dtDynIndex>(1)};
    }

    return {static_cast<dtDynIndex>(0), static_cast<dtDynIndex>(0)};
}

} // namespace dt::Dynamics

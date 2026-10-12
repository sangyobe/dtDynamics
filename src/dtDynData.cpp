/*!
 \file      dtDynData.cpp
 \brief     Mutable native dtDynamics runtime data definitions.
 \author    Joonhee Jo, joonhee.jo@hyundai.com
 \date      2026. 07. 16
 \version   0.1.3
 \copyright RoboticsLab ART All rights reserved.
*/

#include "dtDynamics/dtDynData.h"

namespace dt::Dynamics
{

namespace
{

bool DenseVectorHasSize(const dtDynDenseVector &vector, const dtDynIndex dim)
{
    return vector.GetDim() == ToDtMathDim(dim);
}

bool DenseMatrixHasSize(const dtDynDenseMatrix &matrix,
                        const dtDynIndex rows,
                        const dtDynIndex cols)
{
    return matrix.GetRowSize() == ToDtMathDim(rows) &&
           matrix.GetColSize() == ToDtMathDim(cols);
}

void SetDenseMatrixZeroIfAllocated(dtDynDenseMatrix &matrix)
{
    if (matrix.GetRowSize() > 0 && matrix.GetColSize() > 0)
    {
        matrix.SetZero();
    }
}

template <typename T>
bool ArrayHasSize(const dtArray<T> &array, const dtDynIndex size)
{
    return array.size() == static_cast<std::size_t>(size);
}

bool DenseMatrixArrayHasSize(const dtArray<dtDynDenseMatrix> &array,
                             const dtDynIndex count,
                             const dtDynIndex rows,
                             const dtDynIndex cols)
{
    if (!ArrayHasSize(array, count))
    {
        return false;
    }

    for (dtDynIndex index = 0; index < count; ++index)
    {
        if (!DenseMatrixHasSize(array[static_cast<std::size_t>(index)], rows, cols))
        {
            return false;
        }
    }

    return true;
}

} // namespace

void dtDynData::Resize(const dtDynModel &model)
{
    ResizeDenseVector(qpos, model.nq);
    ResizeDenseVector(qvel, model.nv);

    xpos.resize(model.nbody);
    xquat.resize(model.nbody);
    xmat.resize(model.nbody);
    xipos.resize(model.nbody);
    ximat.resize(model.nbody);
    subtree_com.resize(model.nbody);
    xanchor.resize(model.njnt);
    xaxis.resize(model.njnt);
    site_xpos.resize(model.nsite);
    site_xmat.resize(model.nsite);

    body_jacp.resize(model.nbody);
    body_jacr.resize(model.nbody);
    body_jacp_dot.resize(model.nbody);
    body_jacr_dot.resize(model.nbody);
    site_jacp.resize(model.nsite);
    site_jacr.resize(model.nsite);
    site_jacp_dot.resize(model.nsite);
    site_jacr_dot.resize(model.nsite);

    for (dtDynIndex bodyId = 0; bodyId < model.nbody; ++bodyId)
    {
        ResizeDenseMatrix(body_jacp[bodyId], 3, model.nv);
        ResizeDenseMatrix(body_jacr[bodyId], 3, model.nv);
        ResizeDenseMatrix(body_jacp_dot[bodyId], 3, model.nv);
        ResizeDenseMatrix(body_jacr_dot[bodyId], 3, model.nv);
        SetDenseMatrixZeroIfAllocated(body_jacp[bodyId]);
        SetDenseMatrixZeroIfAllocated(body_jacr[bodyId]);
        SetDenseMatrixZeroIfAllocated(body_jacp_dot[bodyId]);
        SetDenseMatrixZeroIfAllocated(body_jacr_dot[bodyId]);
    }

    for (dtDynIndex siteId = 0; siteId < model.nsite; ++siteId)
    {
        ResizeDenseMatrix(site_jacp[siteId], 3, model.nv);
        ResizeDenseMatrix(site_jacr[siteId], 3, model.nv);
        ResizeDenseMatrix(site_jacp_dot[siteId], 3, model.nv);
        ResizeDenseMatrix(site_jacr_dot[siteId], 3, model.nv);
        SetDenseMatrixZeroIfAllocated(site_jacp[siteId]);
        SetDenseMatrixZeroIfAllocated(site_jacr[siteId]);
        SetDenseMatrixZeroIfAllocated(site_jacp_dot[siteId]);
        SetDenseMatrixZeroIfAllocated(site_jacr_dot[siteId]);
    }

    cdof.resize(model.nv);
    cdof_dot.resize(model.nv);
    cvel.resize(model.nbody);
    cinert.resize(model.nbody);
    crb.resize(model.nbody);
    BmatComp.resize(model.nbody);
    BmatCompNonzero.resize(model.nbody);
    f1.resize(model.nv);
    f2.resize(model.nv);
    f3.resize(model.nv);
    cacc.resize(model.nbody);
    cfrc.resize(model.nbody);
    scratch_weightedCom.resize(model.nbody);

    for (dtDynIndex bodyId = 0; bodyId < model.nbody; ++bodyId)
    {
        BmatComp[static_cast<std::size_t>(bodyId)].SetZero();
        BmatCompNonzero[static_cast<std::size_t>(bodyId)] = 0;
    }

    ResizeDenseMatrix(Mmat, model.nv, model.nv);
    ResizeDenseMatrix(MmatDot, model.nv, model.nv);
    ResizeDenseMatrix(Cmat, model.nv, model.nv);
    ResizeDenseVector(gravityTorq, model.nv);
    ResizeDenseVector(coriolisTorq, model.nv);
    ResizeDenseVector(nonlinTorq, model.nv);

    const dtDynIndex rootCount = model.nroot > 0 ? model.nroot : 1;
    ResizeDenseMatrix(comJmat, 3 * rootCount, model.nv);
    ResizeDenseMatrix(comJmatDot, 3 * rootCount, model.nv);
    ResizeDenseMatrix(cmAmat, 6 * rootCount, model.nv);
    ResizeDenseMatrix(cmAmatDot, 6 * rootCount, model.nv);
    ResizeDenseVector(centMom, 6 * rootCount);
    ResizeDenseVector(comPos, 3 * rootCount);
    ResizeDenseVector(comVel, 3 * rootCount);
}

bool dtDynData::IsCompatible(const dtDynModel &model) const
{
    dtDynIndex rootCount = 1;
    if (model.nroot > 0)
    {
        rootCount = model.nroot;
    }

    if (!DenseVectorHasSize(qpos, model.nq))
    {
        return false;
    }
    if (!DenseVectorHasSize(qvel, model.nv))
    {
        return false;
    }
    if (!ArrayHasSize(xpos, model.nbody))
    {
        return false;
    }
    if (!ArrayHasSize(xquat, model.nbody))
    {
        return false;
    }
    if (!ArrayHasSize(xmat, model.nbody))
    {
        return false;
    }
    if (!ArrayHasSize(xipos, model.nbody))
    {
        return false;
    }
    if (!ArrayHasSize(ximat, model.nbody))
    {
        return false;
    }
    if (!ArrayHasSize(subtree_com, model.nbody))
    {
        return false;
    }
    if (!ArrayHasSize(xanchor, model.njnt))
    {
        return false;
    }
    if (!ArrayHasSize(xaxis, model.njnt))
    {
        return false;
    }
    if (!ArrayHasSize(site_xpos, model.nsite))
    {
        return false;
    }
    if (!ArrayHasSize(site_xmat, model.nsite))
    {
        return false;
    }
    if (!DenseMatrixArrayHasSize(body_jacp, model.nbody, 3, model.nv))
    {
        return false;
    }
    if (!DenseMatrixArrayHasSize(body_jacr, model.nbody, 3, model.nv))
    {
        return false;
    }
    if (!DenseMatrixArrayHasSize(body_jacp_dot, model.nbody, 3, model.nv))
    {
        return false;
    }
    if (!DenseMatrixArrayHasSize(body_jacr_dot, model.nbody, 3, model.nv))
    {
        return false;
    }
    if (!DenseMatrixArrayHasSize(site_jacp, model.nsite, 3, model.nv))
    {
        return false;
    }
    if (!DenseMatrixArrayHasSize(site_jacr, model.nsite, 3, model.nv))
    {
        return false;
    }
    if (!DenseMatrixArrayHasSize(site_jacp_dot, model.nsite, 3, model.nv))
    {
        return false;
    }
    if (!DenseMatrixArrayHasSize(site_jacr_dot, model.nsite, 3, model.nv))
    {
        return false;
    }
    if (!ArrayHasSize(cdof, model.nv))
    {
        return false;
    }
    if (!ArrayHasSize(cdof_dot, model.nv))
    {
        return false;
    }
    if (!ArrayHasSize(cvel, model.nbody))
    {
        return false;
    }
    if (!ArrayHasSize(cinert, model.nbody))
    {
        return false;
    }
    if (!ArrayHasSize(crb, model.nbody))
    {
        return false;
    }
    if (!ArrayHasSize(BmatComp, model.nbody))
    {
        return false;
    }
    if (!ArrayHasSize(BmatCompNonzero, model.nbody))
    {
        return false;
    }
    if (!ArrayHasSize(f1, model.nv))
    {
        return false;
    }
    if (!ArrayHasSize(f2, model.nv))
    {
        return false;
    }
    if (!ArrayHasSize(f3, model.nv))
    {
        return false;
    }
    if (!ArrayHasSize(cacc, model.nbody))
    {
        return false;
    }
    if (!ArrayHasSize(cfrc, model.nbody))
    {
        return false;
    }
    if (!ArrayHasSize(scratch_weightedCom, model.nbody))
    {
        return false;
    }
    if (!DenseMatrixHasSize(Mmat, model.nv, model.nv))
    {
        return false;
    }
    if (!DenseMatrixHasSize(MmatDot, model.nv, model.nv))
    {
        return false;
    }
    if (!DenseMatrixHasSize(Cmat, model.nv, model.nv))
    {
        return false;
    }
    if (!DenseVectorHasSize(gravityTorq, model.nv))
    {
        return false;
    }
    if (!DenseVectorHasSize(coriolisTorq, model.nv))
    {
        return false;
    }
    if (!DenseVectorHasSize(nonlinTorq, model.nv))
    {
        return false;
    }
    if (!DenseMatrixHasSize(comJmat, 3 * rootCount, model.nv))
    {
        return false;
    }
    if (!DenseMatrixHasSize(comJmatDot, 3 * rootCount, model.nv))
    {
        return false;
    }
    if (!DenseMatrixHasSize(cmAmat, 6 * rootCount, model.nv))
    {
        return false;
    }
    if (!DenseMatrixHasSize(cmAmatDot, 6 * rootCount, model.nv))
    {
        return false;
    }
    if (!DenseVectorHasSize(centMom, 6 * rootCount))
    {
        return false;
    }
    if (!DenseVectorHasSize(comPos, 3 * rootCount))
    {
        return false;
    }
    return DenseVectorHasSize(comVel, 3 * rootCount);
}

} // namespace dt::Dynamics

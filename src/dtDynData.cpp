/*!
 \file      dtDynData.cpp
 \brief     Mutable native dtDynamics runtime data definitions.
 \author    Joonhee Jo, joonhee.jo@hyundai.com
 \date      2026. 07. 16
 \version   0.1.0
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

bool DenseMatrixHasSize(const dtDynDenseMatrix &matrix, const dtDynIndex rows,
                        const dtDynIndex cols)
{
    return matrix.GetRowSize() == ToDtMathDim(rows) && matrix.GetColSize() == ToDtMathDim(cols);
}

template <typename T> bool ArrayHasSize(const dtArray<T> &array, const dtDynIndex size)
{
    return array.size() == static_cast<std::size_t>(size);
}

bool DenseMatrixArrayHasSize(const dtArray<dtDynDenseMatrix> &array, const dtDynIndex count,
                             const dtDynIndex rows, const dtDynIndex cols)
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
        body_jacp[bodyId].SetZero();
        body_jacr[bodyId].SetZero();
        body_jacp_dot[bodyId].SetZero();
        body_jacr_dot[bodyId].SetZero();
    }

    for (dtDynIndex siteId = 0; siteId < model.nsite; ++siteId)
    {
        ResizeDenseMatrix(site_jacp[siteId], 3, model.nv);
        ResizeDenseMatrix(site_jacr[siteId], 3, model.nv);
        ResizeDenseMatrix(site_jacp_dot[siteId], 3, model.nv);
        ResizeDenseMatrix(site_jacr_dot[siteId], 3, model.nv);
        site_jacp[siteId].SetZero();
        site_jacr[siteId].SetZero();
        site_jacp_dot[siteId].SetZero();
        site_jacr_dot[siteId].SetZero();
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
    const dtDynIndex rootCount = model.nroot > 0 ? model.nroot : 1;

    return DenseVectorHasSize(qpos, model.nq) && DenseVectorHasSize(qvel, model.nv) &&
           ArrayHasSize(xpos, model.nbody) && ArrayHasSize(xquat, model.nbody) &&
           ArrayHasSize(xmat, model.nbody) && ArrayHasSize(xipos, model.nbody) &&
           ArrayHasSize(ximat, model.nbody) && ArrayHasSize(subtree_com, model.nbody) &&
           ArrayHasSize(xanchor, model.njnt) && ArrayHasSize(xaxis, model.njnt) &&
           ArrayHasSize(site_xpos, model.nsite) && ArrayHasSize(site_xmat, model.nsite) &&
           DenseMatrixArrayHasSize(body_jacp, model.nbody, 3, model.nv) &&
           DenseMatrixArrayHasSize(body_jacr, model.nbody, 3, model.nv) &&
           DenseMatrixArrayHasSize(body_jacp_dot, model.nbody, 3, model.nv) &&
           DenseMatrixArrayHasSize(body_jacr_dot, model.nbody, 3, model.nv) &&
           DenseMatrixArrayHasSize(site_jacp, model.nsite, 3, model.nv) &&
           DenseMatrixArrayHasSize(site_jacr, model.nsite, 3, model.nv) &&
           DenseMatrixArrayHasSize(site_jacp_dot, model.nsite, 3, model.nv) &&
           DenseMatrixArrayHasSize(site_jacr_dot, model.nsite, 3, model.nv) &&
           ArrayHasSize(cdof, model.nv) && ArrayHasSize(cdof_dot, model.nv) &&
           ArrayHasSize(cvel, model.nbody) && ArrayHasSize(cinert, model.nbody) &&
           ArrayHasSize(crb, model.nbody) && ArrayHasSize(BmatComp, model.nbody) &&
           ArrayHasSize(BmatCompNonzero, model.nbody) && ArrayHasSize(f1, model.nv) &&
           ArrayHasSize(f2, model.nv) && ArrayHasSize(f3, model.nv) &&
           ArrayHasSize(cacc, model.nbody) && ArrayHasSize(cfrc, model.nbody) &&
           ArrayHasSize(scratch_weightedCom, model.nbody) &&
           DenseMatrixHasSize(Mmat, model.nv, model.nv) &&
           DenseMatrixHasSize(MmatDot, model.nv, model.nv) &&
           DenseMatrixHasSize(Cmat, model.nv, model.nv) &&
           DenseVectorHasSize(gravityTorq, model.nv) &&
           DenseVectorHasSize(coriolisTorq, model.nv) && DenseVectorHasSize(nonlinTorq, model.nv) &&
           DenseMatrixHasSize(comJmat, 3 * rootCount, model.nv) &&
           DenseMatrixHasSize(comJmatDot, 3 * rootCount, model.nv) &&
           DenseMatrixHasSize(cmAmat, 6 * rootCount, model.nv) &&
           DenseMatrixHasSize(cmAmatDot, 6 * rootCount, model.nv) &&
           DenseVectorHasSize(centMom, 6 * rootCount) &&
           DenseVectorHasSize(comPos, 3 * rootCount) && DenseVectorHasSize(comVel, 3 * rootCount);
}

} // namespace dt::Dynamics

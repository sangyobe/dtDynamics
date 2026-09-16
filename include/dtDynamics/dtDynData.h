/*!
 \file      dtDynData.h
 \brief     Mutable native dtDynamics runtime data.
 \author    Joonhee Jo, joonhee.jo@hyundai.com
 \date      2026. 07. 16
 \version   0.1.2
 \copyright RoboticsLab ART All rights reserved.
*/

#ifndef ALGORITHM_DTDYNAMICS_DTDYNDATA_H_
#define ALGORITHM_DTDYNAMICS_DTDYNDATA_H_

#include "dtDynamics/dtDynModel.h"

namespace dt::Dynamics
{

/*!
 \brief Runtime state and computed dynamics outputs for a compiled dtDynModel.
 \details
  dtDynData is the mutable storage object used by kinematics and dynamics
  algorithms. It contains user-written state (`qpos`, `qvel`), world-space
  kinematic outputs, Jacobian outputs, spatial scratch arrays, mass/bias terms,
  centroidal quantities, and center-of-mass quantities.

  Call Resize(model) before entering a hard realtime loop. After Resize(), the
  documented realtime compute paths update existing dtDynData buffers rather
  than allocating new storage. Caller-owned destination matrices passed to APIs
  must also be pre-sized when the caller needs a strict zero-allocation path.
  Resizing, model compilation, and name lookup are setup-time work. Plain
  one-off Jacobians need current forward kinematics; derivative and bulk/indexed
  Jacobian writers also consume work arrays prepared by a current dynamics pass.

  The matrix/vector dimensions are derived from model.nq, model.nv, model.nbody,
  model.nsite, and model.nroot. Passing data resized for a different model is a
  contract violation; use IsCompatible() in integration checks or debug gates.
*/
class dtDynData
{
public:
    /* Generalized state */
    dtDynDenseVector qpos; //!< Caller-written configuration; free=[p_W(3), q_wxyz(4)] (nq).
    dtDynDenseVector qvel; //!< Caller-written tangent velocity; free=[v_W(3), omega_B(3)] (nv).

    /* Forward-kinematics outputs */
    dtArray<dt::Math::Vector3<dtDynReal>> xpos;        //!< Body-origin positions in world frame (nbody x 3).
    dtArray<dt::Math::Vector4<dtDynReal>> xquat;       //!< Body-to-world unit quaternions in wxyz order (nbody x 4).
    dtArray<dt::Math::Matrix3<dtDynReal>> xmat;        //!< Body-to-world rotation matrices (nbody x 3 x 3).
    dtArray<dt::Math::Vector3<dtDynReal>> xipos;       //!< Body inertial-origin positions in world frame (nbody x 3).
    dtArray<dt::Math::Matrix3<dtDynReal>> ximat;       //!< Inertial-to-world rotation matrices (nbody x 3 x 3).
    dtArray<dt::Math::Vector3<dtDynReal>> subtree_com; //!< Subtree centers of mass in world frame (nbody x 3).
    dtArray<dt::Math::Vector3<dtDynReal>> xanchor;     //!< Joint anchor positions in world frame (njnt x 3).
    dtArray<dt::Math::Vector3<dtDynReal>> xaxis;       //!< World hinge/slide axes; free-joint entries are local placeholders (njnt x 3).
    dtArray<dt::Math::Vector3<dtDynReal>> site_xpos;   //!< Site-origin positions in world frame (nsite x 3).
    dtArray<dt::Math::Matrix3<dtDynReal>> site_xmat;   //!< Site-to-world rotation matrices (nsite x 3 x 3).

    /* Body/site task Jacobian outputs */
    dtArray<dtDynDenseMatrix> body_jacp;     //!< World linear Jacobians at body origins (nbody x 3 x nv).
    dtArray<dtDynDenseMatrix> body_jacr;     //!< World angular Jacobians at body origins (nbody x 3 x nv).
    dtArray<dtDynDenseMatrix> body_jacp_dot; //!< Time derivatives of body linear Jacobians (nbody x 3 x nv).
    dtArray<dtDynDenseMatrix> body_jacr_dot; //!< Time derivatives of body angular Jacobians (nbody x 3 x nv).
    dtArray<dtDynDenseMatrix> site_jacp;     //!< World linear Jacobians at site origins (nsite x 3 x nv).
    dtArray<dtDynDenseMatrix> site_jacr;     //!< World angular Jacobians at site origins (nsite x 3 x nv).
    dtArray<dtDynDenseMatrix> site_jacp_dot; //!< Time derivatives of site linear Jacobians (nsite x 3 x nv).
    dtArray<dtDynDenseMatrix> site_jacr_dot; //!< Time derivatives of site angular Jacobians (nsite x 3 x nv).

    /* Spatial work arrays use world components, root-subtree CoM references,
       and [angular; linear] order unless stated otherwise. */
    dtArray<dt::Math::Vector6<dtDynReal>> cdof;                //!< World-frame motion-subspace columns at root CoM references (nv x 6).
    dtArray<dt::Math::Vector6<dtDynReal>> cdof_dot;            //!< Time derivatives of cdof columns (nv x 6).
    dtArray<dt::Math::Vector6<dtDynReal>> cvel;                //!< Body spatial velocities [omega_W; v_W] at root CoM references (nbody x 6).
    dtArray<dt::Math::Matrix<6, 6, dtDynReal>> cinert;         //!< World-frame body spatial inertias about root CoM references (nbody x 6 x 6).
    dtArray<dt::Math::Matrix<6, 6, dtDynReal>> crb;            //!< World-frame composite rigid-body inertias (nbody x 6 x 6).
    dtArray<dt::Math::Matrix<6, 6, dtDynReal>> BmatComp;       //!< Composite CCRBA bias matrices; inspect only when flagged nonzero (nbody x 6 x 6).
    dtArray<dtDynIndex> BmatCompNonzero;                       //!< Logical-nonzero flags; zero means corresponding BmatComp storage may be stale (nbody).
    dtArray<dt::Math::Vector6<dtDynReal>> f1;                  //!< CCRBA workspace: crb*cdof_dot + BmatComp*cdof (nv x 6).
    dtArray<dt::Math::Vector6<dtDynReal>> f2;                  //!< CCRBA workspace: crb*cdof (nv x 6).
    dtArray<dt::Math::Vector6<dtDynReal>> f3;                  //!< CCRBA workspace: transpose(BmatComp)*cdof (nv x 6).
    dtArray<dt::Math::Vector6<dtDynReal>> cacc;                //!< World-frame body spatial accelerations used by RNEA (nbody x 6).
    dtArray<dt::Math::Vector6<dtDynReal>> cfrc;                //!< World-frame body spatial forces used by RNEA (nbody x 6).
    dtArray<dt::Math::Vector3<dtDynReal>> scratch_weightedCom; //!< Mass-weighted CoM scratch values (nbody x 3).

    /* Base dynamics outputs */
    dtDynDenseMatrix Mmat;         //!< Generalized mass matrix (nv x nv).
    dtDynDenseMatrix MmatDot;      //!< Mass-matrix derivative; filled by CCRBA when MMAT_DOT is requested (nv x nv).
    dtDynDenseMatrix Cmat;         //!< Coriolis matrix satisfying Cmat*qvel = coriolisTorq; cleared by RNEA (nv x nv).
    dtDynDenseVector gravityTorq;  //!< Generalized gravity-force vector (nv).
    dtDynDenseVector coriolisTorq; //!< Generalized Coriolis-force vector (nv).
    dtDynDenseVector nonlinTorq;   //!< Generalized nonlinear force, gravity plus Coriolis (nv).

    /* Centroidal/CoM storage uses nr=max(nroot,1). The first nroot blocks
       correspond to roots; the nroot==0 fallback block is sizing-only. */
    dtDynDenseMatrix comJmat;    //!< Per-root CoM Jacobians (3*nr x nv).
    dtDynDenseMatrix comJmatDot; //!< Per-root CoM Jacobian time derivatives (3*nr x nv).
    dtDynDenseMatrix cmAmat;     //!< Per-root centroidal momentum matrices, [linear; angular] (6*nr x nv).
    dtDynDenseMatrix cmAmatDot;  //!< Per-root centroidal momentum matrix derivatives (6*nr x nv).
    dtDynDenseVector centMom;    //!< Per-root momentum [linear; angular] [kg m/s; kg m^2/s] (6*nr).
    dtDynDenseVector comPos;     //!< Per-root world CoM positions [m], filled when COM_POS is requested (3*nr).
    dtDynDenseVector comVel;     //!< Per-root world CoM velocities [m/s], filled when COM_VEL is requested (3*nr).

    /*! \brief Allocate and size all runtime arrays for \p model dimensions.
     \details This is setup-time work and may allocate heap memory. Do not call
      from the hard realtime control loop.
     \post IsCompatible(model) returns true when allocation succeeds.
    */
    void Resize(const dtDynModel &model);

    /*! \brief Check whether allocated dimensions match \p model.
     \return true when qpos/qvel and model-indexed output buffers are sized for
      the supplied model.
    */
    bool IsCompatible(const dtDynModel &model) const;
};

} // namespace dt::Dynamics

#endif // ALGORITHM_DTDYNAMICS_DTDYNDATA_H_

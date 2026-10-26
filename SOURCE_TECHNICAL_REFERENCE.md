# Technical reference to every document in `Source/`

**Status: consolidated technical reference, wrap-up edition dated 2026-10-17.** All twelve supplied PDFs (127 pages) are represented. Source-specific methods, mathematical derivations, algorithms, experiments, equation audits, and limitations are followed by a notation guide and equation/page and figure/table indexes. Missing source material and unresolved printed conventions are explicitly recorded in Section 16; this document does not claim independent experimental replication or formal verification of every source proof.

This document records the approaches, mathematical models, formulas, proofs, algorithms, experiments, assumptions, and limitations read in the twelve supplied PDFs. It is a source-by-source technical reference, followed by a synthesis. It is not a reproduction of the papers. Equation numbers identified as “source Eq.” refer to the linked PDF; other equations are explanatory restatements or derivations. Numerical results describe the experiments in the supplied versions, not independent replication.

**Coverage convention.** PDF page numbers count from the first page of the local file. All twelve files are included, including the dissertation preview and its missing-content boundary. The coverage includes problem formulation, geometry, dynamics, control, estimation, optimization, learning, proofs, parameter choices, experiments, figures/tables, caveats, and stated future work. Bibliographies identify dependencies, but cited works absent from this folder have not themselves been reviewed. Administrative material such as publisher notices and affiliations is not transcribed. Where a source omits a derivation or parameter, that omission is retained rather than filled with an invented result. Explanatory deductions and implementation cautions are explicitly distinguished from source claims.

## Contents

1. [Source inventory](#1-source-inventory)
2. [Shared mathematical foundations](#2-shared-mathematical-foundations)
3. [Coordinate-free dynamics and task flatness, ICRA 2020](#3-coordinate-free-dynamics-and-task-flatness-icra-2020)
4. [Dynamically feasible task-space planning, RA-L 2021](#4-dynamically-feasible-task-space-planning-ra-l-2021)
5. [Symmetry and geometric flat outputs](#5-symmetry-and-geometric-flat-outputs)
6. [Almost-global tracking on homogeneous manifolds](#6-almost-global-tracking-on-homogeneous-manifolds)
7. [Symmetry-reduced reinforcement learning](#7-symmetry-reduced-reinforcement-learning)
8. [Adaptive aerial-manipulator control](#8-adaptive-aerial-manipulator-control)
9. [Predictive admittance control](#9-predictive-admittance-control)
10. [Sensorless six-dimensional interaction](#10-sensorless-six-dimensional-interaction)
11. [Passive alignment during pushing](#11-passive-alignment-during-pushing)
12. [Separating interaction forces from disturbances](#12-separating-interaction-forces-from-disturbances)
13. [Ground-effect-aware modeling and control](#13-ground-effect-aware-modeling-and-control)
14. [Dissertation preview: available and unavailable material](#14-dissertation-preview-available-and-unavailable-material)
15. [Cross-paper synthesis and implementation implications](#15-cross-paper-synthesis-and-implementation-implications)
16. [Coverage, source limitations, and reproducibility](#16-coverage-source-limitations-and-reproducibility)
17. [Notation, frames, units, and symbol collisions](#17-notation-frames-units-and-symbol-collisions)
18. [Source-equation and page locator](#18-source-equation-and-page-locator)
19. [Figure, table, and experimental-evidence index](#19-figure-table-and-experimental-evidence-index)
20. [Revision record](#20-revision-record)

## 1. Source inventory

| ID | Local PDF | Title / supplied version | Pages | Principal subject |
|---|---|---|---:|---|
| P01 | [2025-4-RAL.pdf](Source/2025-4-RAL.pdf) | Predictive Admittance Control for Aerial Physical Interaction; Alharbat, Gabellieri, Mersha, Franchi; 2025 preprint | 8 | Joint prediction of vehicle and virtual impedance |
| P02 | [2209.11869v2 (1).pdf](Source/2209.11869v2%20%281%29.pdf) | The Role of Symmetry in Constructing Geometric Flat Outputs for Free-Flying Robotic Systems; Welde, Kvalheim, Kumar; v2, March 2023 | 7 | Principal bundles and sufficient conditions for flatness |
| P03 | [2409.11238v3.pdf](Source/2409.11238v3.pdf) | Leveraging Symmetry to Accelerate Learning of Trajectory Tracking Controllers for Free-Flying Robotic Systems; Welde, Rao, Kunapuli, Jayaraman, Kumar; v3, May 2025 | 8 | Quotient tracking MDPs and RL |
| P04 | [2506.19424v1.pdf](Source/2506.19424v1.pdf) | Ground-Effect-Aware Modeling and Control for Multicopters; Yang, Chai, Ji, Wu, Xu, Gao; 2025 | 11 | Near-ground lift, torque, drag, allocation, and estimation |
| P05 | [Almost Global Asymptotic Trajectory Tracking…](Source/Almost_Global_Asymptotic_Trajectory_Tracking_for_Fully-Actuated_Mechanical_Systems_on_Homogeneous_Riemannian_Manifolds.pdf) | Almost Global Asymptotic Trajectory Tracking for Fully-Actuated Mechanical Systems on Homogeneous Riemannian Manifolds; Welde, Kumar; 2024 | 6 | Intrinsic tracking errors and navigation functions |
| P06 | [IFAC14_ifac_final.pdf](Source/IFAC14_ifac_final.pdf) | Adaptive control for UAVs equipped with a robotic arm; Caccavale, Giglio, Muscio, Pierri; 2014 | 6 | Inverse kinematics and adaptive inverse dynamics |
| P07 | [Passive Aligning Physical Interaction…](Source/Passive_Aligning_Physical_Interaction_of_Fully-Actuated_Aerial_Vehicles_for_Pushing_Tasks.pdf) | Passive Aligning Physical Interaction of Fully-Actuated Aerial Vehicles for Pushing Tasks; Hui, Cuniato, Pantic, Tognon, Fumagalli, Siegwart; ICRA 2024 | 7 | Selective rotational compliance and passive alignment |
| P08 | [RAL_2021_Preprint.pdf](Source/RAL_2021_Preprint.pdf) | Dynamically Feasible Task Space Planning for Underactuated Aerial Manipulators; Welde, Paulos, Kumar; accepted December 2020, RA-L 2021 | 8 | General arm flatness, task inversion, internal dynamics |
| P09 | [drones-10-00129-v2 (1).pdf](Source/drones-10-00129-v2%20%281%29.pdf) | 6D Physical Interaction with an Omnidirectional Aerial Robot; Veenstra, Ali, Gabellieri, Franchi; Drones 10, 129 | 16 | Fixed-tilt omnidirectional robot, wrench observer, admittance |
| P10 | [icra_2020.pdf](Source/icra_2020.pdf) | Coordinate-Free Dynamics and Differential Flatness of a Class of 6DOF Aerial Manipulators; Welde, Kumar; ICRA 2020 | 7 | Three-body dynamics and special two-joint task flatness |
| P11 | [out (1).pdf](Source/out%20%281%29.pdf) | Geometric Methods for Efficient and Explainable Control of Underactuated Robotic Systems; Jake Welde; 2025 dissertation **preview** | 24 | Research program, contents, introduction; not full dissertation |
| P12 | [s10514-023-10128-2.pdf](Source/s10514-023-10128-2.pdf) | Multi-directional Interaction Force Control with an Aerial Manipulator Under External Disturbances; Malczyk, Brunner, Cuniato, Tognon, Siegwart; Autonomous Robots 2023 | 19 | EKF, tool compensation, hybrid force/motion control |

Total: **127 PDF pages**. File names are retained exactly in link labels where practical; encoded spaces in link targets are intentional.

## 2. Shared mathematical foundations

### 2.1 Configuration, state, velocity, and actuation are different objects

A rigid body's pose is an element of

$$
SE(3)=\left\{\begin{bmatrix}R&p\\0&1\end{bmatrix}:R\in SO(3),\ p\in\mathbb R^3\right\},
\qquad SO(3)=\{R:R^TR=I,\det R=1\}.
$$

A manipulator with a floating rigid base and $n$ revolute joints usually has configuration manifold $SE(3)\times\mathbb T^n$, with $\mathbb T^n=(S^1)^n$. It has $6+n$ configuration degrees of freedom. A conventional quadrotor supplies one thrust magnitude and three moments; adding $n$ independently actuated joints yields $4+n$ inputs and leaves two unactuated directions. A six-dimensional end-effector task can therefore be achieved by a two-joint quadrotor manipulator without the complete eight-dimensional configuration being fully actuated.

A state includes configuration **and** velocity: intrinsically it belongs to a tangent bundle $TQ$. A vector such as $v=(v_B,\omega_B,\dot\theta)$ is often a *quasi-velocity*: it is not the derivative of one global Euclidean coordinate vector. In particular,

$$\dot R=R\widehat\omega_B,\qquad \dot p=Rv_B.$$

The hat/vee operators obey $\widehat a b=a\times b$ and $(\widehat a)^\vee=a$. For a unit vector $s$, $I-ss^T$ projects onto $T_sS^2$, while $\widehat s^2=ss^T-I$.

### 2.2 Frames, origins, and power consistency

A rotation $R_{AB}$ maps coordinates from frame $B$ to frame $A$. Force vectors rotate, but a wrench also depends on its reference point. If $r$ points from origin $O$ to the force application point, then the moment about $O$ includes $r\times f$. Consequently, sensor-to-body transformations and body-to-tip transformations require both rotation and moment-arm terms.

A fixed tool offset $r$ in the body frame gives

$$
p_E=p_B+Rr,
\quad \dot p_E=\dot p_B+R(\omega\times r),
$$
$$
\ddot p_E=\ddot p_B+R\big(\dot\omega\times r+\omega\times(\omega\times r)\big).
$$

The angular-acceleration and centripetal terms are essential when a controller tracks the tool instead of the center of mass. A coordinate transformation of velocity must be accompanied by its dual transformation of force so that instantaneous power remains unchanged. If $\xi=Tv$, then the corresponding generalized force is $T^{-T}f$.

### 2.3 Mechanical equations and the origin of dynamic feasibility

The familiar equation

$$M(q)\dot v+C(q,v)v+g(q)=B(q)u$$

contains inertial coupling, velocity-dependent forces, gravity, and the actuation map. On a Riemannian configuration manifold, the same structure is

$$\nabla_{\dot q}\dot q+\operatorname{grad}P=f^\sharp.$$

The kinetic energy defines a metric $\kappa$, and $\sharp:T^*Q\to TQ$ converts covectors to vectors using that metric. Its inverse is $\flat$. The Levi-Civita connection $\nabla$ encodes the inertial terms geometrically.

A proposed trajectory is dynamically feasible only if its required generalized force lies in $\operatorname{im}B$. Equivalently, for a matrix $B^\perp$ spanning the left-null directions,

$$(B^\perp)^T(M\dot v+Cv+g)=0.$$

This condition is separate from actuator bounds. An algebraically reconstructible trajectory may still demand excessive thrust, speed, torque, or joint motion.

### 2.4 Differential flatness and its limits

A flat output $y$ provides finite-derivative reconstructions

$$x=\alpha(y,\dot y,\ldots,y^{(r)}),\qquad u=\beta(y,\dot y,\ldots,y^{(r)}).$$

The output itself must also be obtainable from finitely many derivatives of the original system variables, and its components must be differentially independent. For the systems here, the output dimension equals the independent input dimension. Configuration flatness is the special case $y=\varphi(q)$.

Flatness replaces differential equality constraints with algebraic reconstruction on regular domains. It does not remove inequality constraints, collisions, singularities, model errors, or contact-mode changes. It does not imply that an arbitrary physically convenient output—especially the end-effector pose—is flat. “Globally defined output map” also does not mean “nonsingular reconstruction for every possible output derivative.”

### 2.5 Group actions, bundles, and three distinct uses of symmetry

A group action $\Phi_g(q)$ transforms configurations. In a principal bundle, this action is free and proper, and configurations with the same shape form a group orbit. A section $\sigma(s)$ chooses one representative configuration for each shape. A local trivialization writes

$$q=\Phi_g(\sigma(s)).$$

The source collection uses geometry in three different ways:

- **Flat-output construction:** select a section so the unactuated equations determine shape algebraically from group motion (P02).
- **Tracking control:** use a transitive action and a lifted reference to turn tracking into regulation on a homogeneous manifold (P05). The action need not be free.
- **Learning:** identify transformations preserving tracking dynamics and reward, then learn on the quotient rather than repeatedly learning equivalent states (P03).

The groups in these constructions are not automatically identical. In particular, the body-axis rotation in P02 and the gravity-preserving world rotation used in tracking problems play different roles.

### 2.6 Contact-control vocabulary

An impedance specifies force as a function of motion error; a virtual mass-spring-damper relation is

$$M_d\ddot\delta+D_d\dot\delta+K_d\delta=f_{\mathrm{ext}}.$$

An **admittance** implementation integrates this equation to turn force measurements into motion commands for an inner tracker. A **direct force controller** instead feeds back force error. A **hybrid controller** assigns different directions to force and motion control. A **disturbance observer** estimates unexplained wrench, which is not necessarily the same as desired tool contact force.

For one independent axis,

$$\omega_n=\sqrt{k/m_d},\qquad \zeta=\frac{d}{2\sqrt{km_d}},\qquad \delta_\infty=f/k.$$

A low apparent mass can require the actuators to accelerate *with* an applied force. Compliance is therefore an actively controlled behavior, not merely weak position feedback.

## 3. Coordinate-free dynamics and task flatness, ICRA 2020

**Source:** P10, pp. 1–6; mathematical development in Sections II–V, Eqs. (1)–(57), simulation in Section VI. Page 7 is the bibliography.

### 3.1 System and the special mechanical-design condition

The system has three inertial bodies: quadrotor base $B$, first arm link, and second arm link. Each body has nonzero mass and rotational inertia. The end-effector frame $E$ is parallel to the second link's frame. The state representation uses $q=(g_{OB},\theta)$ and $v=(v_{OB},\omega_{OB},\dot\theta)$, with two joint coordinates. Inputs are thrust $f$, body moment $\mu\in\mathbb R^3$, and two joint torques $\tau$.

The **dynamic model** is not restricted by the special design condition. The **end-effector flatness proof** is: the combined system center of mass $S$ must be fixed in frame $E$,

$$\dot p_{ES}=0.$$

A sufficient physical construction places each joint's rotation axis through the combined center of mass of all bodies inboard of that joint. Simply attaching the first link at the quadrotor center of mass is insufficient for a general three-dimensional two-joint arm. The distinction separates a general modeling result from a narrower planning result.

### 3.2 Local derivation yielding global equations

To avoid the singularities of a fixed Euler-angle chart, the paper introduces a temporary chart around an arbitrary configuration $q_0$:

$$
\Phi(\phi;q_0)=\left(g_{OB,0}\begin{bmatrix}R_{\mathrm{loc}}(\phi_{4:6})&\phi_{1:3}\\0&1\end{bmatrix},\theta_0+\phi_{7:8}\right),
$$
$$R_{\mathrm{loc}}=\exp(\phi_4\widehat e_1)\exp(\phi_5\widehat e_2)\exp(\phi_6\widehat e_3).$$

The velocity transformation is

$$v=S(\phi)\dot\phi=\begin{bmatrix}R_{\mathrm{loc}}^T\dot\phi_{1:3}\\(R_{\mathrm{loc}}^T\dot R_{\mathrm{loc}})^\vee\\\dot\phi_{7:8}\end{bmatrix}.$$

After differentiating in local coordinates, every expression is evaluated at $\phi=0$. Because the center $q_0$ was arbitrary, the resulting equations are globally expressed in pose and quasi-velocity. The local chart is a derivation device, not a global state parameterization.

### 3.3 Mass matrix from body Jacobians

For each body $i$, compute its body-expressed twist

$$V_{Oi}=J_iv,$$
$$v_{Oi}=R_{Bi}^T(v_{OB}+\dot p_{Bi}+\widehat\omega_{OB}p_{Bi}),$$
$$\omega_{Oi}=\left(R_{Bi}^T\widehat\omega_{OB}R_{Bi}+R_{Bi}^T\dot R_{Bi}\right)^\vee.$$

Because these quantities are linear in $v$, the coefficients give $J_i$. With spatial inertia at the body's own center of mass,

$$\mathcal I_i=\operatorname{diag}(m_iI_3,J_i^{\mathrm{rot}}),\qquad M=\sum_{i\in\{B,1,2\}}J_i^T\mathcal I_iJ_i.$$

Here the Jacobian $J_i$ and the rotational inertia $J_i^{\mathrm{rot}}$ are deliberately distinguished; the source's notation can otherwise be confusing. This sum includes all cross-coupling automatically. The gravitational potential is

$$U(q)=\sum_i m_i g e_3^Tp_{Oi},\qquad G=\left.S(\phi)^{-T}\frac{\partial U(\Phi(\phi;q_0))}{\partial\phi}\right|_{\phi=0}.$$

The Coriolis matrix, source Eq. (21), contains both inertia derivatives and the non-coordinate velocity-basis correction:

$$
C_{ij}=\sum_k\left.\left(\frac{\partial M_{ij}}{\partial\phi_k}-\frac12\frac{\partial M_{jk}}{\partial\phi_i}\right)\right|_0v_k
+\sum_{k,s}\left.\left(\frac{\partial S_{si}}{\partial\phi_k}-\frac{\partial S_{sk}}{\partial\phi_i}\right)M_{sj}\right|_0v_k.
$$

Dropping the second term and using an ordinary coordinate Christoffel formula would generally be wrong for these quasi-velocities. Virtual work gives the input map

$$B=\begin{bmatrix}0_{2\times6}\\I_6\end{bmatrix},$$

consistent with the ordering $(v_x,v_y,v_z,\omega_x,\omega_y,\omega_z,\dot\theta_1,\dot\theta_2)$: body lateral translations have no direct inputs.

### 3.4 Inertial decomposition

The transformed velocity is

$$\xi=Tv=(\dot x_S,\omega_{OB},\rho),$$

where $\rho$ is a dynamically decoupled generalized arm velocity, not simply $\dot\theta$. The first block $T_S$ is the center-of-mass Jacobian and $T_\omega$ extracts body angular velocity. For distinct blocks $\alpha,\beta$, impose

$$T_\alpha M^{-1}T_\beta^T=0.$$

Choose rows of $Z$ as a basis of the common nullspace of the stacked $T_S,T_\omega$ constraints, then

$$T_\rho=(ZMZ^T)^{-1}ZM.$$

The transformed equations are

$$\Lambda\dot\xi+\Gamma\xi+N=\bar Bu,$$

with, by direct substitution,

$$\Lambda=T^{-T}MT^{-1},\quad \Gamma=T^{-T}(CT^{-1}-MT^{-1}\dot TT^{-1}),\quad N=T^{-T}G,\quad\bar B=T^{-T}B.$$

The explanatory expansion above makes explicit the transformation hidden in the source's compact notation. The translational block is

$$m\ddot x_S+mge_3=fR_{OB}e_3.$$

The remaining blocks govern base rotation and $\rho$. Their inertia is block diagonal, but velocity-dependent coupling remains. “Inertially decoupled” does not mean all dynamic interactions vanish.

### 3.5 Reconstruction from the end-effector pose

The proposed output is $Y=g_{OE}\in SE(3)$. Because $p_{ES}$ is fixed,

$$x_S=p_{OE}+R_{OE}p_{ES}.$$

Let $b_i$ be the columns of $R_{OB}$, $a=\ddot x_S+ge_3$, and choose the positive-thrust branch. Then

$$f=m\|a\|,\quad b_3=a/\|a\|,\quad \dot f=mb_3^Tx_S^{(3)}.$$

The lateral angular rates and accelerations follow from differentiating $fb_3=ma$:

$$\begin{bmatrix}\omega_1\\\omega_2\end{bmatrix}=\frac m f\begin{bmatrix}-b_2^T\\b_1^T\end{bmatrix}x_S^{(3)},$$
$$\begin{bmatrix}\dot\omega_1\\\dot\omega_2\end{bmatrix}=\frac m f\begin{bmatrix}-b_2^T\\b_1^T\end{bmatrix}x_S^{(4)}-2\frac{\dot f}{f}\begin{bmatrix}\omega_1\\\omega_2\end{bmatrix}+\omega_3\begin{bmatrix}\omega_2\\-\omega_1\end{bmatrix}.$$

For the illustrated geometry, define $z=(m/f)R_{OE}^Ta$. The orientation constraint becomes

$$z=\begin{bmatrix}-\cos\theta_1\sin\theta_2\\\sin\theta_1\\\cos\theta_1\cos\theta_2\end{bmatrix}.$$

On the stated collision-free branch $\theta_1\in(-\pi/2,\pi/2)$, $\theta_2\in(0,\pi)$,

$$\theta_2=\operatorname{atan2}(-z_1,z_3),\qquad \theta_1=\operatorname{atan2}(z_2\sin\theta_2,-z_1).$$

Then $R_{OB}=R_{OE}(R_{B1}R_{12})^T$. Differentiating this relation yields

$$\omega_{OB}=A_\omega(\theta)\omega_{OE}+A_\theta(\theta)\dot\theta.$$

Its first two rows determine joint rates:

$$\dot\theta=\operatorname{diag}(1,1/\cos\theta_1)\left(\begin{bmatrix}e_1^T\\e_2^T\end{bmatrix}A_\omega\omega_{OE}-\begin{bmatrix}\omega_1\\\omega_2\end{bmatrix}\right).$$

Differentiating again gives the analogous solve for $\ddot\theta$ from $\dot\omega_{1:2}$, $\dot A_\omega\omega_{OE}+A_\omega\dot\omega_{OE}+\dot A_\theta\dot\theta$. The third row recovers $\omega_3$ and $\dot\omega_3$. Finally $\dot\xi=\dot Tv+T\dot v$ recovers $\dot\rho$.

Partition the transformed rotational input matrix as $[B_f\ B_{\mu\tau}]$. Required moments and joint torques are

$$\begin{bmatrix}\mu\\\tau\end{bmatrix}=B_{\mu\tau}^{-1}\left(\begin{bmatrix}\Lambda_2&0\\0&\Lambda_3\end{bmatrix}\begin{bmatrix}\dot\omega_{OB}\\\dot\rho\end{bmatrix}+\begin{bmatrix}\Gamma_2\\\Gamma_3\end{bmatrix}\xi-fB_f\right).$$

The source shows $B_{\mu\tau}$ is full-rank upper triangular. A nearby explanatory sentence calls the wrong pair of subsystems fully actuated; the equations show that the rotational and generalized-arm blocks are the fully actuated ones, while translation remains underactuated.

### 3.6 Conditions, validation, and boundaries

Fourth derivatives of the pose output appear, so $C^4$ task trajectories are required by this reconstruction. It fails at zero required thrust and at physical manipulation singularities such as $\cos\theta_1=0$. The latter is a loss of manipulability, not a removable Euler-coordinate singularity. Self-collision and actuator bounds still require explicit checking.

MATLAB symbolic tools generate the dynamics and decomposition. An aerial-videography simulation orbits a subject while preserving desired camera pointing and horizon orientation. Open-loop integration over 15 s gives reported translational RMSE $3.3\times10^{-8}$ m and angular RMSE $2.7\times10^{-5}$ rad. These are numerical consistency results for matching model and feedforward inputs, not robustness or hardware-tracking measurements. Figures 2–4 show the task, reconstructed states, and controls.

The paper suggests feedback linearization for the fully actuated arm block and cascaded/backstepping control for base rotation and translation, but does not supply a complete experimental closed-loop implementation. If the center-of-mass condition is removed, task inversion becomes a nonlinear ODE rather than the demonstrated algebraic reconstruction. P08 develops that generalization.

### 3.7 Reconstruction order, derivative requirements, and verification

An implementation of the flatness construction must respect its dependency order. First compute the center-of-mass trajectory from the desired end-effector pose and its fixed offset. Its acceleration determines thrust magnitude and direction. The orientation constraint then determines the two joint angles on a selected regular branch and hence base attitude. Jerk supplies the first two base angular rates. Differentiated orientation kinematics supplies joint rates and the remaining angular rate. Snap supplies the first two angular accelerations, after which the differentiated kinematic relation supplies joint accelerations and the last angular acceleration. The transformed velocity equations finally determine the required body and joint moments.

The angular-rate identities used in that process are

$$\widehat\omega=R^T\dot R,\qquad
\widehat{\dot\omega}=R^T\ddot R-\widehat\omega^2.$$

The second expression must be skew-symmetric for a consistent differentiable rotation trajectory. Entrywise interpolation of rotation matrices generally fails this check unless the interpolation preserves $SO(3)$. Likewise, independent polynomial interpolation of quaternion components requires normalization and derivative handling. These are reconstruction consequences, not additional interpolation algorithms supplied by P10.

The center-of-mass identity requires derivatives of the rotated offset, not merely derivatives of end-effector position. A fixed $p_{ES}$ in the tool frame still moves in the world when $R_{OE}$ changes. Thus changing camera orientation can demand translational acceleration and rotor thrust even if the camera origin is stationary.

The source's statement following Eq. (7) prints differentiation with respect to the chart coordinate when obtaining $S$. Since $v=S(\phi)\dot\phi$, the required Jacobian is $\partial v/\partial\dot\phi$. The rotational inertia notation in Eq. (18) should likewise be interpreted as each body's own inertia, as explicitly distinguished in Section 3.3. These notation issues do not alter the kinetic-energy construction.

Useful algebraic checks are $M=M^T\succ0$, $\Lambda=T^{-T}MT^{-1}$, the absence of off-diagonal inertial blocks claimed by the decomposition, and equality of generalized power before and after the velocity transformation. At each reconstructed sample, the residual $M\dot v+Cv+G-Bu$ should vanish in the ideal model. Small open-loop simulation error checks that these formulas and numerical integration agree; it does not test feedback robustness. Required thrust must also remain nonzero, the selected joint branch regular, and commanded inputs within physical limits. These last inequalities are additional feasibility checks beyond algebraic flatness.

## 4. Dynamically feasible task-space planning, RA-L 2021

**Source:** P08, pp. 2–7, Eqs. (1)–(45); simulations and discussion on pp. 7–8.

### 4.1 General model and center-of-mass coordinates

For an arbitrary $n$-joint arm, choose $q=(x,R,\theta)$ with $x$ the **whole-system** center of mass. Each body pose is

$$x_i=x+Rp_i(\theta),\qquad R_i=RQ_i(\theta),\qquad mx=\sum_{i=0}^nm_ix_i.$$

Starting from ordinary base-origin forward kinematics, the mass-weighted identity solves for the base position and produces these functions. The resulting equations have the sparse form

$$
\begin{bmatrix}mI_3&0\\0&M_{\rm rot}\end{bmatrix}\dot v+
\begin{bmatrix}0&0\\0&C_{\rm rot}\end{bmatrix}v+
\begin{bmatrix}mge_3\\0\end{bmatrix}=
\begin{bmatrix}Re_3&0\\B_{\rm rot}&I_{3+n}\end{bmatrix}u.
$$

$M_{\rm rot}$ and $B_{\rm rot}$ depend on $\theta$; $C_{\rm rot}$ depends on $\theta,\dot\theta,\Omega$. A change of rotational input absorbs the direct thrust coupling $B_{\rm rot}f$. The complete system is a fully actuated rotational/joint subsystem feeding thrust direction to an underactuated center-of-mass subsystem. Unlike P10, no special arm geometry is required for this flatness result.

### 4.2 Attitude fiber bundle and flat coordinates

Project attitude onto thrust direction: $p(R)=Re_3=s\in S^2$. The remaining freedom is rotation about $s$, a circle. This is the nontrivial bundle $S^1\hookrightarrow SO(3)\to S^2$, related to the Hopf fibration by the quaternion double cover.

On $S^2\setminus\{-e_3\}$ define

$$H_1(\psi)=\begin{bmatrix}\cos\psi&-\sin\psi&0\\\sin\psi&\cos\psi&0\\0&0&1\end{bmatrix},$$
$$H_2(s)=\begin{bmatrix}1-\frac{s_1^2}{1+s_3}&-\frac{s_1s_2}{1+s_3}&s_1\\-\frac{s_1s_2}{1+s_3}&1-\frac{s_2^2}{1+s_3}&s_2\\-s_1&-s_2&s_3\end{bmatrix}.$$

Then $R=H_2(s)H_1(\psi)$. The flat output is

$$y=(x,\theta^+),\qquad\theta^+=(\psi,\theta)\in\mathbb T^{n+1}.$$

Here $\psi$ is a rotation around the thrust direction supplied by this section, not a generic Euler yaw. The output dimension is $3+(n+1)=4+n$, matching the input dimension. The north/south chart boundary is a topological issue; another section can cover the excluded direction.

### 4.3 Explicit flatness reconstruction

Projection onto the unactuated directions gives

$$s\times(\ddot x+ge_3)=0.$$

Let $a=\ddot x+ge_3$. On the positive-thrust branch,

$$s=\frac a{\|a\|},\qquad
\dot s=\frac{(I-ss^T)x^{(3)}}{\|a\|},$$
$$\ddot s=\frac{(I-ss^T)x^{(4)}-(2\dot ss^T+s\dot s^T)x^{(3)}}{\|a\|}.$$

Use the section and its derivatives to recover $R,\Omega,\dot\Omega$, and use the supplied joint trajectories for $\theta,\dot\theta,\ddot\theta$. Then

$$u=B^\dagger(M\dot v+Cv+g),\qquad B^\dagger=(B^TB)^{-1}B^T$$

when $B$ has full column rank. This pseudoinverse gives an exact dynamics solution because the unactuated feasibility constraint was already imposed. Required smoothness is $x\in C^4$ and $\theta^+\in C^2$.

### 4.4 Why task inversion is a separate problem

For zero, one, and two joints the proposed task spaces are respectively

$$\mathcal T_0=\mathbb R^3\times S^1,\quad \mathcal T_1=\mathbb R^3\times S^2,\quad \mathcal T_2=SE(3).$$

The task specifies end-effector position plus yaw, a pointing axis, or full attitude. Adjacent parallel joint axes require a different choice, and tasks for $n>2$ are application-dependent; the paper does not give a universal higher-redundancy task parameterization.

For a task map $\tau(q)=t$, each regular region $Q_i$ has a self-motion family $q=\sigma_\tau^i(t;s)$. Thrust direction $s$ parameterizes its two configuration degrees of freedom. The wrist-like inverse kinematics is:

- Two joints: solve $H_2(s)^TR_E=H_1(\psi)Q_E(\theta)$ for three virtual/physical angles.
- One joint: solve $H_2(s)^Ta_E=H_1(\psi)Q_E(\theta)\tilde a_E$ for two angles.
- Zero joints: the task provides $\psi$ directly.

In all cases, $x=x_E-Rp_E(\theta)$. Different regular regions produce the familiar discrete inverse-kinematics branch multiplicity.

### 4.5 Coupled kinematic/dynamic constraints reduce to a shape ODE

The exact problem is the DAE

$$\tau(\sigma_\pi(s;y))=t,\qquad s\times(\ddot x+ge_3)=0.$$

Define the center-of-mass map induced by task inversion,

$$\chi(t;s)=\operatorname{proj}_x\big(\varphi(\sigma_\tau^i(t;s))\big).$$

Substitution gives

$$s\times\left(ge_3+\frac{d^2}{dt^2}\chi(t;s)\right)=0,\qquad s^Ts=1.$$

The $t$ argument of $\chi$ denotes the task output, whose own time variation contributes through the chain rule; it is not merely an explicit scalar-time dependence. Expanding the derivative involves task accelerations, shape accelerations, and quadratic mixed velocity terms. The sphere constraint gives $s^T\dot s=0$ and $s^T\ddot s=-\|\dot s\|^2$, supplying the radial acceleration equation needed alongside the two independent cross-product constraints.

Numerical integration uses two antipodal stereographic charts to improve constraint accuracy. The continuous solution family is parameterized by $(s_0,\dot s_0)\in TS^2$, a **four-dimensional** space, subject to regularity and existence conditions. It is not an unrestricted trajectory search in all joint and base coordinates.

### 4.6 Low-dimensional trajectory optimization

The shooting optimization selects initial shape state:

$$\min_{(s_0,\dot s_0)\in TS^2}\int_0^T J(s^\star,\dot s^\star,\ddot s^\star)\,dt.$$

The example $J=\|\ddot s\|^2$ discourages rapid thrust-direction changes and resembles a minimum-snap objective because $\ddot s$ involves $x^{(4)}$. It is a surrogate, not an exact motor-energy expression. The paper reports convergence in seconds on a standard laptop using numerical integration and gradient descent. Its “anytime” property means each valid integrated candidate already satisfies the modeled task and dynamics, even before cost optimality. It does not imply candidates automatically satisfy omitted actuator or collision constraints.

### 4.7 Zero dynamics, finite-time escape, and stable geometry

Even a constant task can excite self-motion. In general, the shape ODE contains terms quadratic in $\dot s$, so bounded task commands alone do not automatically preclude finite-time escape.

A useful geometry satisfies

$$\frac{\partial\chi}{\partial s}=-\lambda I_3,\qquad \lambda\ne0\text{ constant}.$$

For a fixed task,

$$s\times(ge_3-\lambda\ddot s)=0,$$
$$\ddot s=-\frac g\lambda\widehat s^2e_3-\|\dot s\|^2s.$$

Define

$$H(s,\dot s)=-\frac g\lambda e_3^Ts+\frac12\|\dot s\|^2.$$

Direct differentiation using $s^T\dot s=0$ gives $\dot H=0$. Since the potential term is bounded on the sphere, finite energy bounds shape velocity and prevents finite-time escape. The equilibrium $s=\operatorname{sgn}(\lambda)e_3,\dot s=0$ is the energy minimum; the antipodal equilibrium is unstable. Conservation gives stability, **not asymptotic convergence**. The paper's wording about global Lyapunov stability should not be read as global attraction in this conservative system with another equilibrium.

The design uses concurrent physical and virtual joint axes, with their intersection and arm-link centers of mass determined by task outputs. Then $\lambda=(m_0/m)\delta$, where $\delta$ is the signed thrust-axis distance from vehicle center to the intersection. Positive $\lambda$ favors upright hover and avoids unnecessary inverted/zero-thrust regimes. A footnote sketches extension of the no-finite-escape result to time-varying tasks with bounded derivatives via a power bound and comparison principle; the complete derivation is not printed.

### 4.8 Task flatness is the zero-shape-dependence case

If

$$\frac{\partial\chi}{\partial s}=0,$$

the center of mass is determined by the task alone. The shape equation becomes algebraic, and the task is itself a local flat output. The continuous internal-dynamics family collapses to discrete reconstruction branches.

For two joints, the whole-system center of mass must be fixed in the end-effector frame. For one joint it must additionally lie on the specified end-effector axis. For zero joints it must coincide with the end-effector origin. Inboard combined centers of mass lying on each joint axis supply a construction. These conditions recover P10 and related special cases. Task flatness also ensures periodic task trajectories reconstruct periodically on a consistently chosen regular branch; ordinary flatness alone does not remove task-space internal drift.

### 4.9 Simulations and scope

The examples are dynamic grasping of an object moving on a curved path with a body-fixed gripper, spray painting a curved elevated surface with one joint, and six-dimensional pick-and-place with two joints. Figures 1–3 establish geometry, cascade structure, and task spaces; Figure 4 shows multiple feasible solutions ordered by cost; Figure 5 contrasts stable-internal-dynamics and task-flat geometries; Figure 6 shows tasks.

The validation is simulation and open-loop consistency with the derived model. Contact-force exchange is excluded, with small-object or noncontact tasks offered as motivation. The paper explicitly leaves substantial contact wrenches, contact-mode transitions, closed-loop hardware execution, and their effect on dynamic feasibility for future work.

### 4.10 Explicit shape-acceleration solve and numerical planning procedure

To make the chain rule in Eq. (36) explicit, write the prescribed task in local coordinates as $\eta(t)$ and the associated center of mass as $x=\chi(\eta,s)$. Locally,

$$\ddot x=\chi_s\ddot s+b,$$
$$b=\chi_\eta\ddot\eta+\chi_{\eta\eta}[\dot\eta,\dot\eta]
+2\chi_{\eta s}[\dot\eta,\dot s]+\chi_{ss}[\dot s,\dot s].$$

These are ordinary coordinate derivatives of the local inverse-kinematic map; their individual values depend on coordinates, but the reconstructed physical trajectory does not. Choose a $3\times2$ orthonormal tangent basis $E(s)$ with $E^Ts=0$. The cross-product constraint is equivalent to $E^T(ge_3+\ddot x)=0$. Together with the twice-differentiated sphere constraint, it gives the explanatory local solve

$$\begin{bmatrix}E^T\chi_s\\s^T\end{bmatrix}\ddot s
=\begin{bmatrix}-E^T(ge_3+b)\\-\|\dot s\|^2\end{bmatrix}.$$

The $3\times3$ matrix must be nonsingular. This is a dynamic-inversion regularity condition in addition to regular inverse kinematics. A chart singularity can be removed by switching sphere coordinates; a genuinely singular physical inversion cannot. The source describes solving this acceleration equation and integrating in two stereographic charts, rather than printing this particular matrix notation.

A practical reading of its shooting method is: choose a discrete inverse-kinematic branch and initial $(s_0,\dot s_0)$; integrate the shape ODE over the task interval; reconstruct $y=(x,\psi,\theta)$; recover state and control from its derivatives; evaluate the cost; and improve the four-dimensional initial condition. Trajectories that encounter a singularity or fail to exist over the full interval are not valid candidates. If actuator, collision, joint-limit, or terminal-state constraints are added, they must also be checked during shooting. The article's reported objective and anytime claim do not automatically enforce those extra constraints.

The natural initial-condition domain is $TS^2$, not four unrelated Euclidean entries. In ambient coordinates it obeys $s_0^Ts_0=1$ and $s_0^T\dot s_0=0$. Local coordinates or a retraction can enforce those conditions. The source does not specify a complete optimizer tolerance, chart-switch policy, or derivative implementation sufficient to reproduce every timing result.

### 4.11 Internal-energy check and the task-flat limit

For $\chi_s=-\lambda I$ and a constant task, differentiating the stated energy gives

$$\dot H=-\frac g\lambda e_3^T\dot s+
\dot s^T\left[\frac g\lambda(I-ss^T)e_3-\|\dot s\|^2s\right]=0.$$

The cancellation uses $s^T\dot s=0$. Since $e_3^Ts\in[-1,1]$, finite initial energy bounds $\|\dot s\|$. Smooth dynamics on the compact sphere with bounded velocity then avoid the quadratic-velocity finite-time escape feared in the general case. A stationary energy minimum is stable, but undamped motions can persist. This explains why the planner may benefit from initial-condition optimization even for a simple task.

When $\lambda\to0$, the displayed ODE coefficients become singular. At exactly $\chi_s=0$, the correct interpretation is a different algebraic reconstruction: task acceleration determines $s$ directly. It is not valid to simulate the $1/\lambda$ equation with $\lambda=0$. This distinction separates the task-flat geometry from the nonzero-offset geometry with benign internal dynamics. For time-varying tasks, energy can be injected by the reference; bounded task derivatives and the source's comparison argument are needed instead of the constant-task conservation claim.

## 5. Symmetry and geometric flat outputs

**Source:** P02, pp. 2–6, Definitions 1–15, Propositions 1–4, Theorems 1–2, Eqs. (1)–(30). This source explains why the preceding flat outputs can be constructed from mechanics rather than guessed.

### 5.1 Principal-bundle setup

A principal bundle consists of a configuration manifold $Q$, smooth free proper group action $\Phi:G\times Q\to Q$, and quotient projection $\pi:Q\to S$. A section $\sigma:U\subseteq S\to Q$ obeys $\pi\circ\sigma=\mathrm{id}$. Its canonical trivialization is

$$q\longmapsto(\pi(q),\varphi(q)),\qquad q=\Phi_{\varphi(q)}(\sigma(\pi(q))),$$

with equivariance $\varphi(\Phi_g(q))=g\varphi(q)$. The vertical subspace is $VQ=\ker T\pi$. A principal connection selects a smooth equivariant complementary horizontal subspace $HQ$. For the section-induced **flat connection**,

$$HQ=\ker T\varphi.$$

This is not necessarily the mechanical connection used in momentum reduction. Its special role is removing shape-derivative dependence in the unactuated equations.

### 5.2 Symmetry, broken symmetry, and the unactuated distribution

The mechanical system has Lagrangian $L=K-P$, with $K(v)=\tfrac12\langle\!\langle v,v\rangle\!\rangle$, and control codistribution $F\subseteq T^*Q$. Full symmetry preserves $L$ and $F$; broken symmetry preserves the kinetic metric and $F$ but need not preserve potential. Gravity frequently breaks potential symmetry without spoiling the construction.

The unactuated subbundle is

$$U_qQ=\{X\in T_qQ:\langle f,X\rangle=0\text{ for every }f\in F_q\}.$$

Projecting the forced geodesic equation gives implicit feasibility:

$$\langle\!\langle X,\nabla_{\dot q}\dot q+\operatorname{grad}P\rangle\!\rangle=0\quad\forall X\in\Gamma(UQ).$$

Define the computable underactuation distribution

$$\Delta=\operatorname{span}\{X,\nabla_YX:X\in\Gamma(UQ),Y\in\Gamma(TQ)\}.$$

It includes how unactuated directions change over configuration, not merely the instantaneous nullspace of the input matrix. Under the symmetry assumptions it is equivariant.

### 5.3 The crucial shape-derivative elimination

Write $\dot q=\xi^aV_a+\dot s^\alpha H_\alpha$ in vertical and horizontal bases. Then

$$\nabla_{\dot q}\dot q=\dot\xi^aV_a+\ddot s^\alpha H_\alpha+\xi^a\xi^b\nabla_{V_a}V_b+2\xi^a\dot s^\beta\nabla_{V_a}H_\beta+\dot s^\alpha\dot s^\beta\nabla_{H_\alpha}H_\beta.$$

The mixed terms combine because the horizontal lifts commute with the appropriate vertical generators. Initially the implicit equations have the form $E(s,\dot s,\ddot s;g,\xi,\dot\xi)=0$.

Proposition 4 states that they are independent of shape derivatives exactly when $HQ\perp\Delta$ in the kinetic metric. Metric compatibility explains the mechanism:

$$\langle\!\langle\nabla_YH,X\rangle\!\rangle=Y\langle\!\langle H,X\rangle\!\rangle-\langle\!\langle H,\nabla_YX\rangle\!\rangle=0.$$

All projected coefficients multiplying $\dot s$ or $\ddot s$ vanish. The remaining equation is

$$E(s;g,\xi,\dot\xi)=0.$$

### 5.4 Necessary condition and sufficient theorem

A **geometric flat output** is a flat output $y:Q\to G$ whose pairing with $\pi$ is a bundle trivialization. Necessarily,

$$\dim G=\operatorname{rank}F.$$

A flat output must have the independent-input dimension. Choosing a different subgroup can change whether this dimension test holds.

The main sufficient theorem assumes this equality and a section satisfying:

1. **Orthogonality:** its tangent image is perpendicular to $\Delta$.
2. **Regularity:** $\partial E/\partial s$ has full shape rank at generic points satisfying the implicit dynamics.

Equivariance transports section orthogonality along every fiber. Therefore the canonical connection eliminates shape derivatives. Because $\dim S=\operatorname{rank}UQ$, the shape Jacobian is square. The implicit function theorem gives $s=s(g,\xi,\dot\xi)$ locally. In the canonical flat connection $\xi=\dot gg^{-1}$, so shape depends on $g,\dot g,\ddot g$. Reconstruct $q$, differentiate twice, and use the full mechanical equation to recover force. Thus the group variable is flat.

“Generic” means an open dense set in the relevant smooth-trajectory topology, not every trajectory. Reconstruction singularities occur in derivative tuples such as $(q,\dot q,\ddot q)$, not just configuration. The paper also relaxes full broken-symmetry assumptions to equivariance of $\Delta^\flat$. Orthogonality is typically the harder construction problem; regularity still must be checked.

### 5.5 Example: planar rocket

With $q=(x_1,x_2,\theta)$,

$$M=\operatorname{diag}(m,m,J),\quad P=mgx_2,\quad F=\operatorname{span}\begin{bmatrix}\cos\theta&-\sin\theta\\\sin\theta&\cos\theta\\r&0\end{bmatrix}.$$

Translations give $G=\mathbb R^2$, shape $S^1$. The distributions are

$$UQ=\operatorname{span}\begin{bmatrix}-r\cos\theta\\-r\sin\theta\\1\end{bmatrix},\quad
\Delta=\operatorname{span}\begin{bmatrix}-r\cos\theta&r\sin\theta\\-r\sin\theta&-r\cos\theta\\1&0\end{bmatrix}.$$

An orthogonal section is $\sigma(\theta)=(\frac{J}{mr}\sin\theta,-\frac{J}{mr}\cos\theta,\theta)$. The resulting flat output is

$$y=\left(x_1-\frac{J}{mr}\sin\theta,\ x_2+\frac{J}{mr}\cos\theta\right).$$

It locates the center of oscillation. The offset is obtained from the kinetic metric, not chosen by visual intuition. This construction presumes the indicated nonzero moment arm $r$.

### 5.6 Example: planar aerial manipulator

The configuration is $SE(2)\times\mathbb T^1$, group $SE(2)$, shape the joint angle $\phi$. Coordinates $(x_1,x_2,\theta)$ describe the end-effector pose. The underactuated direction is the translation perpendicular to the vehicle thrust axis,

$$UQ=\operatorname{span}(-\sin(\phi+\theta),\cos(\phi+\theta),0,0)^T,$$

and $\Delta$ spans the two translational directions. An orthogonal section is

$$\sigma(\phi)=\left(\frac{\ell_qm_q}{m_g+m_q}\cos\phi,\frac{\ell_qm_q}{m_g+m_q}\sin\phi,0,\phi\right).$$

The group output is

$$y=(x_1+c_4\cos(\phi+\theta),x_2+c_4\sin(\phi+\theta),\theta),\quad c_4=-\frac{\ell_qm_q}{m_g+m_q}.$$

For $\ell_q=0$ this is exactly the end-effector pose; otherwise it is the pose of a parallel, shifted frame. The source gives the two-column $\Delta^\perp$ using $c_1=\ell_g(m_g+2m_q)$, $c_2=\ell_qm_q$, $c_3=m_g+m_q$:

$$\Delta^\perp=\operatorname{span}\begin{bmatrix}-c_1\sin\theta-2c_2\sin(\phi+\theta)&-c_2\sin(\phi+\theta)\\c_1\cos\theta+2c_2\cos(\phi+\theta)&c_2\cos(\phi+\theta)\\2c_3&0\\0&c_3\end{bmatrix}.$$

This explicitly verifies why Euclidean orthogonality of coordinate columns is not the correct test: the orthogonality is metric-dependent. A projection-map domain printed as $SE(3)$ in this planar example is inconsistent with the stated $SE(2)$ model and should be read in that planar context.

### 5.7 Example: quadrotor and the global obstruction

For $Q=SE(3)$, use translations and right multiplication of attitude by a body-thrust-axis rotation:

$$\Phi_{(a,\psi)}(R,x)=(R\operatorname{Rot}_{e_3}(\psi),x+a).$$

With axisymmetric rotational inertia, the kinetic metric has this broken symmetry. Shape is $s=Re_3$. The unactuated distribution is body lateral translation, while $\Delta$ spans all translations. A section with constant position and varying attitude is therefore metric-orthogonal to $\Delta$.

The north section is $H_2(s)$ from Section 4. A south section has rotation

$$H_S(s)=\begin{bmatrix}1+\frac{s_1^2}{s_3-1}&-\frac{s_1s_2}{s_3-1}&s_1\\\frac{s_1s_2}{s_3-1}&-1-\frac{s_2^2}{s_3-1}&s_2\\s_1&-s_2&s_3\end{bmatrix},\quad s\ne e_3.$$

Each section gives almost-global position/body-yaw outputs. No global section exists for this nontrivial bundle; an atlas and chart switching can cover the full manifold. The source notes that the obtained outputs remain valid without the axisymmetric-inertia assumption using the weaker equivariance condition.

### 5.8 Catalog and unresolved problems

Table I also includes a 3D two-joint aerial manipulator ($G=SE(3)$), a quadrotor with slung load ($G=\mathbb R^3\times S^1$, shape $S^2\times S^2$), and a chain of spring-mass systems ($G=\mathbb R$, shape $\mathbb R^{n-1}$). These are cataloged known geometric flat outputs, not all new applications of the sufficient theorem. Higher-order shape reconstruction in the latter examples motivates the gap between necessity and sufficiency.

The paper proposes numerical section search with basis functions, recursive treatment of shape as another bundle, and extension to nonholonomic systems. These are future directions. It does not present a general implemented flat-output discovery algorithm or a necessary-and-sufficient test for arbitrary robots.

---



### 5.9 Local algebra behind the theorem and section checks

To recover a usable control force from a configuration curve, first project the forced geodesic equation onto every unactuated direction. If those projections vanish, the remaining force covector lies in the control codistribution. When an input matrix has independent columns, its coefficients can then be solved uniquely; redundant actuator realizations require allocation. Thus the implicit equation is a dynamic-feasibility test, not the complete actuator command by itself.

The necessary dimension equality makes the number of shape coordinates equal the number of unactuated equations. Orthogonality removes shape derivatives from those equations; regularity makes their shape Jacobian invertible locally. Each ingredient has a separate role. A square Jacobian can still be singular. A regular equation involving $\ddot s$ defines internal dynamics rather than an algebraic flatness reconstruction. An orthogonal distribution need not integrate into a global section with the required topology.

For the planar rocket, a convenient scaled horizontal generator is

$$H=(J\cos\theta,J\sin\theta,mr)^T.$$

It is proportional to the derivative of the section in Section 5.5. Pairing it with each column of $\Delta$ using $\operatorname{diag}(m,m,J)$ gives zero: the translation and rotation contributions cancel. This explicitly verifies the metric dependence of the center-of-oscillation offset. The vertical basis is $\operatorname{span}\{(1,0,0)^T,(0,1,0)^T\}$.

For the planar manipulator, the source's vertical generators are the columns of

$$V=\begin{bmatrix}1&0&-x_2\\0&1&x_1\\0&0&1\\0&0&0\end{bmatrix}.$$

A horizontal generator equivalent to Eq. (27), scaled to avoid division by $\ell_qm_q$, is

$$H=\big(-\ell_qm_q\sin(\phi+\theta),\ell_qm_q\cos(\phi+\theta),0,m_g+m_q\big)^T.$$

At $\ell_q=0$ this remains a valid pure-shape direction even though the source's divided expression is undefined. The output then reduces to end-effector pose as stated. The complete kinetic matrix is not printed in P02, so the displayed metric-orthogonal basis is a source result rather than an independently reconstructed full multibody model here.

For both quadrotor section matrices, the third column is $s$, $H^TH=I$, and $\det H=1$ away from their excluded pole. On the overlap, $H_N(s)^TH_S(s)$ fixes $e_3$ and is therefore a yaw rotation. A chart change must update the yaw coordinate by this transition rotation to preserve physical attitude. The section singularity is distinct from the zero-thrust singularity in output derivatives.

The catalog's full structure is:

| System | Configuration | Group output | Shape | Bundle |
|---|---|---|---|---|
| Planar rocket | $SE(2)$ | $\mathbb R^2$ | $S^1$ | Trivial |
| Planar aerial manipulator | $SE(2)\times\mathbb T^1$ | $SE(2)$ | $\mathbb T^1$ | Trivial |
| Spatial two-joint aerial manipulator | $SE(3)\times\mathbb T^2$ | $SE(3)$ | $\mathbb T^2$ | Trivial |
| Quadrotor | $SE(3)$ | $\mathbb R^3\times S^1$ | $S^2$ | Nontrivial |
| Quadrotor with slung load | $SE(3)\times S^2$ | $\mathbb R^3\times S^1$ | $S^2\times S^2$ | Nontrivial |
| Chain of $n$ spring–mass systems | $\mathbb R^n$ | $\mathbb R$ | $\mathbb R^{n-1}$ | Trivial |

A globally defined output map still permits nongeneric derivative singularities. “Global geometric output” therefore does not mean every arbitrary output curve is physically realizable with bounded actuators.

## 6. Almost-global tracking on homogeneous manifolds

**Source:** P05.

The objective is tracking on a configuration manifold $Q$ that need not itself be a Lie group. The assumptions are a transitive action $\Phi:G\times Q\to Q$, a $\Phi$-invariant Riemannian metric $\kappa$, and full actuation:

$$\nabla_{\dot q}\dot q=\kappa^\sharp(f_q).$$

Full actuation permits cancellation of known additional forces and modification of the effective connection by feedback. The theorem does not directly apply to an arbitrary underactuated robot. The paper instead gives an underactuated satellite whose *reduced* pointing dynamics are fully actuated.

### 6.1 Lifted reference and intrinsic error

Choose an origin $0_Q$ and a lifted reference $g_d(t)$ satisfying $\Phi(g_d(t),0_Q)=q_d(t)$. Define

$$e(t)=\Phi(g_d(t)^{-1},q(t)),\qquad \dot e(t)\in TQ.$$

The zero error is the zero tangent vector **over** $0_Q$, so it specifies both correct configuration and correct velocity. With an invariant metric, configuration distance is preserved:

$$\operatorname{dist}_Q(q,q_d)=\operatorname{dist}_Q(e,0_Q).$$

For Lie groups, $e=g_d^{-1}g$. For spheres, $e=R_d^Tq$, where $R_d\in SO(n+1)$ lifts the sphere reference. A reductive splitting $\mathfrak g=\mathfrak f\oplus\mathfrak q$ separates the stabilizer algebra from horizontal motion and gives a unique horizontal lift for a chosen initial lift. A nontrivial bundle forbids a single global section, but it does not forbid a smooth lift along an individual reference curve.

The paper recommends integrating fiber coordinates in local trivializations rather than the entire group. This preserves the exact projection onto the reference curve even when numerical integration introduces fiber error. A finite time interval can be subdivided into finitely many chart intervals.

### 6.2 Navigation functions and closed-loop construction

A navigation function is a proper Morse function with one local minimum at the selected origin. Examples are

$$P_{S^n}(q)=-k_P0_{S^n}^Tq,$$
$$P_{SE(3)}(R,x)=\operatorname{tr}(K_R(I-R))+x^TK_xx,$$

with positive-definite gains and distinct eigenvalues of $K_R$ for the stated Morse construction. Positive-definite dissipation is represented by another metric $\nu$.

For fixed actual state $(q,\dot q)$, define the auxiliary curve and field

$$e_q(r)=\Phi(g_d(r)^{-1},q),\qquad X_{\dot q}(r)=d\Phi_{g_d(r)^{-1}}(\dot q).$$

The source's controller, Eq. (14), is

$$f_q=-d\Phi_{g_d^{-1}}^*\left[dP(e)+\nu^\flat(\dot e)+\kappa^\flat\left(\nabla_{\dot e_q}(\dot e_q+2X_{\dot q})\right)(t)\right].$$

The dummy variable $r$ matters: $q$ and $\dot q$ are held fixed while the reference moves. The proof uses the two-parameter family $E(r,s)=\Phi(g_d(r)^{-1},q(s))$ and its diagonal $e(t)=E(t,t)$. The covariant second derivative has reference, mixed, and actual-motion terms; the feedforward term cancels the first two. The resulting error dynamics are autonomous:

$$\nabla_{\dot e}\dot e=-\kappa^\sharp(dP(e)+\nu^\flat(\dot e)).$$

For energy $V=P(e)-P(0_Q)+\tfrac12\kappa(\dot e,\dot e)$,

$$\dot V=-\nu(\dot e,\dot e)\le0.$$

The navigation-function structure excludes convergence to unwanted critical points except on a measure-zero exceptional set. The theorem guarantees almost-global asymptotic tracking on the **whole tangent bundle**, plus local exponential convergence, assuming the lifted reference has bounded body velocity. That boundedness is used to infer actual state convergence from error-state convergence. “Almost global” is not global convergence from every initial condition.

### 6.3 Explicit specializations

On $S^n$, the source expresses the force as a row covector:

$$
f_q=-k_Pq_d^T(qq^T-I)-k_D(\dot q^T+q^T\dot R_dR_d^T)
+(q^T\ddot R_d+2\dot q^T\dot R_d)R_d^T(qq^T-I).
$$

Projection by $I-qq^T$ keeps force tangent to the sphere. On a Lie group, define $\xi_e=\xi-\operatorname{Ad}_{e^{-1}}\xi_d$ and $\zeta_P(e)=dL_e^*dP(e)$. Then source Eq. (28) is

$$\tau=-\zeta_P(e)-D^\flat\xi_e+I^\flat(\operatorname{Ad}_{e^{-1}}\dot\xi_d+[\xi,\xi_e])+\operatorname{ad}_{\xi_e}^*I^\flat\xi_e-\operatorname{ad}_\xi^*I^\flat\xi.$$

These are geometric feedforward-plus-potential-plus-damping controllers, not simple subtraction of arbitrary coordinate velocities.

Applications: an axisymmetric satellite with $J_1=J_2$, only two transverse torques, and zero initial axial angular velocity reduces to fully actuated pointing on $S^2$; an omnidirectional rigid aerial robot after gravity compensation is controlled on $\mathbb R^3\times SO(3)$. The illustrated results are numerical rollouts, not hardware trials. Cascade stability is discussed as a route to underactuated hierarchical control, not proved for every possible interconnection here.

### 6.4 What the tracking proof actually establishes

The proof has three logically separate parts: constructing a globally meaningful error, assigning its mechanical dynamics, and transferring convergence back to the moving physical trajectory. None can be replaced by simply subtracting coordinate vectors.

For a fixed time, the configuration map $q\mapsto\Phi(g_d^{-1},q)$ is a diffeomorphism. Its velocity map contains both the differential acting on actual velocity and the derivative of the moving reference action. Thus the state-error transformation is an affine transformation on each tangent fiber, not just $d\Phi_{g_d^{-1}}\dot q$. Its inverse reconstructs actual velocity from error velocity and the reference motion. This is why a zero configuration error with arbitrary velocity is not zero state error.

Write $D_r,D_s$ for covariant differentiation along the two directions of $E(r,s)$. Because the Levi-Civita connection is torsion-free, the mixed derivatives agree. Along the diagonal,

$$\nabla_{\dot e}\dot e=D_r\partial_rE+2D_r\partial_sE+D_s\partial_sE.$$

The first term is reference acceleration with the actual configuration held fixed. The second is the geometric counterpart of a moving-frame Coriolis term. The final term is actual acceleration transformed by the action. Invariance of the metric implies invariance of its Levi-Civita connection, so

$$D_s\partial_sE=d\Phi_{g_d^{-1}}(\nabla_{\dot q}\dot q).$$

The controller pulls error-space force covectors back into the physical cotangent space and cancels the first two terms. The resulting regulation system has no explicit reference dependence. Differentiating its kinetic energy uses metric compatibility:

$$\frac{d}{dt}\frac12\kappa(\dot e,\dot e)=\kappa(\nabla_{\dot e}\dot e,\dot e).$$

Potential-power terms cancel, leaving the negative dissipation shown in Section 6.2. Properness of the potential and positive kinetic energy keep energy sublevel sets bounded in the relevant manifold sense. The invariant zero-dissipation set consists of zero velocity at critical points of the potential. A Morse minimum has positive-definite Hessian, giving local exponential stability with damping. Other nondegenerate critical points have unstable directions; their stable manifolds form the exceptional set. The cited navigation-function result supplies an open dense basin of full measure. It does not say that every nonzero error decreases monotonically, nor that every initial condition reaches the reference.

Finally, the paper uses the Sasaki metric on $TQ$. A local tangent-bundle distance bound combines configuration distance with velocity mismatch after parallel transport along a short geodesic. The bounded body velocity of the lift controls the additional velocity term induced by the moving action. Without that hypothesis, a small configuration error multiplied by an arbitrarily fast reference motion need not imply a small physical velocity error. The excluded initial set is mapped back through the time-dependent error diffeomorphism for each initial time.

### 6.5 Computing a reference lift and matching a physical model

Let $F$ be the stabilizer of the origin. The projection $G\to Q$, $g\mapsto\Phi(g,0_Q)$, has fibers isomorphic to $F$. A reductive splitting $\mathfrak g=\mathfrak f\oplus\mathfrak q$ chooses a horizontal distribution $H_g=dL_g(\mathfrak q)$. A horizontal lift satisfies

$$\Phi(g_d,0_Q)=q_d,\qquad dL_{g_d^{-1}}\dot g_d\in\mathfrak q.$$

Given its initial value, it is unique. In a local section $\sigma:U\to G$, write $g_d=\sigma(q_d)f$, with $f\in F$. The horizontality equation determines an ODE for $f$. On leaving the chart, change section and transform the fiber coordinate so that $g_d$ remains continuous. Integration drift in $f$ changes which point of the fiber is chosen but does not change the projected reference, provided $f$ remains in $F$. This is the computational reason for integrating fiber coordinates instead of all group coordinates. A global smooth section is unnecessary for lifting a smooth curve.

For $S^n$, the stabilizer rotates the tangent $n$-plane while fixing the chosen pole; horizontal generators rotate the pole toward tangent directions. For a Lie group acting on itself, the stabilizer is trivial and the reference is its own lift. Different admissible lifts can yield different transient controls while retaining the theorem's qualitative convergence conclusion.

The physical mass metric need not be the invariant metric chosen for design. Under the explicit convention

$$\widetilde\nabla_{\dot q}\dot q=\widetilde\kappa^\sharp(u+F),\qquad
\nabla_{\dot q}\dot q=\kappa^\sharp f_v,$$

an explanatory matching law is

$$u=\widetilde\kappa^\flat\left[\kappa^\sharp f_v+(\widetilde\nabla-\nabla)(\dot q,\dot q)\right]-F.$$

The connection difference is a tensor, so this is coordinate-independent. It requires known model terms, full actuation, and the ability to produce the commanded force. A compact stabilizer is a sufficient condition for existence of an invariant metric; it does not make an unknown physical model known or remove actuator saturation.

### 6.6 Consistency checks, applications, and simulation evidence

On Euclidean space with a constant mass matrix $M$, the construction reduces to familiar computed torque,

$$u=M\ddot q_d-K_P(q-q_d)-K_D(\dot q-\dot q_d),$$

when the potential and damping are selected in these force units. This check explains the role of the geometric feedforward terms. For a stationary sphere reference, the tangent acceleration becomes

$$\nabla_{\dot q}\dot q=k_P(I-qq^T)q_d-k_D\dot q.$$

Its ambient realization is $\ddot q=k_P(I-qq^T)q_d-k_D\dot q-\|\dot q\|^2q$ for the unit-sphere metric. The last term enforces $q^Tq=1$; omitting it from an ambient simulation changes the system. The antipodal stationary state is an example of an excluded equilibrium.

For the Lie-group specialization, covariant differentiation in body coordinates uses

$$\nabla_I(u,v)=\frac12[u,v]-\frac12I^\sharp\big(\operatorname{ad}_u^*I^\flat v+\operatorname{ad}_v^*I^\flat u\big).$$

This identity, together with $\xi_e=\xi-\operatorname{Ad}_{e^{-1}}\xi_d$, produces the bracket and coadjoint terms in Eq. (28). Their signs depend on the selected error and trivialization; mixing them with another paper's inverse group error is invalid.

For the satellite, $J_3\omega_3$ is conserved. Zero initial axial rate allows pointing reduction to a mechanical system on $S^2$ with metric $J_1\rho$; a nonzero conserved spin would require the appropriate reduced gyroscopic dynamics. The theorem controls pointing, not an arbitrary independently specified axial angle. For the aerial robot, position and attitude form $\mathbb R^3\times SO(3)$ and gravity is compensated before applying the result. Figure 1 shows sampled sphere-state trajectories converging to a moving reference; Figure 2 shows the rigid aerial robot example. These illustrate the theorem and its implementation, but do not estimate the measure of the exceptional set experimentally, establish robustness to unmodeled aerodynamics, or certify a saturated implementation.

## 7. Symmetry-reduced reinforcement learning

**Source:** P03.

### 7.1 Tracking as one stationary MDP

Physical dynamics are $x_{t+1}\sim f(\cdot\mid x_t,u_t)$. Rather than train for one fixed time-indexed reference, define the MDP state

$$s_t=(x_t,x_t^d,u_t^d),\qquad a_t=u_t,$$

with

$$x_{t+1}^d\sim f(\cdot\mid x_t^d,u_t^d),\qquad u_{t+1}^d\sim\rho,$$
$$R(s,u)=-J_X(x,x^d)-J_U(u,u^d).$$

The reference-input distribution $\rho$ makes this a stationary stochastic MDP even when the physical robot model is deterministic. Training references and preplanned evaluation references are different distributions; generalization to the latter is demonstrated empirically, not guaranteed by the homomorphism theorem.

### 7.2 Exact abstraction and policy lifting

A continuous MDP homomorphism consists of a state projection $p$ and state-dependent action map $h_s$ satisfying

$$R(s,a)=\widetilde R(p(s),h(s,a)),$$
$$\tau(p^{-1}(\widetilde B)\mid s,a)=\widetilde\tau(\widetilde B\mid p(s),h(s,a)).$$

It preserves action values for lifted policies, and an optimal reduced policy lifts to an optimal original policy. This is an exact structural statement, not a guarantee that PPO finds the optimal policy.

For free proper state action $\Phi$ and compatible action transformation $\Psi$, the quotient $S/G$ is a smooth manifold. Choose an equivariant moving-frame map $\lambda:S\to G$ and set

$$p(s)=\Phi_G(s),\qquad h(s,a)=\Psi_{\lambda(s)^{-1}}(a).$$

A deterministic policy is lifted by

$$a=\Psi_{\lambda(s)}\big(\widetilde\pi(p(s))\big).$$

For stochastic policies the corresponding map is a pushforward of the reduced action distribution. The reward and transition law must be independent of the orbit representative. The paper's symmetry definition requires transition agreement on **invariant sets**, which is weaker than pointwise equivariance of every transition. Its tracking theorem permits the transformed next-state group element to differ from the present one, provided the transformation depends only on the current group/action transformation, not the particular state/action.

The appendix proves free/proper inheritance for product and diagonal actions using compact-preimage arguments. This supplies the geometric regularity needed for a smooth quotient. A measurable moving frame can be built from local trivializations; a continuous global one need not exist.

### 7.3 Three examples

**Particle.** Dynamics are $r^+=r+v\Delta t$, $v^+=v+u\Delta t/m$. Joint translation, velocity-shift, and input-shift symmetry reduces the 15-dimensional tracking state to the six-dimensional error $(r-r_d,v-v_d)$. The reduced input is $u-u_d$:

$$r_e^+=r_e+v_e\Delta t,\quad v_e^+=v_e+u_e\Delta t/m,\quad u=\widetilde\pi(r_e,v_e)+u_d.$$

The reward uses $\alpha(y)=c_r\|y\|+\tanh(a_r\|y\|)-1$ and penalties on velocity and input error. Ablations remove only position symmetry, then position and velocity symmetry, then all three.

**Astrobee.** State is pose and twist, action a six-dimensional wrench. Left $SE(3)$ pose symmetry yields

$$p(s)=(q^{-1}q_d,\xi,\xi_d,u_d).$$

The action map is identity. The paper's discrete model uses a pose exponential update and simplified translational/rotational velocity updates. It should be treated as the specified experimental model; the missing body-velocity transport term is identified in Section 7.5 and must be reconciled before hardware reuse.

**Quadrotor.** Gravity restricts symmetry to world translation and world yaw, $SE(2)\times\mathbb R$. The sufficient reduced observation is

$$p(s)=(q^{-1}q_d,R^Te_3,\xi,\xi_d,u_d).$$

The body-frame gravity direction must remain. Actual and desired angular velocities also remain separately: replacing both by their difference generally loses nonlinear gyroscopic information. Manifold dimension falls from 28 to 24; a matrix/quaternion implementation can have more scalar observation entries than intrinsic dimensions.

### 7.4 Experimental record

JAX environments and custom PPO are used with identical hyperparameters across variants of each environment. Ten training seeds and twenty preplanned trajectories are evaluated. Quadrotor trajectories are generated from flat-space Lissajous curves. Source Table I reports:

| System / reduction | Position RMS, cm | Velocity RMS, cm/s | Attitude RMS, rad | Rate RMS, rad/s |
|---|---:|---:|---:|---:|
| Particle baseline | 298.3 ± 1.2 | 103.0 ± 1.1 | — | — |
| Particle, translations | 10.0 ± 0.1 | 4.8 ± 0.1 | — | — |
| Particle, position + velocity shifts | 9.8 ± 0.2 | 4.3 ± 0.0 | — | — |
| Particle, full reduction | 8.3 ± 0.1 | 3.9 ± 0.2 | — | — |
| Astrobee baseline | 10.4 ± 1.0 | 6.0 ± 1.0 | 0.75 ± 0.01 | 0.24 ± 0.02 |
| Astrobee reduced | 1.3 ± 2.7 | 3.3 ± 3.1 | 0.37 ± 0.04 | 0.16 ± 0.06 |
| Quadrotor baseline | 66.3 ± 3.2 | 41.3 ± 2.5 | 0.48 ± 0.02 | 0.25 ± 0.02 |
| Quadrotor reduced | 28.4 ± 4.1 | 19.6 ± 3.1 | 0.25 ± 0.05 | 0.12 ± 0.01 |

Most particle improvement comes from translation reduction. The authors acknowledge that baseline reward engineering or retuning could improve performance. The PDF points to external code for implementation details; it does not print a complete PPO hyperparameter/reproduction specification. Obstacles, spatially varying wind, contact surfaces, or action bounds can break the proposed symmetries and must be included in any validity check for a new task.

### 7.5 Discrete-model and dimension audit

The Astrobee model uses body-twist configuration integration $q^+=q\exp(\hat\xi\Delta t)$ with $\xi=(\omega,v)$, but prints $v^+=v+(f/m)\Delta t$ without the body-coordinate transport term $-\omega\times v$. For physical body-frame linear velocity that term is required. This is a modeling/convention issue distinct from the symmetry theorem: the printed model can still possess the claimed symmetry, and the correctly transported left-invariant model can possess it as well.

The quadrotor model similarly prints a body-frame force/gravity update without that transport term. Its rotor moment map uses the components $\ell(u_1-u_3)$, $\ell(u_2-u_4)$, and $c(u_1-u_2+u_3-u_4)$. Any reimplementation must agree on rotor indexing, force signs, and velocity frames rather than borrowing equations individually.

The full tracking-MDP state dimensions reduce as follows: particle $15\to6$ under a nine-dimensional symmetry, Astrobee $30\to24$ under a six-dimensional symmetry, and quadrotor $28\to24$ under a four-dimensional symmetry. Particle partial reductions give 12 dimensions for position reduction and 9 for position-and-velocity reduction. These are intrinsic dimensions; observations embedding rotations or constrained variables can use more scalar entries.

The policy reconstruction maps and quotient maps require measurable choices. A statement about a local section does not automatically establish a globally smooth section. The reference-transition distribution and the theorem's weaker invariant-set condition also need to be treated as stated rather than replaced by a stronger, unproved distributional invariance requirement. A complete PPO hyperparameter/seed table is not supplied in the reviewed text, and no external code has been reviewed.

### 7.6 Why the quotient preserves decisions

A state quotient is valid only when representatives of the same orbit have the same reward and the same probability of entering every reduced-state event after corresponding actions. The reduced transition is defined by

$$\widetilde\tau(\widetilde B\mid p(s),\widetilde a)
=\tau\big(p^{-1}(\widetilde B)\mid s,\Psi_{\lambda(s)}\widetilde a\big).$$

To show this is well-defined, choose another representative $s'=\Phi_gs$. Equivariance gives $\lambda(s')=g\lambda(s)$, so its reconstructed action is $\Psi_g\Psi_{\lambda(s)}\widetilde a$. The event $p^{-1}(\widetilde B)$ is invariant. The symmetry condition therefore makes the transition probability identical. Reward invariance supplies the same argument for $\widetilde R$.

For a lifted policy, equality of one-step rewards and reduced transitions implies equality of finite-horizon returns by induction. Discounted limits then give equality of action values under the hypotheses of the cited homomorphism result. The optimal-policy statement concerns the mathematical MDPs. Finite network capacity, optimization error, training distribution, and numerical integration remain separate sources of approximation.

In the tracking construction the actual and reference next-state laws are multiplied with the independently sampled next reference-input law $\rho$. A group transformation can shift the two next physical states by a new common group element $k'$. Since reduced events are unchanged by that common transformation, the proof leaves the fresh draw from $\rho$ untouched. Consequently, the theorem does not require $\rho$ to be invariant under every reference-input transformation. Replacing this argument with ordinary equivariance of the full transition kernel would impose a stronger condition and miss the particle's input-shift reduction.

For the particle, a position shift, velocity shift, and input shift propagate into a different common position/velocity shift one step later. Subtraction cancels all these common terms, yielding exactly the two error updates in Section 7.3. The policy is a learned residual force plus the current desired force. Because a bounded actuator box is not invariant under arbitrary force translation, this exact symmetry cannot automatically be extended to a bounded-input particle problem.

For Astrobee, the left pose action cancels in $q^{-1}q_d$; body twists and body wrenches remain unchanged. For the quadrotor, full spatial rotation would rotate the world gravity direction, so only yaw and translation remain symmetries. The retained $R^Te_3$ is the physical information lost by an unjustified full-$SE(3)$ reduction. Two states with the same relative pose and twist errors can require different actions if their attitudes relative to gravity differ.

### 7.7 Training protocol, reward interpretation, and figure reading

The reward is negative tracking and action-error cost; the particle position term contains a constant offset through $\tanh(a_r\|r-r_d\|)-1$. Thus its plotted reward is not an error in meters, and reward magnitudes across different systems are not directly comparable. Astrobee also penalizes the norm of the relative-rotation logarithm and twist mismatch. The quadrotor uses the analogous pose/twist tracking costs with rotor-force actions. A rotation logarithm requires a convention near angle $\pi$ even when the quotient itself is geometrically well-defined.

Figure 1 combines mean training returns over ten seeds with evaluation of the best-performing policy on twenty preplanned reference trajectories. Table I reports mean and standard deviation across ten training seeds of each trained policy’s RMS error on twenty trajectories. The spreads are standard deviations, not confidence intervals; this was verified in the original table caption. The particle ablation is especially informative: retaining absolute position causes a much larger loss than removing only the remaining symmetries. This shows which redundancy dominates in that benchmark, not a universal ranking of symmetry factors.

The paper evaluates preplanned references after training on stochastic reference inputs. The flat-output Lissajous construction makes quadrotor references dynamically consistent with the chosen model. It does not force training and evaluation trajectories to have identical distributions. The reported results support sample-efficiency and tracking improvements for the tested implementations. They do not isolate every possible advantage of alternative reward shaping, baseline tuning, network architecture, or training budget; the authors explicitly acknowledge this baseline limitation.

The appendix's free/proper-action lemma supports quotient regularity, not an extra robot controller. Freeness removes stabilizer ambiguity for the moving frame. Properness supplies well-behaved orbit separation. Measurable local choices suffice for the policy construction, while a globally continuous moving frame would require stronger bundle structure. These distinctions matter if the reduced observation or reconstructed action is used inside a smooth optimization or differentiable-control pipeline.

## 8. Adaptive aerial-manipulator control

**Source:** P06. This earlier approach uses Euler angles, hierarchical inverse kinematics, approximate inverse dynamics, and online uncertainty approximation.

### 8.1 Kinematics and hierarchy

With end-effector offset $p_{eb}^b(q)$,

$$p_e=p_b+R_bp_{eb}^b,\quad R_e=R_bR_e^b,$$
$$v_e=J_\zeta\dot\zeta+J_\sigma\dot\sigma_b,$$

where $\zeta=(p_b,\psi,q)$ contains base position, yaw, and joints, while $\sigma_b=(\theta,\phi)$ contains pitch and roll. The latter are determined through the translational control loop rather than independent task commands. The task dimension requires $n+4\ge6$.

The acceleration-level inverse-kinematics command is

$$\ddot\zeta_r=J_\zeta^\dagger\left(\dot v_{e,d}+K_v(v_{e,d}-v_e)+K_pe-\dot J_\zeta\dot\zeta-J_\sigma\ddot\sigma_b-\dot J_\sigma\dot\sigma_b\right),$$

with $J_\zeta^\dagger=J_\zeta^T(J_\zeta J_\zeta^T)^{-1}$ on full-row-rank regions. Position error is desired minus actual; orientation error uses the vector part of the relative quaternion rotated by desired orientation. Redundancy requires secondary-task selection or nullspace damping; it cannot simply be ignored.

### 8.2 Approximate inverse dynamics and learned correction

The model is

$$M(\xi)\ddot\xi+C(\xi,\dot\xi)\dot\xi+g(\xi)+d(\xi,\dot\xi)=u.$$

An ideal law is $u=M\alpha+C\dot\xi+g+\hat d$, with PD-plus-reference-acceleration choices for $\alpha$. However, computing translational force would require roll/pitch control terms before their references exist. The implementation zeros two columns of the translational-attitude coupling block and treats the induced error as an uncertainty. It also removes roll/pitch reference angular accelerations to avoid repeated noisy differentiation.

From desired world force $u_f$,

$$f_z=\|u_f\|,\quad \theta_r=\arctan\frac{u_{f,x}\cos\psi+u_{f,y}\sin\psi}{u_{f,z}},$$
$$\phi_r=\arcsin\frac{u_{f,x}\sin\psi-u_{f,y}\cos\psi}{\|u_f\|}.$$

These formulas require nonzero thrust and the chosen upright angle domain. The full dynamics are not globally nonsingular because the Euler representation and force inversion have restricted domains.

Collect approximation errors, omitted feedforward, and physical uncertainty into $\delta$:

$$\ddot\xi=\alpha-\delta+\hat\delta,\qquad \delta=\Lambda(\xi)\chi+\varsigma.$$

The PDF prints radial functions $\lambda_{ih}=\exp(-\|\xi-c_{ih}\|/(2\sigma^2))$, with an **unsquared** norm; this differs from the conventional Gaussian RBF and should not silently be replaced when reproducing the source. The adaptation law is

$$\dot{\hat\chi}=\frac1\beta\Lambda^TB^TQz,\quad z=(\widetilde\xi,\dot{\widetilde\xi}),\quad B=\begin{bmatrix}0\\I\end{bmatrix}.$$

For intended stable PD error dynamics $\dot z=Az+B\Lambda\widetilde\chi$, the candidate

$$V_i=\tfrac12z^TQz+\tfrac\beta2\widetilde\chi^T\widetilde\chi$$

cancels the cross term under the adaptation update. With $A^TQ+QA=-P$, $\dot V_i=-\tfrac12z^TPz$. Exact approximation and the usual boundedness conditions yield tracking convergence; parameter convergence additionally requires persistent excitation. Bounded approximation residuals generally yield bounded rather than zero errors, and projection is suggested to keep parameter estimates bounded without excitation.

### 8.3 Source-equation issues already identified

The printed inner-loop gain blocks have negative gains even though Eq. (38) also negates them; the printed state matrix swaps the expected position/velocity gain ordering. The stated Hurwitz argument must be interpreted through the intended stable PD system, not implemented literally. The Lyapunov derivative also has a factor-of-two inconsistency with its own definition. These were visually checked on PDF page 4. The manipulator torque expression uses a coupling block whose dimensions are inconsistent for general $n$; the joint-position coupling should be rederived from the corresponding row of $M$. The yaw term in Eq. (19) prints desired yaw where its derivative is expected; the full quaternion sign conventions remain unresolved.

The outer-loop sufficient gain inequality is

$$\lambda_{\min}(K_{v,O})>\frac{\lambda_{\max}(K_{p,O})^2}{2\lambda_{\min}(K_{p,O})}.$$

The printed simulation gains $K_p=7.5I_6$, $K_v=0.6I_6$ do **not** satisfy that sufficient inequality ($0.6>3.75$ is false). Successful simulation therefore does not establish that particular certificate for those gains. For imperfect inner-loop tracking, the outer loop sees $J_\zeta\ddot{\widetilde\zeta}$; the paper distinguishes vanishing and bounded nonvanishing perturbations.

### 8.4 Simulation record

MATLAB/SimMechanics, five revolute joints, nominal inertia at 90% of true inertia, 250 Hz measurements, first-order derivative filters with 0.03 s time constant. Noise mean / standard deviation: position $0.001/0.005$ m; orientation $0.001/0.001$ rad; attitude rates $0.005/0.005$ rad/s; joints $0.0001/0.0005$ rad. Commands include a 3D path, five-second terminal hover, and $\pi/5$ rotations.

Gains: vehicle position $K_P=12I_3,K_V=5I_3$; joints $140I_5,20I_5$; yaw $8,3$; pitch and roll each $2,1$; inverse kinematics $7.5I_6,0.6I_6$; adaptation $\beta=4$. Adaptation mainly improves base position and attitude; lightweight arm joint tracking changes little. Complete physical parameters are referred to an earlier paper, not tabulated here.

### 8.5 Additional approximation and proof checks

The source prints a radial-basis expression with an **unsquared** distance in the exponential. Calling it a conventional squared-distance Gaussian without qualification would change the approximation model. Other flagged expressions include the desired-yaw derivative in Eq. (19), the arm coupling block in Eq. (32), and the order/signs of the inner-loop gain blocks. The arm equation's use of a translation–rotation coupling block instead of a translation–joint block is dimensionally problematic for the five-joint arm.

The outer-loop Lyapunov derivative and cross-term bounds need to be derived with the exact quaternion-error convention. The displayed derivative does not follow automatically from treating quaternion error as an ordinary Euclidean coordinate. The numerical gains 7.5 and 0.6 should not be represented as satisfying the stated sufficient inequality without checking its constants. Euler-coordinate limits and quaternion sign ambiguity also matter when interpreting a global claim. Bounded nonvanishing inner-loop approximation error does not by itself establish asymptotic zero outer-loop error. Sections 8.6–8.7 derive the consistent translational and adaptive error systems. The printed quaternion argument remains an unresolved source limitation; this reference does not claim a replacement global attitude proof.

### 8.6 From kinematic inversion to the two error systems

Differentiating $v_e=J_\zeta\dot\zeta+J_\sigma\dot\sigma_b$ yields four terms: $J_\zeta\ddot\zeta$, $\dot J_\zeta\dot\zeta$, $J_\sigma\ddot\sigma_b$, and $\dot J_\sigma\dot\sigma_b$. The inverse-kinematics law cancels the last three and prescribes end-effector acceleration. Full row rank is essential: $J_\zeta J_\zeta^\dagger=I_6$ only on the regular region. Near a singularity, the pseudoinverse can produce large accelerations even for a modest task error. Damped inversion would be a practical modification, not the exact cancellation used in the source proof.

Under perfect acceleration tracking,

$$\dot v_{e,d}-\dot v_e=-K_v(v_{e,d}-v_e)-K_pe.$$

For the translational component, $e_P=p_{e,d}-p_e$ and therefore

$$\ddot e_P+K_{v,P}\dot e_P+K_{p,P}e_P=0.$$

Its standard state matrix is $A_P=[0\ I;-K_{p,P}\ -K_{v,P}]$. Positive-definite proportional and derivative matrices give the familiar damped second-order system; a Lyapunov matrix solves $A_P^TQ_P+Q_PA_P=-P_P$. The orientation component is not the same Euclidean second-order equation because quaternion vector error has its own differential kinematics. The source's candidate combines quaternion-coordinate error, angular-rate error, and translational quadratic energy. Its cross-term bound leads to the sufficient inequality recorded above. The printed quaternion convention and derivative do not justify silently treating that candidate as an independently verified global proof.

With $\widetilde\zeta=\zeta_r-\zeta$, imperfect acceleration tracking gives

$$\dot v_{e,d}-\dot v_e=-K_v(v_{e,d}-v_e)-K_pe+J_\zeta\ddot{\widetilde\zeta}.$$

The paper bounds the perturbation using $\|J_\zeta\|\|\ddot{\widetilde\zeta}\|$. A vanishing, sufficiently controlled perturbation allows a cascade argument; a persistent residual generally leaves bounded task errors. This also requires the robot to remain in a domain where Jacobians and their derivatives remain bounded and the chosen orientation coordinates are valid.

### 8.7 Adaptive cancellation, residuals, and implementation order

Using positive intended gains, let

$$A=\begin{bmatrix}0&I\\-K_P&-K_V\end{bmatrix},\qquad
\dot z=Az+B\Lambda\widetilde\chi+B\varsigma,\quad
\widetilde\chi=\chi-\hat\chi.$$

For constant ideal parameters, $\dot{\widetilde\chi}=-\dot{\hat\chi}$. Substitution into the candidate derivative gives

$$\dot V_i=-\frac12z^TPz+z^TQB\Lambda\widetilde\chi
-\beta\widetilde\chi^T\dot{\hat\chi}+z^TQB\varsigma.$$

The update law cancels the middle two terms exactly. If $\varsigma=0$, the remaining negative quadratic term supports the source's tracking argument with boundedness and uniform-continuity hypotheses. If $\|\varsigma\|\le\bar\varsigma$, the explanatory estimate

$$\dot V_i\le-\frac12\lambda_{\min}(P)\|z\|^2
+\|QB\|\bar\varsigma\|z\|$$

shows why negative drift is only assured outside a residual-dependent radius. It does not alone bound parameter drift without an appropriate projection or excitation condition. If $\chi$ varies, another term $\beta\widetilde\chi^T\dot\chi$ appears; “slowly varying” is an approximation unless that term is bounded explicitly.

The implementation dependency order is consequential. First, the task loop generates base-position, yaw, and joint references. The translation loop then computes a desired force using the approximate coupling matrix. Force direction and yaw determine roll/pitch references. Filtered derivatives produce desired angular rates, and the simplified attitude controller omits noisy second derivatives. Attitude generalized forces are converted to physical body moments through the Euler-rate mapping. Rotor forces are found from the vehicle input map, and arm torques from the joint row of the coupled equations. The adaptive term compensates both parameter mismatch and deliberate omissions, so it is not exclusively learning a physical external disturbance.

Figure 1 defines frames and the manipulator geometry; Figure 2 shows this hierarchy. Figure 3 shows the desired three-dimensional end-effector path; Figure 4 shows end-effector pose-error norms; Figure 5 compares vehicle/joint motion-control errors with and without adaptation. The simulation includes 10% inertia-model mismatch and noisy filtered measurements. It does not provide hardware timing or demonstrate persistent excitation of every RBF coefficient. Since some physical parameters are delegated to another paper, the supplied PDF alone is insufficient to reproduce the exact simulated plant.

## 9. Predictive admittance control

**Source:** P01.

The novelty is putting virtual impedance dynamics **inside** the NMPC prediction, rather than holding the current output of an external admittance filter across the prediction horizon.

With tool position $p$, body attitude quaternion $q$, angular velocity $\omega$, rotor thrusts $\gamma$, and measured contact force $f_c$,

$$x=(p,\dot p,q,\omega,\gamma,f_c),\qquad \dot\gamma=u,\qquad\dot f_c=0.$$

The end-effector model is

$$\ddot p=-ge_3+m^{-1}RG_f\gamma+m^{-1}f_c+R(\widehat{\dot\omega}r+\widehat\omega^2r),$$
$$\dot\omega=J^{-1}(-\omega\times J\omega+G_\tau\gamma+r\times R^Tf_c),\qquad\dot q=\tfrac12q\otimes(0,\omega).$$

Force is constant *within the prediction horizon*, not physically asserted constant for the entire experiment. No environment stiffness/contact model is needed, but rapid contact-force changes violate the prediction approximation.

Virtual displacement obeys

$$\dot\chi=\begin{bmatrix}\dot\delta\\M_d^{-1}(-D_d\dot\delta-K_d\delta+f_c)\end{bmatrix},\qquad\chi=(\delta,\dot\delta).$$

The OCP minimizes a weighted squared residual over robot and impedance predictions. Its residual is

$$\Delta=\operatorname{col}(p-p_r-\delta,\dot p-\dot p_r-\dot\delta,\ddot p-\ddot p_r-\ddot\delta,q\ominus q_r,\omega-\omega_r,\dot\omega-\dot\omega_r).$$

Constraints bound rotor thrusts and thrust rates. The robot initializes from the state estimate; the impedance initializes from the previous prediction's first advanced state. The cost has no explicit input penalty. Source notation calls the weight matrix positive definite, but the actual acceleration weights are zero, hence the experimental matrix is positive semidefinite. Zero force restores ordinary trajectory tracking only after impedance displacement/velocity are zero; releasing contact does not instantaneously erase stored virtual motion.

Implementation: 2.8 kg tilted hexarotor; $J=\operatorname{diag}(0.115,0.114,0.194)$ kg m²; rotor tilt alternating ±20°; rotor-to-rotor distance 0.7 m; thrust coefficient $11.75\times10^{-4}$ N/Hz²; drag-torque coefficient $2.388\times10^{-5}$ Nm/Hz². A rigid tool carries an IIT FT45 sensor; force is filtered by a second-order 10 Hz Butterworth filter and 0.2 N deadband. State UKF/controller run at 250 Hz, IMU at 500 Hz, MoCap at 150 Hz. MATLAB/Simulink runs on an external i7-12800HX PC with a serial cable. MATMPC, qpOASES, real-time iteration, multiple shooting, RK4, and CasADi are used. Horizon: 1.5 s, 15 intervals, 0.1 s prediction interval. This interval differs from the 4 ms feedback period; the exact impedance-state shift at these two rates deserves implementation verification.

Both controllers use weights $(50I,50I,0,5\times10^5I,I,0)$ for position, velocity, acceleration, attitude, angular velocity, angular acceleration. Free-flight simulation applies a −4 N pulse between 5 and 20 s. Apparent masses are 0.1, 1, and 10 times physical mass; damping ratios 0.5, 1, 1.5; natural frequency 2 rad/s. Parameters satisfy $k=\bar m\omega_n^2$, $d=2\zeta\sqrt{k\bar m}$. The critically damped physical-mass example uses $M_d=2.8I,D_d=11.2I,K_d=11.2I$.

Reported improvement reaches 90% smaller position error in one underdamped case; it is not a uniform improvement over all cases. High apparent mass gives similar performance and occasionally smaller transient baseline errors. Low apparent mass explains why the proposed controller may thrust in the direction of the applied force. Hardware wall pushing remains smooth with the predictive method; the tested baseline loses contact, oscillates, and becomes unstable after increased penetration. Reducing baseline attitude weight from $5\times10^5$ to $5\times10^2$ avoids the crash but retains oscillation. Average solve times are 2.19 ms versus 2.13 ms. These are empirical results, not a general contact-stability or recursive-feasibility proof.

### 9.1 Prediction dimensions, constraints, and actuator interpretation

The visually checked finite-horizon formulation optimizes robot states and actuator-rate inputs, with the admittance state propagated alongside them. It minimizes a sum of weighted squared residuals $\|\Delta(x_k,\chi_k,r_k)\|_{Q_k}^2$, constrains the initial robot state to the estimate, initializes the admittance state from the preceding prediction, and imposes actuator and actuator-rate limits. Interval indices should be read as all prediction intervals $0,\ldots,N-1$, not only the two endpoints suggested by a literal set reading of the printed notation.

The robot-state storage dimension is $n+16$: position 3, velocity 3, quaternion 4, angular velocity 3, actuator thrusts $n$, and contact force 3. For six rotors this is 22 stored components. Adding six admittance components gives 28, with the quaternion's unit constraint reducing the intrinsic dimension. The reference has 19 stored components but the residual has 18 because attitude error is three-dimensional. The precise quaternion residual is delegated to cited work rather than fully printed.

The stated cost does not contain a separate actuator-effort penalty. Rate limits restrict command changes but do not alone imply every possible smoothness property. If thrust satisfies $\gamma=cn^2$, then $\dot\gamma=2cn\dot n$; commanded thrust rate is therefore not simply motor torque or rotor acceleration.

The contact model carries a three-dimensional force and its moment $r\times R^Tf_c$; it does not independently model arbitrary contact torque, despite using a six-axis sensor. Zero commanded force and return to a desired impedance equilibrium are separate statements. The relationship between the 250 Hz feedback loop, the 0.1 s prediction grid, and the shifted admittance-state initialization needs further implementation detail; the printed sampling choices alone do not establish an implementation error.

Additional hardware notes retained from the source: 13-inch propellers, MK3638 motors, BL-Ctrl 2.0 controllers, a 3200 mAh 4S battery, and an external Ubuntu 20.04 computer with 32 GB RAM.

### 9.2 Expanded optimal-control problem and numerical workflow

Let $\phi_h$ denote the discretized robot dynamics and $\Xi_h$ the discretized admittance dynamics. A consistent restatement of the source OCP is

$$\begin{aligned}
\min_{x_{0:N},\chi_{0:N},u_{0:N-1}}\quad&\sum_{k=0}^{N}\Delta(x_k,\chi_k,r_k)^TQ_k\Delta(x_k,\chi_k,r_k)\\
\text{subject to}\quad&x_{k+1}=\phi_h(x_k,u_k),\quad \chi_{k+1}=\Xi_h(\chi_k,f_{c,k}),\\
&x_0=\hat x(t),\quad\chi_0=\chi_{t-1,1},\\
&\gamma_{\min}\le\gamma_k\le\gamma_{\max},\\
&\dot\gamma_{\min}(\gamma_k)\le u_k\le\dot\gamma_{\max}(\gamma_k).
\end{aligned}$$

The dynamic constraints apply to every interval, state bounds to their specified nodes, and actuator-rate bounds to every input interval. Eliminating $\chi$ by deterministic forward propagation gives an equivalent smaller decision-variable representation when its initial condition and force prediction are fixed. The essential feature is its dependence on predicted force/state, not whether it appears as an explicit solver variable.

At each feedback step the estimator supplies pose, twist, and rotor information; the filtered measured force updates the contact-force state. The previous solution provides a warm start. Multiple shooting introduces node states and continuity constraints; RK4 propagates each interval. Real-time iteration solves a local sequential quadratic-program approximation rather than fully converging a nonlinear program at every step. The first rate command is applied and the problem is rebuilt with fresh measurements. This explains the distinction between feedback frequency, prediction-grid spacing, and average solver time.

The end-effector acceleration contains angular-acceleration and centripetal terms because the tool is offset from the body origin. The same contact force contributes both translation and the lever-arm torque. Consequently, penalizing position, attitude, and their derivatives does not produce six independent scalar optimizations. Rotor constraints couple the achievable force and torque even for a full-rank wrench map.

The paper does not include a terminal invariant set or a printed recursive-feasibility proof. A terminal tracking penalty, actuator bounds, and fast solve times are not substitutes for such a result. Likewise, the constant-force prediction is an estimator extrapolation, not a model of unilateral contact, penetration, friction, or wall stiffness.

### 9.3 Why predicting admittance changes the response

For diagonal virtual parameters each axis has transfer function

$$\frac{\delta(s)}{f_c(s)}=\frac{1}{m_ds^2+d_ds+k_d}.$$

The static displacement depends on stiffness, while mass and damping determine transient response. At the onset of a force step from rest, $\ddot\delta(0^+)=f_c/m_d$. If $m_d$ is much smaller than physical mass, following this acceleration can require actuator force in the same direction as the external force. This accounts for the low-apparent-mass thrust plots without suggesting that the controller has reversed the desired compliance sign.

In the comparison architecture, an external admittance filter computes a modified reference at the current instant and the NMPC treats it as fixed over the horizon. In the proposed architecture, the same virtual mass, damping, and spring evolve across the horizon. The optimizer therefore anticipates the reference moving in response to a sustained measured force. This is particularly relevant at a wall, where the actual tool position is constrained and force-induced reference motion changes the predicted error.

The $3\times3$ parameter sweep combines three apparent masses with three damping ratios at fixed natural frequency. Figure 4 shows a representative free-flight step response; Figure 5 compares position/velocity error across the sweep; Figure 6 explains the low-mass actuator response. The source's largest reported improvement belongs to an underdamped case. It does not mean every mass/damping choice improves by that amount.

Figures 7–9 document the wall experiment and the mechanism proposed for the difference. The baseline initially increases force, the admittance reference then moves away, and contact is lost around 3.5 s. Repeated cycles produce oscillation. Increasing penetration near 12 s drives the baseline into saturation around 13 s with substantial pitch error. The proposed method maintains smoother reference/force evolution. Lowering baseline attitude weight by three orders of magnitude prevents the reported crash while retaining oscillation, demonstrating that tuning materially affects the comparison. Average solve times of 2.19 and 2.13 ms fit inside a 4 ms period on average; no worst-case deadline guarantee follows from those means.

## 10. Sensorless six-dimensional interaction

**Source:** P09, published February 13, 2026.

### 10.1 Vehicle and state-dependent allocation

Eight reversible rotors occupy a cube of side 0.25 m. Rotor tilt is 50°; alternating axes and sign sequence $[1,-1,-1,1,-1,1,1,-1]$ define the layout. Signed-square input $u_i=|\omega_i|\omega_i$ gives

$$f_i=c_fu_iz_i,\quad m_i=r_i\times f_i-k_ic_\tau u_iz_i,\quad w_{\rm rot}=Au,\quad A\in\mathbb R^{6\times8}.$$

Rotor interference makes one constant nominal map inaccurate. Different body-frame gravity directions require different rotor speed/reversal patterns. The method identifies local maps over ten regions of $S^2$ rather than modeling every aerodynamic interaction explicitly. Each region uses fifteen hover orientations; the known balancing wrench has force magnitude $mg$ and zero net torque.

For $a=\operatorname{vec}(A)$, each sample satisfies $w_j\approx U_ja$, where $U_j=[u_1I_6\ \cdots\ u_8I_6]$. A dimensionally consistent least-squares formulation is

$$\min_a\|\bar Ua-\bar w\|_W^2\quad\text{s.t.}\quad a_{\rm nom}-\Delta\le a\le a_{\rm nom}+\Delta,$$

with **vertical** stacking of $U_j$ and experimental deviation bound $\Delta=3\times10^{-5}$. This is an explanatory corrected formulation: the printed PDF horizontally stacks sample blocks despite incompatible dimensions and gives a linear-cost coefficient missing the factor of two for its printed quadratic convention. These issues were visually confirmed on page 5.

Hover identification does not fully excite arbitrary torque/wrench directions. Its generalization depends on interference scaling acceptably away from hover. A global fitted map and local fitted maps are compared. Free-flight validation uses five random orientations, five seconds each; a nominal force bias of approximately 2 N is largely eliminated in the shown region.

### 10.2 Observer, admittance, and allocation

The translational observer is

$$\dot{\hat f}_{\rm ext}=-L\hat f_{\rm ext}+L(m\ddot p+mge_3-RA_1u).$$

The rotational momentum observer integrates modeled input torque and $\omega\times J\omega$ to estimate external torque without directly differentiating angular velocity. A tool-frame wrench transform feeds

$$M_E\ddot e+D_E\dot e+K_Ee=\hat w_E^{\rm ext}.$$

An inner pose controller tracks the compliant motion. The source requires the motion loop to have higher bandwidth than the admittance dynamics. The observer estimates unexplained external wrench; wind and modeling error can therefore create unintended compliance.

Allocation respects $-\omega_{i,\max}^2\le u_i\le\omega_{i,\max}^2$ and penalizes wrench residual and effort. The PDF calls this a QP but prints an **unsquared** residual norm, which would not be a quadratic objective. A true least-squares QP needs a squared norm. The printed pose-controller torque compensation also adds estimated external torque despite the dynamics adding that torque; its sign needs reconciliation. The integral-error definition is self-referential as printed. These are issues in the printed equations, not claims about the deployed software. ProxSuite allocation is reported at 1 kHz with warm starts.

### 10.3 Experimental parameters and findings

Mass 1.625 kg; isotropic inertia $0.0113I_3$ kg m²; tool offset $(0,0.5,0)$ m. Observer gains $L=15I_3,K_I=25I_3$ plus 10 Hz filtering. Pose gains: $K_p=\operatorname{diag}(23,23,25)$, $K_v=\operatorname{diag}(15,15,20)$, $K_i=\operatorname{diag}(4,4,6)$, $K_R=300I$, $K_\omega=80I$. UKF fuses IMU and MoCap; software uses TeleKyb3.

Admittance scalars, each multiplying $I_3$:

| Task | Translational mass | Rotational mass | Translational damping | Rotational damping | Translational stiffness | Rotational stiffness |
|---|---:|---:|---:|---:|---:|---:|
| Point contact / sliding | 1.5 | 1.5 | 13.5 | 13.5 | 30 | 30 |
| Peg insertion | 1.5 | 0.5 | 15 | 2.14 | 35 | 2.29 |

Point contact uses nominal allocation, straight and 30°-tilted surfaces, ±20° tool reorientation, and a surface-mounted ATI Mini40 for ground truth. Tangential drift is approximately 1–2 cm on the straight surface and 2–3 cm on the tilted one; tilted-contact force is underestimated by about 1 N.

Sliding follows a pipe of radius 0.55 m up to 45° roll, with desired waypoints 3 cm inside the surface. Nominal allocation loses contact around 32 s after an erroneous roughly 1 N vertical force estimate moves the compliant reference away. Both fitted-map approaches maintain contact. Local maps switch near 30 s. Positional RMS $(x,y,z)$ in cm: nominal $(1.96,0.71,0.76)$; single fitted map $(3.00,0.50,0.32)$; local maps $(0.88,0.68,1.29)$. Improvement is axis-dependent, not uniformly better on every metric.

Peg insertion uses a 10 cm funnel, 8 cm entrance, 19 mm terminal hole, and 16 mm peg (1.5 mm radial clearance). Starting roll misalignment is −10°; compliance brings it to about −5° and enables insertion. This trial also uses nominal allocation. The paper leaves fuller SO(3) coverage, smoother map switching, torque-model identification, sensitivity analysis, and more quantitative force validation for future work. The complete RMS/max table is included in Section 10.4.

### 10.4 Identification conventions, region coverage, and complete pose metrics

The visually checked identification expressions need consistent least-squares normalization. If

$$L(a)=(\bar Ua-\bar w)^TW(\bar Ua-\bar w),$$

then, up to a constant,

$$L(a)=a^T\bar U^TW\bar Ua-2a^T\bar U^TW\bar w.$$

The printed combination of $a^THa+f^Ta$, $H=\bar U^TW\bar U$, and $f=-\bar U^TW\bar w$ omits that factor of two. The alternative convention $\tfrac12a^THa+f^Ta$ gives the intended minimizer with those definitions. Stacking multiple measurement equations requires a vertical stack; the displayed horizontal stack does not have the required regression dimensions. Parameter corrections are bounded by $\pm3\times10^{-5}$.

The allocator's printed objective has an unsquared residual norm plus $\lambda\|u\|_2^2$. That is not a quadratic objective as written; squaring the residual gives the described quadratic-program form. This distinction changes the optimization problem and should not be silently erased.

Other controller notation issues remain relevant: the moment-compensation sign differs from what the displayed additive disturbance model suggests; the attitude error uses world-frame rotation products while the moment equation is expressed in body coordinates; and the integral-error definition is self-referential rather than explicitly integrating position error. These require consistent conventions before reconstruction of an implementation.

The ten-region map uses upper and lower caps with elevation intervals $[\pi/3,\pi/2]$ and $[-\pi/2,-\pi/3]$, respectively. The remaining bands have elevations $[0,\pi/3]$ and $[-\pi/3,0]$. The azimuth sectors printed in the figure caption are $[-\pi/4,\pi/4]$, $[-3\pi/4,-\pi/4]$, $[\pi/4,3\pi/4]$, and $[-\pi,-3\pi/4]$. As written, these omit $[3\pi/4,\pi]$. A wrapped final sector is a plausible intended completion, but its union and boundary handling are not specified by that caption. The identification uses fifteen measurements per region; the illustrated R3 validation uses five trials of five seconds.

The complete Table 1 transcription below records RMS and maximum absolute pose errors. Position units are **cm** and attitude units are **degrees**. PC = point contact; SD = sliding; PiH = peg insertion. Each six-value group is ordered $x,y,z,\mathrm{roll},\mathrm{pitch},\mathrm{yaw}$.

| Trial | RMS x | RMS y | RMS z | RMS roll | RMS pitch | RMS yaw | Max x | Max y | Max z | Max roll | Max pitch | Max yaw |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| PC, straight | 2.09 | 1.87 | 0.56 | 0.40 | 0.16 | 0.50 | 5.06 | 4.01 | 2.50 | 1.70 | 0.63 | 1.55 |
| PC, tilted | 1.66 | 1.44 | 1.13 | 0.47 | 0.24 | 0.41 | 3.75 | 3.03 | 2.77 | 2.03 | 0.95 | 1.38 |
| SD, nominal | 1.96 | 0.71 | 0.76 | 0.51 | 0.20 | 0.20 | 3.48 | 2.03 | 3.64 | 3.85 | 1.32 | 1.51 |
| SD, single optimized map | 3.00 | 0.50 | 0.32 | 0.29 | 0.16 | 0.16 | 4.90 | 1.17 | 1.32 | 1.63 | 0.59 | 0.70 |
| SD, multiple optimized maps | 0.88 | 0.68 | 1.29 | 0.25 | 0.13 | 0.15 | 1.65 | 1.53 | 3.09 | 1.45 | 0.54 | 0.72 |
| PiH | 2.02 | 1.36 | 1.14 | 0.29 | 0.13 | 0.17 | 2.45 | 2.94 | 1.52 | 0.67 | 0.38 | 0.44 |

The rotational momentum observer integrates $\omega\times J\omega-A_2u-\hat m$ and combines it with $J\omega(t)-J\omega(t_0)$. Differentiating the observer with an exact model yields first-order disturbance-estimation dynamics. An allocation-map change also changes the model residual seen by the observer; therefore region switching and observer transients are coupled. Continuity across the map is an unresolved improvement, not an established property.

The reported point-contact force reaches roughly one quarter of vehicle weight: a 15.94 N weight corresponds to approximately 4 N. A 0.5 m tool amplifies the associated moment. The illustrative static spring calculation $30\,\mathrm{N/m}\times0.03\,\mathrm m=0.9\,\mathrm N$ is an explanatory equilibrium calculation, not a separately measured exact force. A 1 kHz control/allocation rate does not imply that every sensor operates at that rate. The paper offers data on request; external data and software have not been inspected here.

### 10.5 Allocation identification as a constrained estimation problem

Each sample contributes six wrench equations for the 48 entries of an allocation matrix. Fifteen samples therefore produce a $90\times48$ regression matrix under vertical stacking. More equations than unknowns do not guarantee identifiability: the rotor-input samples must excite independent directions, and hover imposes relationships between them. The nominal-map box constraint regularizes the estimate toward a plausible geometry-derived map, but cannot create missing physical information.

The orientation-dependent map indexes aerodynamic operating regimes by body-frame gravity direction. Two orientations differing only by world yaw have the same gravity direction, while different rotor patterns and body tilts can change interference. This is a useful reduced scheduling variable, not proof that every aerodynamic coefficient depends only on that two-dimensional quantity. Speed, acceleration, contact wrench, and proximity to surfaces can introduce additional dependence.

For a chosen matrix $A$, bounded signed-square inputs form a box $\mathcal U$. Its attainable wrench set is $A\mathcal U$. Even if $\operatorname{rank}A=6$, this set is bounded. A least-squares allocator trades wrench mismatch against effort when the requested wrench is infeasible. A positive effort regularizer can make a correctly squared objective strictly convex, but the printed objective ambiguity must first be resolved. Reversible rotors allow positive and negative signed-square inputs; reversal transients are not captured merely by these static bounds.

### 10.6 Observer dynamics and the interpretation of compliance

Let the true translational residual be $r_f=m\ddot p+mge_3-RA_1u$. With a perfect allocation model $r_f=f_{\rm ext}$. The observer obeys $\dot{\hat f}=L(f_{\rm ext}-\hat f)$, so for estimation error $e_f=\hat f-f_{\rm ext}$,

$$\dot e_f=-Le_f-\dot f_{\rm ext}.$$

It converges exponentially for a constant force and constant positive-definite $L$. Fast-changing force creates lag, and acceleration/model noise enters through the same channel. For a scalar gain $l$, the ideal force-to-estimate transfer function is $l/(s+l)$; the additional 10 Hz filtering adds further lag. The gains 15 and 25 are observer parameters, not direct guarantees of the entire feedback-loop bandwidth.

The rotational observer can be expressed with momentum $p_\omega=J\omega$ as

$$\hat m=K_I\left[p_\omega(t)-p_\omega(t_0)+\int_{t_0}^{t}\left(\omega\times J\omega-A_2u-\hat m\right)d\tau\right].$$

Using $\dot p_\omega=A_2u+m_{\rm ext}-\omega\times J\omega$ gives $\dot{\hat m}=K_I(m_{\rm ext}-\hat m)$. The integral form avoids explicit differentiation of measured angular velocity but still relies on the applied-torque model and correct initial conditions.

If the allocation error is $\Delta A$, then the translational residual contains $R\Delta A_1u$ and the rotational residual contains $\Delta A_2u$. These terms are indistinguishable from physical external wrench to this observer. Better identification therefore improves contact behavior indirectly by preventing false compliance, as the sliding trial illustrates. It does not turn the observer into a contact-location detector or a separator of wind from tool force.

The wrench is transformed to the tool frame before applying the six-axis virtual dynamics. Torque must be shifted to the tool origin as well as rotated; force rotation alone is insufficient for a nonzero 0.5 m offset. Translational virtual mass and rotational virtual inertia have different physical units even when equal numerical values appear in Table 2. The rotational error coordinates also need a defined local orientation convention; six-dimensional compliance is not globally equivalent to adding six Euclidean coordinates to an arbitrary pose.

### 10.7 What each hardware trial tests

The straight and tilted point-contact trials examine simultaneous contact and reorientation using an external force sensor for validation. They use nominal allocation, so their success is evidence for the overall compliant-control architecture, not an isolated validation of the fitted maps. The sliding trials compare nominal, single fitted, and region-dependent fitted allocation, making them the direct test of interference compensation. Contact preservation is a separate outcome from every component of positional RMS; the single fitted map can preserve contact while having a larger error on one axis.

The peg trial tests angular compliance under a geometric insertion constraint. The funnel assists capture, and the 19 mm hole with a 16 mm peg leaves 3 mm diametral clearance. Success in that fixture is not a statistical insertion-success rate across randomized offsets, and the article does not supply such a rate. Figures 5–6 show point contact; Figures 7–9 show sliding and contact loss; Figures 10–11 show approach, alignment correction, and insertion. Figure 3's validation violins describe a particular region's wrench residuals; they do not establish uniform error bounds over all ten regions.

## 11. Passive alignment during pushing

**Source:** P07.

A three-foot end effector must establish all three contacts with a flat surface. A stiff attitude controller can oppose the small rotation necessary to achieve full contact. The method deliberately releases two rotational axes while pushing along the tool symmetry axis. Three feet are spaced 120° apart at radius $d_r$; pressure sensors indicate individual contact.

Body dynamics and hybrid control are

$$M\dot\nu+C\nu+g=w_a+w_e,$$
$$w_a=\Lambda w_{\rm mot}+w_f+C\nu+g,\quad \Lambda=\operatorname{diag}(1,1,0,0,0,1).$$

Thus tool-axis translation is force-controlled; roll/pitch-like rotations are passive; lateral translations and axial rotation remain motion-controlled. Motion command is $w_{\rm mot}=M\dot\nu_{\rm ref}-D_ve_v-K_pe_p$. Tool-axis force uses PI feedback:

$$f_B=f_{\rm ref}-k_pe_f-k_i\int e_fdt,\qquad e_f=f_{\rm est}-f_{\rm ref}.$$

The force estimate is momentum-based. The FT sensor used in experiments provides validation rather than force feedback. Switching from free-flight to interaction is manual, beginning at zero force reference.

Let $\beta$ measure the projected angular misalignment and $d_{CC}$ the central tool-face gap. Geometry gives $d_{CC}=\|l_O\|\sin|\beta|$. For one contact $\|l_O\|=d_r$; for two contacts it lies between $d_r/2$ and $d_r$, simplified to $d_r/2$ when the center of pressure is at the edge midpoint. The three-dimensional out-of-plane force angle $\theta$ satisfies the geometric bound $\sin\theta_{\max}<\sqrt3\sin|\beta|$ in the analyzed regime.

Two conditions follow:

$$\mu_{\rm lim}(\beta)=\tan|\beta|\sqrt{1+\frac3{1-3\sin^2|\beta|}},\qquad\mu_{\rm lim}(\beta_{\max})\le\eta\mu_S,$$
$$|f_B|\sqrt{1-3\sin^2|\beta_{\max}|}\frac{d_r}{2}>|\tau_d|_{\max}.$$

The first ensures sufficient friction with safety factor $\eta\in(0,1]$. The second ensures the contact-pivot restoring moment exceeds uncertain opposing torque. Their derivation assumes rigid flat contact, no slipping, gravity compensation, the stated geometry, and a bounded perturbation; the initial impact transient is neglected. The square-root expression requires a suitable angle domain. The paper analyzes small/moderate misalignment, plotting up to 30°, not arbitrary orientations.

The pendulum approximation is $m\|l_C\|_P^2\ddot\beta=\tau_e^p-\tau_d$. Increasing foot radius or pushing force increases the restoring moment; friction material and maximum expected angular error jointly determine whether pivoting can occur without slipping.

Experiments use $d_r=0.0525$ m, $\eta=0.4$, Vicon, and a ceiling-mounted wooden surface. Misalignment ranges are 0–6° and 7–11°, giving rounded friction bounds 0.2 and 0.4. Required static friction is approximately 0.5 and 1.0; tested coefficients are approximately 0.1, 0.6, and 1.2. Groups violating the condition slip and become unstable; groups satisfying it complete interaction. With disturbance torque bounded by 0.5 Nm and 6° misalignment, the rotation condition requires more than 19.4 N. Tests at 5, 10, and 20 N give the best three-foot contact at 20 N. A normalized pressure-sharing RMSE metric and tool-face gap quantify alignment. This is task-assisted passive dynamics, not absence of active control on the complete vehicle.

### 11.1 Derivation of the friction condition

Resolve the commanded push into its projection $f_P$ in the plane spanned by the surface normal and pivot lever arm, and the out-of-plane component $f_S$. Then

$$|f_n|=|f_B|\cos\theta\cos\beta,$$
$$|f_t|^2=|f_B|^2\left(\cos^2\theta\sin^2\beta+\sin^2\theta\right).$$

Dividing gives the explanatory intermediate identity

$$\mu^2=\left(\frac{|f_t|}{|f_n|}\right)^2
=\tan^2\beta+\frac{\tan^2\theta}{\cos^2\beta}.$$

For one foot in contact, the remaining feet cannot rotate through the work surface. Their triangular geometry bounds the out-of-plane angle by $\sin\theta_{\max}<\sqrt3\sin|\beta|$. For two feet, the simplifying midpoint center-of-pressure assumption gives $\theta=0$ and lever arm $d_r/2$. Substitution of the one-foot bound yields

$$\tan^2\theta<\frac{3\sin^2\beta}{1-3\sin^2\beta},$$

and hence the stated $\mu_{\rm lim}$. This requires $1-3\sin^2\beta>0$, or $|\beta|<35.264\ldots^\circ$. The bound becomes increasingly conservative and diverges at that endpoint. The source plots only up to 30°.

Here $\mu$ is the **required tangential-to-normal force ratio**, while $\mu_S$ is the available static coefficient. They are not the same variable. The safety factor imposes $\mu_{\rm lim}\le\eta\mu_S$, leaving a margin for unmodeled effects. Increasing force magnitude cancels from this ratio; it does not repair insufficient friction in this idealized geometry. Increasing force instead helps the separate restoring-torque condition.

### 11.2 Restoring rotation and its limits

For one or two contacts the projected push supplies a moment at least

$$|\tau_e|>|f_B|\sqrt{1-3\sin^2\beta_{\max}}\frac{d_r}{2}.$$

If this lower bound exceeds the largest opposing disturbance moment, the modeled acceleration points toward smaller absolute misalignment. The printed moment-magnitude equations should be combined with this restoring sign; an always-positive magnitude is not a signed angular equation valid on both sides of zero.

The effective pendulum inertia $m\|l_C\|_P^2$ approximates the robot as a lumped mass about the contact pivot. A full rigid-body inertia about that pivot includes the body rotational inertia and any changing contact geometry. The result therefore concerns the simplified contact model and explicitly neglects the short impact phase. It is a sufficient condition for the desired pivoting mechanism, not a proof of asymptotic stability of every contact-switching trajectory or of arbitrary curved-surface interaction.

For $d_r=0.0525$ m, $|\tau_d|_{\max}=0.5$ Nm, and $\beta_{\max}=6^\circ$, the threshold computed from the formula is about 19.37 N, explaining the source's 19.4 N and the choice of 20 N. Larger foot spacing increases the lever arm but also changes tool geometry, workspace, and possible collision constraints. The analysis does not optimize those tradeoffs.

### 11.3 Experimental groups and full contact metric

| Group | Misalignment range | Approximate available friction $\mu_S$ | Source's rounded required $\mu_S$ | Intended condition status |
|---|---|---:|---:|---|
| 1 | 0–6° | 0.1 | 0.5 | Friction condition violated |
| 2 | 0–6° | 0.6 | 0.5 | Friction condition satisfied |
| 3 | 7–11° | 0.6 | 1.0 | Friction condition violated |
| 4 | 7–11° | 1.2 | 1.0 | Friction condition satisfied |

These required values use the paper's rounded $\mu_{\rm lim}$ values 0.2 and 0.4 divided by $\eta=0.4$. Direct evaluation gives approximately 0.213 and 0.406, so the displayed design values are rounded, not exact equalities. The tested successful coefficients still exceed the corresponding unrounded thresholds. Groups 1 and 3 demonstrate the selected failure cases; they do not prove that every violation of a sufficient bound must fail.

For three pressure-sensor sequences $S_j$, the normalized metric is

$$\delta=\frac13\sum_{j=1}^{3}\operatorname{RMSE}\left(S_j/S_{\rm ref},\mathbf1\right),\qquad
S_{\rm ref}=|f_{\rm meas}^{z_B}|/3.$$

It is dimensionless. Equal sharing of the measured total normal force gives zero; unequal sharing, sensor bias, or fluctuating readings raise it. Normalization should be interpreted over the contact interval, since division by nearly zero total force would be ill-conditioned. The PDF writes “RSME” in the equation but defines the intended root-mean-square error in the text.

| Force reference | Reported pressure-sharing metric $\delta$ |
|---|---:|
| 5 N | 1.1095 |
| 10 N | 0.8913 |
| 20 N | 0.3465 |

These values were checked directly in Figure 9(b). Figure 10 compares measured friction ratios for the successful groups, while Figure 11 shows the geometric gap decreasing with larger force. Pressure balance and gap are complementary measurements: one evaluates load distribution, the other geometric alignment. The force estimate used for feedback is momentum-based; the FT sensor and pressure sensors provide evaluation information. Active motion control remains on the other directions throughout the passive-alignment experiment.

## 12. Separating interaction forces from disturbances

**Source:** P12.

### 12.1 Why an additional sensor changes observability

Vehicle motion alone reveals the combined external wrench. It cannot generally distinguish a tool force from wind that produces the same resultant wrench. A six-axis FT sensor at the arm **base** supplies independent information about tool interaction, including its moment arm. The model assumes a known point contact with three force components, not an arbitrary six-axis tool wrench.

With body-frame tool offset $r_E$,

$$w_i=\begin{bmatrix}I\\[r_E]_\times\end{bmatrix}f_i,\qquad w_{\rm ext}=w_i+w_d.$$

FT measurements contain interaction, sensor bias, arm weight, vibration, and inertial contributions:

$$w_{FT}=w_b+w_a+w_{i,FT}+w_w.$$

The arm is assumed light and slowly moving, so its dominant effect is configuration-dependent gravity:

$$w_a=\begin{bmatrix}m_aR^Tg_W\\r_{\rm COG,a}\times m_aR^Tg_W\end{bmatrix}.$$

Bias is assumed approximately constant during a task. The general multibody model is introduced, then simplified for estimation/control; this is not the full dynamically coupled arm model of P10/P08.

### 12.2 EKF structure

The stored state has 22 components:

$$x=(r_W,q_{WB},v_W,\omega_B,f_i^B,f_d^W,\tau_d^B).$$

A four-component unit quaternion represents only three attitude degrees of freedom. Interaction force, world disturbance force, and body disturbance torque follow independent random walks. Input thrust and torque have additive process noise. Rotational prediction includes $r_E\times f_i$, gyroscopic torque, and center-of-gravity offset torque. Covariances determine how quickly each wrench component may change.

Measurements comprise position, quaternion, linear velocity, angular rate, FT force, and FT torque: 19 stored components. After gravity/bias compensation, the contact-related measurement is

$$\begin{bmatrix}f_m\\\tau_m\end{bmatrix}=\begin{bmatrix}I\\[r_{FT}]_\times\end{bmatrix}f_i+\text{noise}.$$

The wrench states for disturbances have no direct FT measurement; they are inferred through motion residuals. The standard EKF recursion is

$$\hat x^-=f(\hat x^+,0),\quad P^-=AP^+A^T+LQL^T,$$
$$K=P^-H^T(HP^-H^T+MRM^T)^{-1},$$
$$\hat x^+=\hat x^-+K(z-h(\hat x^-,0)),\quad P^+=(I-KH)P^-.$$

Section 12.5 records the checked quaternion-matrix dimensions, gravity signs, and missing inverse in the printed gain. Quaternion normalization/error-state handling is not fully specified by the source. The paper also inconsistently labels some body/world quantities and the dimensions of the interaction Jacobian. Frame-consistent restatement above should be distinguished from literal copying of those expressions.

### 12.3 Hybrid control and force-reference adaptation

A geometric PD motion controller is combined with PI contact-force control mapped through the tool Jacobian. Disturbance rejection is explicit:

$$w_{\rm act}=S_Mw_M+S_Fw_F-\hat w_d.$$

The method selects the body axis most aligned with the desired interaction force. Along the leading tool axis, it uses translational force control; for sideways forces, it changes attitude through torque control while preserving other motion directions. This is motivated by avoiding a collision of rotors/body with nearby surfaces before the tool reaches them.

Only one selected force component is controlled directly. Estimated force in uncontrolled components is subtracted from the workpiece-frame desired force, and the controlled component is adjusted for the cosine of frame misalignment. In consistent scalar normal-force notation, the principle is

$$f_{\rm controlled}=\frac{f_{n,\rm desired}-n^Tf_{\rm uncontrolled}}{n^T\zeta}.$$

This is an explanatory projection formula; the printed Eq. (36) uses vector notation that needs interpretation. A small denominator would make the correction ill-conditioned. Section 12.5 gives the selection-matrix and binary-variable checks. The branch in Eq. (33) correctly prints a not-equal sign; its apparent equality in extracted text was an extraction artifact.

### 12.4 Experiments and limits

The omnidirectional platform uses six independently tilting propeller groups, an Intel NUC and Pixhawk, a 230 g Rokubi FT sensor, and C++/ROS control and EKF at 200 Hz. A loosely connected safety tether is present. Tools: a 0.72 kg delta arm with 0.28 kg moving mass on a 4.6 kg flying robot, or a 72.5 g carbon stick with 0.395 m sensor-to-tip and 0.555 m body-origin-to-tip distance.

Experiments include L-shaped sliding with the arm fixed relative to the base, triangular sliding with the base fixed and delta arm moving, and transitions across perpendicular surfaces using the stick. Known virtual disturbances of 5 N per world axis provide quantitative observer tests; a fan supplies real aerodynamic disturbance. Force reference is initially 5 N then 3 N in the first sliding task. Interaction-force RMS is 0.54 N without virtual disturbance and 0.79 N with it; a transient error reaches 1.8 N. Active arm motion increases force variation, consistent with the quasi-static model limitation. Wind experiments show roughly 0.4 Nm disturbance-torque change and attitude error within 4.5°.

A momentum observer is compared using the same data and FT information. Matching its response speed gives more noise; reducing its noise increases delay. The EKF performs better in the reported comparisons. This is not a universal optimality result over all observer tunings. Reported practical force range is 2–10 N. Outdoor localization, lower-force interaction, larger ranges, and estimating an unknown contact point remain future work.

### 12.5 Verified equation and frame audit

Visual inspection of pages 4 and 6–10 resolved or sharpened the following points.

- **Gravity sign:** the notation table defines world gravity as $(0,0,-9.81)$, but source Eq. (39) subtracts the gravity vector from force divided by mass. With that table's convention, free-fall acceleration should add that vector. This is a printed convention inconsistency that must be resolved before implementation.
- **Rotational Coriolis term:** source Eq. (9) uses a rotational block $-I[\omega]_\times$. Multiplying this block by $\omega$ gives zero, whereas the usual rigid-body gyroscopic term is $\omega\times I\omega$. The later Eq. (47) includes the gyroscopic term. The printed early matrix and later dynamics therefore should not be treated as interchangeable without correction.
- **Gravity wrench dimensions:** the block-diagonal notation in Eq. (10), mixing a vector and a zero matrix, does not directly give a six-vector. A wrench must instead be assembled as a force–moment column, with gravity rotated to the chosen frame and any moment taken about the stated origin.
- **Contact Jacobian dimensions:** a point-force wrench map has the form $[I;[r]_\times]f$, a $6\times3$ matrix multiplying a three-vector. If written $J_i^Tf$, then $J_i$ must be $3\times6$, even though the text calls it $6\times3$. Moment arm and force must be represented in the same frame. Nearby body/world labels also require reconciliation.
- **Transport term:** Eq. (22) contains an angular velocity crossed with a rotated acceleration. Its units are acceleration per time, not acceleration. A velocity transport term is a possible intended expression, but the printed page does not establish the deployed formula.
- **PI placement:** the image of Eq. (25) confirms that the proportional and integral corrections are inside the parentheses multiplied by $J_i^T$. An extraction that places those corrections outside the wrench map is misleading.
- **Binary selection:** variables described as binary should take values in $\{0,1\}$, not the open interval $(0,1)$. The prose describing which value means free motion must also be reconciled with the force-selection formula.
- **Selection-frame dimensions:** Eq. (28) left-multiplies a $6\times6$ selection matrix by $R_{BE}\in SO(3)$, which is not dimensionally defined. For a wrench already expressed in the body frame, a corresponding projector is $S_F=A\Sigma A^T$, where $A=\operatorname{diag}(R_{BE},R_{BE})$ and $\Sigma$ is the diagonal selector in the end-effector frame. For a wrench expressed in the end-effector frame, a one-sided transform has a different meaning. The source's frame declarations must decide which operation is intended.
- **Axis choice:** Eq. (31) maximizes the absolute inner product with a candidate axis. This selects an axis independent of force sign; the force sign must be handled separately.
- **Extraction artifact cleared:** Eq. (33) actually prints $\zeta\ne\chi$. The not-equal sign was lost in extracted text. This branch should not be logged as a source error.
- **Complement selector:** the expression $\mathbf1-\zeta$ in Eq. (34) requires a one-hot selector rather than an arbitrary signed direction vector.
- **Force projection:** the $1/\cos\theta$ adjustment in Eq. (36) corresponds to choosing a magnitude whose normal projection equals the desired normal force. It does not by itself resolve the frames of every vector in the printed equation.
- **Angle bound, explanatory deduction:** selecting the nearest unsigned coordinate axis in three dimensions gives $\max_i|n_i|\ge1/\sqrt3$ for a unit normal, hence $\theta\le\arccos(1/\sqrt3)\approx54.74^\circ$. A universal $45^\circ$ bound requires an additional planar restriction; it does not follow for arbitrary three-dimensional normals.
- **Quaternion propagation dimensions:** Eq. (46) multiplies a quaternion by an angular-rate matrix, but Eq. (48) gives a three-dimensional skew matrix. For scalar-first Hamilton quaternions satisfying $\dot q=\tfrac12q\otimes(0,\omega)$, a compatible four-dimensional matrix is

$$\dot q=\frac12\begin{bmatrix}0&-\omega^T\\\omega&-[\omega]_\times\end{bmatrix}q.$$

  Other quaternion conventions change signs or multiplication order; the equation above is an explicit convention, not a transcription of Eq. (48).
- **Kalman gain:** the image of Eq. (69) confirms that the printed innovation-covariance factor has no inverse. The usual gain is

$$K=P^-H^T\left(HP^-H^T+MRM^T\right)^{-1}.$$

  This is a source-level omission, not merely an extraction artifact.
- **Quaternion measurement dimension:** a four-component quaternion measurement is incompatible with treating every measurement-noise block as $3\times3$ unless a local attitude-error representation is introduced. The stored state and measurement have 22 and 19 components; the corresponding quaternion-constrained manifold dimensions are 21 and 18. Normalization or an error-state formulation needs explicit treatment.

The force/torque measurements require bias and arm-gravity compensation. The separation model assumes sufficiently light and slow arm motion; unmodeled arm dynamics can enter the estimated interaction force. External force is represented in the world frame while the disturbance moment is represented in the body frame, so their apparent grouping into a wrench does not remove the need for frame bookkeeping.

Additional recorded implementation details: C++/ROS operation at 200 Hz; a 230 g Rokubi sensor; a 0.72 kg delta arm with 0.28 kg moving mass; a 4.6 kg aerial platform; and a 0.0725 kg stick. The reported geometric lengths include 0.395 m and 0.555 m. Filter initialization assumes zero external disturbance. Numerical covariance tuning is not supplied as a complete reproducibility table.

### 12.6 Measurement information, covariance, and observability

The contact force enters two independent parts of the model: the robot acceleration and the compensated FT wrench. With known contact location, the force-sensor rows provide a direct rank-three measurement of $f_i$. The torque rows add redundant geometric information through $[r_{FT}]_\times f_i$ and can improve noise rejection or reveal inconsistency. The cross-product matrix alone has rank two for nonzero $r_{FT}$; a torque-only measurement cannot recover a force component parallel to its lever arm.

Once $f_i$ is measured, the remaining translational residual can identify the body/world-consistent disturbance force, and the angular residual can identify the disturbance moment. Without the independent FT channel, adding a contact force and subtracting an equal resultant disturbance can leave vehicle motion unchanged. This is the basic ambiguity the extra sensor addresses. It remains conditional on known geometry, sufficient motion measurements, and an adequate actuator/arm model; an EKF does not resolve unobservable decompositions by naming separate states.

The random walks are modeling choices. They permit wrench estimates to change without a deterministic contact or wind model. Larger process covariance for a wrench state generally permits quicker variation at the cost of noisier estimates; smaller measurement covariance trusts the corresponding sensor more. The article defines process-noise blocks for actuator force, actuator moment, interaction force, disturbance force, and disturbance moment, but does not give a complete numerical covariance set. The covariance matrices $A,L,H,M$ in the recursion are derivatives of the discrete dynamics and measurement functions with respect to state and noise. They must be evaluated with the same discretization, coordinate ordering, and quaternion convention as the state update.

The printed measurement matrix has identity rows for the 13 stored pose/twist components, identity rows for contact force, and cross-product rows for its moment. If the contact location varies with arm configuration, the measurement matrix varies with that known configuration; calling the observation linear in the EKF state does not mean it is constant for the moving-arm experiment. Sensor bias and arm gravity must be removed consistently before applying that simplified observation. Otherwise the filter can label compensation error as interaction force.

For numerical covariance robustness, an implementation may use the Joseph update

$$P^+=(I-KH)P^-(I-KH)^T+K(MRM^T)K^T.$$

This is an explanatory implementation alternative, not the equation printed in the paper. Quaternion normalization and covariance projection, or an explicitly derived multiplicative/error-state filter, likewise require a defined implementation rather than an assumed correction.

### 12.7 Hybrid directions, changing surfaces, and force references

A force-selection matrix must act on a wrench expressed in its declared frame. If complementary diagonal selectors are used in one orthonormal frame, then $\Sigma_M+\Sigma_F=I$ and $\Sigma_M\Sigma_F=0$. After a common rotation, $S_M=A\Sigma_MA^T$ and $S_F=A\Sigma_FA^T$ preserve those properties. The controller's choice to release or control particular force/torque components is an additional design decision; merely having complementary matrices does not prove stable surface transitions.

The leading interaction axis points along the manipulator toward its tip. When the desired force is along that axis, translational force control is appropriate. For a side contact, the algorithm changes selected attitude directions so that the tool can reach the workpiece without bringing the vehicle into the near-surface forbidden region. Figure 3 explains this geometry. The body axis selected by the largest absolute projection approximates the desired force direction; the normal-force adaptation corrects the contribution of the uncontrolled components. Figure 4 illustrates that projection correction. Discrete selector changes can change which states are motion-controlled, so smooth Cartesian references alone do not make the hybrid control law globally smooth.

Each waypoint stores desired vehicle pose, workpiece-frame pose, and desired contact force in that frame. Polynomial interpolation and optimization produce smooth reference paths, sampled at 200 Hz. The workpiece frame is supplied by the task specification; the paper is not estimating arbitrary unknown surface geometry online. Disturbance rejection subtracts the estimated disturbance wrench after the required frame transformations, while the selected interaction force is deliberately maintained.

### 12.8 Experimental evidence and observer comparison

The fixed-arm L-shaped sliding trial separates base motion from arm motion. The commanded 5 N initial force is reduced to 3 N during sliding; the authors note that the larger initial force helps establish contact despite structural transients. A known virtual disturbance is injected into the actuation command after contact, with 5 N per world axis and zero commanded disturbance torque. This gives a known signal for estimator comparison, but differs physically from a fan's distributed aerodynamic loading. Figures 5–7 show the task, disturbance estimates, and normal-force response.

The fixed-body triangular trial moves the delta arm while maintaining 3 N normal force. Its larger variation is consistent with neglected arm dynamics and a changing force lever arm. Figures 8–9 therefore test a limitation of the simplified model as well as its capability. The carbon-stick experiment transitions between perpendicular surfaces; Figures 10–12 show the sequence, disturbance response, and normal-force tracking with the fan on and off. The reported attitude deviation below about 4.5° and disturbance-moment change of about 0.4 Nm are observations of those runs.

For comparison, the momentum observer is extended using the same measured interaction wrench so that the remaining residual estimates disturbance. It is not fairly described as an observer deprived of all FT information. Figures 13–14 show the response-speed/noise tradeoff: tuning for a response comparable to the EKF produces more noise, while smoothing it increases delay. No complete robustness or statistical optimality proof across all possible tunings is supplied. The practical 2–10 N range, indoor localization, known contact point, light/slow arm assumption, and unmodeled sensor dynamics delimit the demonstrated system.

## 13. Ground-effect-aware modeling and control

**Source:** P04.

### 13.1 Identified effects and measurement methods

The paper separates additional lift, leveling torque, rotor drag changes, and changes in control allocation. It uses a single-rotor force stand, an entire quadrotor on a six-axis sensor above a 1.2 m square adjustable board, and flight residuals. The ground board is tilted while the vehicle stays level to avoid contaminating torque measurements with vehicle gravity moments. CAD supplies inertial estimates; rank-based Spearman correlation tests examine monotonic relationships.

The nominal mixer maps squared rotor speeds to thrust and moments, with separately calibrated coefficients $k_T,k_{TX},k_{TY},k_I$. Body obstruction/asymmetry means roll/pitch torque coefficients need not equal the thrust coefficient. Detailed fitted parameters and motor-interface procedures are referred to external supplementary material, absent from `Source/`.

### 13.2 Lift and leveling torque

Additional thrust is modeled as

$$f_G=F_G(h)Tz_B,\qquad F_G(h)=\frac{g_2}{h^2+g_1}.$$

The variable $h$ measures distance from the rotor-plane center to the ground. Near-ground thrust increases are about 30% in the measured regime. The proposed leveling torque is

$$\tau_G^B=M_G(h)TR^T(z_B\times z_W),\qquad M_G(h)=\frac{g_5h}{(h^2+g_3h+g_4)^2}.$$

To motivate it, distribute thrust around a circle of diameter $b$, with local height $H(\theta)=h-(b/2)\sin\delta\cos\theta$. Linearizing $F_G(H)$ gives

$$|\tau_G|\approx-\frac{b^2T}{8}\sin\delta\,F_G'(h).$$

Because $F_G'(h)=-2g_2h/(h^2+g_1)^2$, the derivative-based torque peaks at an intermediate height rather than increasing monotonically toward the ground. Experiments show roughly thrust-proportional torque and small-angle linearity, with saturation beyond about 10° tilt. Source Eq. (14) ambiguously multiplies a claimed torque magnitude by an unnormalized cross product; the coefficient-versus-magnitude distinction must be preserved in a full derivation.

### 13.3 Allocation and drag conclusions

Measured roll/yaw allocation coefficient changes are below roughly 10%, compared with approximately 30% thrust changes. The authors treat torque allocation coefficients as effectively unchanged in their tested regime. Pitch-axis identification is limited by residual board tilt. This is an empirical approximation, not proof that every near-ground multirotor has invariant allocation.

A usual rotor-drag model is $f_D=-RD'\sqrt T R^Tv$. At heights 0.1 m and 2 m, the square-root thrust ratio predicts 0.9486, but measured horizontal drag ratios are 0.5963 and 0.6179. The paper therefore uses calibrated altitude-dependent drag,

$$f_D=-RD(h)R^Tv,$$

and explicitly states that the additional reduction mechanism is not theoretically explained.

### 13.4 Equivalent-inertia approximation and flatness

The authors relate leveling torque to the gravitational torque of a hypothetical payload below the vehicle, matching $m_0gr=M_G(h)T$. Their proposed effective inertia is

$$J'(h)=J+\operatorname{diag}\left(\frac{[M_G(h)T]^2}{mg^2},\frac{[M_G(h)T]^2}{mg^2},0\right),$$

with hover approximation $T\approx mg/(1+F_G(h))$. This should be treated as the paper's controller approximation, not an established exact equivalence between an external restoring torque and changing inertia for arbitrary motion. Moving a reference point in rigid-body dynamics requires additional translational/moment terms; a time-varying inertia interpretation also needs care. The paper ultimately acknowledges empirical tuning of $J'(h)$.

The simplified translational model is

$$a=-gz_W+(1+F_G(h))\frac Tm z_B-R\frac{D(h)}mR^Tv,$$

with thrust reconstruction

$$T=m\frac{z_B^T(a+gz_W)}{1+F_G(h)}.$$

Position and yaw are taken as flat outputs; attitude/body-rate reconstruction follows cited rotor-drag flatness work. The paper does not print all those derivations. It neglects time derivatives of $D(h)$ for near-constant-height trajectories. Thus its practical flatness construction is tied to these approximations; it is not an exact theorem for arbitrary full aerodynamics and rapidly changing altitude.

### 13.5 Combined feedforward and incremental inversion

Position/velocity feedback adds to reference acceleration; predicted drag and ground lift are subtracted as feedforward disturbances. Attitude error produces desired rate, then rate error produces desired angular acceleration. Model torque uses $J'(h)$, and the combined incremental controller is

$$\tau_{B,\rm des}=\hat\tau_B+J'(h)(\dot\omega_{\rm des}-\dot\omega_f).$$

The measured/filtered angular acceleration corrects remaining model errors. The paper describes this practically as altitude-dependent gain scheduling after fine tuning. The mixer inverse produces rotor speed squares, then a calibrated motor model and bidirectional telemetry support speed control. Section 13.7 records the gravity-unit and quaternion-product inconsistencies and the unresolved acceleration/thrust convention.

### 13.6 Experimental table

Hardware: 7-inch propellers, minimum rotor-plane ground distance 7.2 cm, CUAV V5+, APM firmware with motor-speed feedback at 200 Hz, 16×16 laser array at 100 Hz, Intel NUC, and MoCap/IMU EKF at 200 Hz. Lemniscate results from Table I:

| Acceleration compensation | Torque compensation | Height, m | Speed, m/s | XY RMS, cm | Z RMS, cm | Total RMS, cm | Max error, cm | Std. error magnitude, cm |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| None | None | 0.12 | 1 | 6.54 | 8.64 | 10.85 | 18.04 | 2.19 |
| INDI | None | 0.12 | 1 | 5.51 | 6.83 | 8.77 | 16.07 | 2.06 |
| Neural Land | None | 0.12 | 1 | 6.81 | 4.94 | 8.41 | 16.75 | 2.73 |
| Proposed | None | 0.12 | 1 | 7.12 | 1.94 | 7.38 | 16.45 | 3.05 |
| Proposed | Proposed | 0.12 | 1 | 4.35 | 0.97 | 4.45 | 11.97 | 2.32 |
| Proposed | None | 0.18 | 3 | 12.63 | 1.71 | 12.74 | 26.27 | 5.36 |
| Proposed | Proposed | 0.18 | 3 | 6.67 | 2.84 | 6.97 | 14.66 | 2.72 |
| Proposed | Proposed | 1.75 | 5 | 4.77 | 1.41 | 4.98 | 9.75 | 1.64 |

The reported 45.3% reduction corresponds to $(12.74-6.97)/12.74$ in the 3 m/s comparison. It should not be attached indiscriminately to every baseline. Unmodeled high-speed loss of ground-effect lift and incomplete drag physics are explicit limitations. Low-altitude tested speeds remain below the cited roughly 4 m/s induced-airflow scale; the 5 m/s test is at high altitude.

### 13.7 Model audit, derivative calculation, and additional error metric

The printed force balance in Eq. (1) uses $-gz_W$ beside forces even though $g$ denotes acceleration; the mass factor must be reconciled with later normalized equations. In the leveling-torque derivation, the computed torque magnitude already contains $\sin\delta$, while multiplication by $z_B\times z_W$ supplies another such factor. A consistent vector coefficient should distinguish the coefficient multiplying the cross product from the actual torque magnitude.

For the derivative kernel $h/(h^2+g_1)^2$, differentiation places its positive maximum at $h=\sqrt{g_1/3}$ when $g_1>0$. For the fitted denominator $h^2+g_3h+g_4$, the analogous stationarity condition is $3h^2+g_3h-g_4=0$, giving $h=(-g_3+\sqrt{g_3^2+12g_4})/6$ when it is a valid positive maximum. These are explanatory differentiations of the model, not extra measurements.

Replacing an external leveling torque by altered inertia cannot be an exact arbitrary-motion identity. A static tilted state provides a simple check: with $\omega=\dot\omega=0$, the original dynamics require actuator torque to balance the ground-effect torque; changing inertia alone generates no balancing torque. The effective-inertia construction is best recorded as an approximate compensation model. Likewise, the payload analogy's added inertia $m_0^2r^2/m$ differs from the exact two-point-mass center-of-mass expression $mm_0r^2/(m+m_0)$ and does not include all rigid-body inertia contributions.

The quaternion-error expression in Eq. (36) prints a product without an inverse or conjugate. Multiplying identical nonidentity attitudes does not generally yield identity, so the convention requires clarification. Under a common convention the relative rotation is $q^{-1}\otimes q_d$, with a consistent sign choice. The rotation-vector conversion

$$\frac{2\arccos q_w}{\sqrt{1-q_w^2}}q_v$$

has the local limit $2q_v$ near the positive identity; direct evaluation there needs a limiting expression. The acceleration/thrust equations also require an explicit convention for whether reference acceleration already includes gravity compensation.

Section 13.6 includes the complete Table I error metrics, including the standard deviation of error magnitude. The 3 m/s total-RMS reduction remains $(12.74-6.97)/12.74\approx45.3\%$. The text's roughly 0.2 m peak hover-angle error and roughly 0.18 m peak modeled torque are related observations, not identical quantities. Numerical aerodynamic coefficients, complete inertia values, gains, and calibration details are not all recoverable from the supplied article alone.

### 13.8 Measurement equations, mixer, and parameter units

The source's nominal mixer is

$$\begin{bmatrix}T\\\tau_B\end{bmatrix}=
\operatorname{diag}\left(k_T,\frac{\sqrt2b}{4}k_{TX},\frac{\sqrt2b}{4}k_{TY},k_I\right)
\begin{bmatrix}1&1&1&1\\-1&1&1&-1\\-1&1&-1&1\\-1&-1&1&1\end{bmatrix}
\begin{bmatrix}n_1^2\\n_2^2\\n_3^2\\n_4^2\end{bmatrix}.$$

Here $b$ is diagonal rotor spacing and $n_i$ is measured in rpm. The matrix signs depend on the depicted rotor numbering. Coefficients fitted per rpm squared cannot be substituted for the per-Hz-squared coefficients in P01. If $n_{\rm rpm}=60n_{\rm Hz}$, then a coefficient multiplying rpm squared becomes $3600$ times as large when expressed as a coefficient multiplying Hz squared. The yaw coefficient includes reaction-torque units; the roll and pitch coefficients include the geometry factors printed above.

The thrust-stand estimate is $\widetilde F_G=z_W^T\widetilde f/\widetilde T-1$ with the board level. In flight, after subtracting nominal gravity and thrust acceleration consistently, $\widetilde F_G=mz_W^T\widetilde a_{\rm ext}/\widetilde T$ in the relevant hover experiment. The former depends on a calibrated force sensor and rotor model; the latter also depends on acceleration/state estimation. Agreement between them supports the fitted curve over sampled heights but does not establish a universal aerodynamic law.

The paper uses rank correlation

$$r_s(x,y)=\frac{\operatorname{cov}(\operatorname{rank}x,\operatorname{rank}y)}{\sigma_{\operatorname{rank}x}\sigma_{\operatorname{rank}y}}.$$

Composite rotor-speed coordinates separate collective, roll, pitch, and yaw input patterns. The reported roll-moment correlations are 0.8400 with the roll input coordinate and −0.0937 with height. These support the approximation of relatively height-insensitive torque allocation in that test. A different paragraph prints a correlation with the collective-input coordinate while describing absence of tilt correlation; those labels do not match, so its 0.0054 value cannot be treated as an unambiguous tilt test without the external data.

Dimensional checks clarify the fitted quantities. $F_G$ is dimensionless, so $g_1,g_2$ have units of length squared. Since $\tau_G=M_GT(z_B\times z_W)$, $M_G$ has units of length. In the fitted expression, $g_3$ has units of length, $g_4$ length squared, and $g_5$ length to the fourth power. These are model-unit deductions; their numerical values are not supplied in the article's main text.

### 13.9 Ring derivation, drag approximation, and command sequence

For a uniform nominal thrust density $T/(2\pi)$ around radius $b/2$, the local height change is $-(b/2)\sin\delta\cos\theta$. To first order,

$$F_G(H(\theta))\simeq F_G(h)-\frac b2\sin\delta\cos\theta F_G'(h).$$

The constant term integrates to zero torque because $\int_0^{2\pi}\cos\theta\,d\theta=0$. The gradient term uses $\int_0^{2\pi}\cos^2\theta\,d\theta=\pi$, giving

$$|\tau_G|\simeq-\frac{b^2T}{8}\sin\delta F_G'(h).$$

The approximation requires the height variation around the ring to be small enough for the local linearization. It models the aggregate effect, not the actual four discrete interacting rotor wakes. Because $F_G$ decreases with height, the nearer side receives more lift and the moment tends to level the vehicle. Measured tilt saturation beyond roughly 10° is one reason not to extend the small-angle formula indefinitely. The total ring-force identity also holds only approximately after this linearization; second-order height variation need not average to zero lift correction.

The drag matrix is $D(h)=\operatorname{diag}(d_x(h),d_y(h),0)$. With $z_B=Re_3$, its world force is orthogonal to $z_B$, so projection of the acceleration equation along $z_B$ removes drag and yields Eq. (32). Nonzero axial drag would change that formula. Treating $D$ as constant while differentiating assumes $\dot D=D_h\dot h\approx0$ and $\ddot D=D_{hh}\dot h^2+D_h\ddot h\approx0$; nearly constant height is the justification given by the source.

The implemented hierarchy computes position/velocity feedback and model-based acceleration compensation, reconstructs desired attitude, converts relative quaternion error into a rotation vector, adds desired body-rate feedforward, and then adds desired angular-acceleration feedforward. The model-only torque uses $J'(h)\dot\omega_{\rm des}+\omega_{\rm des}\times J'(h)\omega_{\rm des}$. The combined method instead increments the estimated applied torque by $J'(h)(\dot\omega_{\rm des}-\dot\omega_f)$. The filtered measured acceleration and estimated applied torque must correspond in time for the increment to represent the intended local correction; the PDF does not supply every filter and delay value.

The mixer inverse produces squared rotor speeds. Negative solutions or motor limits require handling before taking square roots and commanding throttle; Eq. (42) itself is an unconstrained inverse, not a bounded allocation optimizer. Calibration, voltage compensation, and rotor-speed feedback are delegated partly to external material. The paper acknowledges fine-tuning the effective inertia, making its final use an altitude-dependent angular-control gain as well as a physical-model approximation.

### 13.10 Interpreting all comparisons

Figures 1–3 introduce the near-ground setting, frames, and measurement apparatus. Figure 4 compares fitted lift against measurements. Figures 5–6 isolate torque dependence on speed, height, and tilt. Figure 7 compares relative changes of mixer coefficients. Figure 8 compares high/low-altitude rotor-drag behavior. Figures 9–10 present the payload analogy and resulting effective inertia. Figure 11 compares hover-angle error versus height for uncompensated, model-based, incremental, and combined compensation.

The first four rows of Table I hold height and speed fixed while changing acceleration compensation without the proposed torque compensation. Row 4 improves vertical RMS markedly but does not have the best horizontal RMS. Row 5 adds torque compensation and improves total RMS from 7.38 to 4.45 cm, about 39.7%. Rows 6–7 isolate its contribution at 3 m/s, giving the reported 45.3% total-RMS reduction while vertical RMS increases from 1.71 to 2.84 cm. Row 8 is a higher-altitude 5 m/s demonstration, not a matched low-altitude comparison. None of those comparisons establishes a universal improvement on every axis.

The source defines hover error using an attitude difference but plots an angle-error quantity; quaternion subtraction norm, rotation-vector norm, and angular distance are not interchangeable without a stated conversion. The reference therefore retains the qualitative figure comparison and the exact trajectory table rather than inventing precise converted hover-angle measurements.

## 14. Dissertation preview: available and unavailable material

**Source:** P11. This is a 24-page preview, not the complete thesis. It contains the title, acknowledgments, abstract, table of contents, lists of tables/figures, epigraph, Chapter 1, and Chapter 2 through the beginning of Definition 2.3 on printed page 11. Later chapters listed in the contents are absent.

The introduction motivates computational efficiency, generality across robot morphologies, and validity beyond local linearizations. Its central argument is that an abstraction must preserve the features needed for the original task, with losses either absent or quantified. The research program connects flatness-based planning, homogeneous-space tracking, cascade stability, and symmetry-based learning.

Available introductory mathematics includes:

$$TQ=\{(q,v_q):q\in Q,v_q\in T_qQ\},\qquad T_q^*Q=(T_qQ)^*,$$

vector fields as sections of $TQ$, complete flows, Lie brackets and Lie derivatives, tangent maps $df$, partial derivatives, dual maps $df^*$, distributions/codistributions, annihilators/coannihilators, frames/coframes, and Einstein summation. Force is a covector; its pairing with velocity is power. A tangent bundle is generally not globally a product $Q\times\mathbb R^n$.

The metric section defines a smooth positive-definite bilinear form, tangent and dual norms, and angles:

$$\|v\|_\kappa=\sqrt{\kappa(v,v)},\qquad\|f\|_\kappa=\sup_{\|v\|_\kappa=1}\langle f,v\rangle,$$
$$\angle(v,w)=\arccos\frac{\kappa(v,w)}{\|v\|_\kappa\|w\|_\kappa}.$$

The preview ends while introducing admissible curves for defining length. The remainder of Riemannian geometry and Lie-group/bundle exposition is listed but unavailable.

The contents and introduction identify these later contributions:

- Chapter 3 expands geometric flat-output construction, including a continuum optimization problem, gauge freedom, and finite-element approximations. The introduction states numerical agreement with known examples, but the optimization equations and results themselves are absent. P02 supplies the earlier analytic theorem, not these missing numerical details.
- Chapter 4 develops task-space planning corresponding to P08.
- Chapter 5 develops homogeneous-space tracking corresponding to P05.
- Chapter 6 studies almost-global stability of cascades. The introduction states that almost-global subsystem stability, chain-recurrent sets consisting only of hyperbolic equilibria, and bounded forward trajectories yield a compositional result; growth conditions can establish boundedness. The full hypotheses and proof are absent and cannot be reconstructed as if reviewed.
- Chapter 7 develops the tracking-MDP/symmetry/RL work corresponding to P03.
- Chapter 8 discusses limitations and shows a prototype in the figure list; the chapter and image are absent.

The supplied preview therefore adds a research roadmap and foundational notation, but cannot serve as evidence that the complete thesis methods or proofs were examined.

### 14.1 The available foundational definitions, expanded

The opening commutative diagram, Eq. (2.1), expresses $g\circ f=h$: following either path between the same vertices gives the same map. This is the notation later used for tangent projections, sections, and bundle trivializations.

The chapter then motivates manifolds as configuration spaces that are locally Euclidean but may have different global topology. A chart is local, whereas a physical trajectory can traverse multiple charts. A diffeomorphism is a smooth bijection with smooth inverse; merely listing the same number of scalar coordinates is insufficient to establish one.

A tangent vector can be represented by the derivative of a curve through its base point. Two curves in first-order contact represent the same tangent vector. The tangent projection $\pi_Q(v_q)=q$ records that base point. A vector field $V$ is a section satisfying $\pi_Q\circ V=\mathrm{id}_Q$. A complete vector field has solutions for all real times from every initial point and generates a global flow. Smoothness guarantees local existence/uniqueness, not completeness; this is the distinction used when P08 discusses finite-time escape.

The original image of Eq. (2.6), PDF page 22, resolves the extraction difficulty: it uses the commutator of flows at times $\pm\sqrt t$, differentiated at zero. Its local-coordinate meaning can be written without the square-root notation as the explanatory identity

$$[V,W]^i=V^j\partial_jW^i-W^j\partial_jV^i.$$

Equivalently, $[V,W]f=V(Wf)-W(Vf)$ for smooth scalar $f$. It measures failure of successive infinitesimal motions to commute. The adjoining text switches the letters of the vector fields when naming their flows; that is a notation mismatch, not a new definition. The scalar Lie derivative is simply

$$\mathcal L_Vf=df(V)=\left.\frac{d}{dt}f(\Phi_t^V(q))\right|_{t=0}.$$

For a smooth map $f:M\to N$, the tangent map sends the derivative of a curve to the derivative of its image:

$$df_q(v_q)=\left.\frac{d}{dt}f(\gamma(t))\right|_0,\qquad\gamma(0)=q,\quad\dot\gamma(0)=v_q.$$

It does not depend on the chosen representative curve. Partial differentials hold the other arguments fixed. A scalar differential $dP$ can be viewed as a covector field, allowing potential force to be expressed without choosing a metric first. The dual map is characterized by

$$\langle df_q^*\alpha_{f(q)},v_q\rangle=\langle\alpha_{f(q)},df_qv_q\rangle.$$

The base points matter: for a noninvertible $f$, a target covector does not select a unique source base point by itself. This fiberwise interpretation is the precise meaning of the compact notation $df^*:T^*N\to T^*M$ used in the preview. The kernel of $df$ consists of tangent directions invisible to the output map; those are the local self-motion directions in task planning.

A distribution assigns a tangent subspace at each configuration; a codistribution assigns a cotangent subspace. Regular means constant rank. Their annihilator/coannihilator are defined by zero natural pairing, requiring no metric. A kinetic-metric orthogonal complement is a different operation: it identifies vectors and covectors using the metric. Confusing the two would change P02's unactuation and orthogonality tests. A local frame spans the tangent bundle over a chart; it need not extend globally. Einstein summation abbreviates repeated-index contractions but does not remove the need to track tangent versus cotangent indices.

A positive-definite metric supplies vector norms and, by duality, force norms. If its local matrix is $M(q)$, the explanatory coordinate versions are

$$\|v\|_\kappa^2=v^TMv,\qquad\|f\|_\kappa^2=f^TM^{-1}f.$$

The angle formula requires two nonzero vectors in the same tangent space. The preview begins defining piecewise smooth admissible curves with nonvanishing interior velocity and then stops. The customary integral defining curve length is not printed in the supplied continuation, so no missing pages are being inferred from that introduction.

### 14.2 Front matter and roadmap boundaries

PDF pages 1–5 contain title/copyright material, acknowledgments, and abstract; pages 6–10 contain the contents; page 11 lists tables; page 12 lists illustrations; page 13 contains the epigraph. Pages 14–19 contain the introduction, and pages 20–24 contain the available start of Chapter 2. The contents establish the intended thesis structure, not access to the listed chapters. In particular, listed numerical flat-output discovery, finite-element experiments, and cascade theorems cannot be expanded into their missing formulas from their titles alone. Related standalone papers in this folder are documented under their own source IDs, preserving version and evidence boundaries.

## 15. Cross-paper synthesis and implementation implications

The comparisons below separate the papers’ mathematical guarantees from their experimental evidence.

| Question | Most relevant sources | Distinction to retain |
|---|---|---|
| How to derive coupled aerial-manipulator dynamics? | P10, P08 | Full inertial model versus quasi-static arm approximation |
| How to find a useful flat output? | P02, P08, P10 | Generic flatness versus end-effector/task flatness |
| How to track on a non-Euclidean space? | P05 | Homogeneous action, reference lift, and tangent-bundle convergence |
| How to reduce RL observations exactly? | P03 | Proved quotient symmetry versus heuristic error-only observations |
| How to handle model uncertainty? | P06, P04 | Adaptive approximation versus calibrated feedforward plus residual inversion |
| How to anticipate compliant motion? | P01 | Predicting impedance states versus reacting to a frozen modified reference |
| How to interact without onboard FT sensing? | P09, P07 | Total-wrench observer bias versus true tool contact |
| How to separate contact and wind? | P12 | Additional FT information and known contact location |
| How to exploit contact rather than oppose it? | P07 | Released angular axes with friction and restoring-moment conditions |
| How does nearby ground change flight? | P04 | Lift, leveling torque, drag, and allocation are separate effects |

A possible architecture combining ideas would use a feasible planner, a manifold-aware tracker or symmetry-reduced learned policy, an interaction layer, and a frame-consistent wrench estimator/allocation model. That combination is an engineering proposal, not a result established jointly by the papers. Combining individually successful modules does not automatically preserve a stability proof, symmetry, flatness, passivity, or contact feasibility.

Especially consequential distinctions are: actuator rank versus bounded wrench feasibility; output-map coverage versus reconstruction regularity; simulation consistency versus robust feedback performance; unknown tool contact versus unexplained total wrench; energy conservation versus asymptotic stability; and empirical approximate models versus exact geometric theorems.

### 15.1 Assumptions and guarantees that do not transfer automatically

| Source family | Mathematical or empirical result | Conditions that matter when combining it with another method |
|---|---|---|
| P10 | Algebraic task-pose reconstruction for the special arm | Center of mass fixed in tool frame; nonzero thrust; regular joint branch; no substantial unmodeled contact |
| P08 | General flatness and an ODE family realizing a task | Regular task inversion and shape dynamics; existence over the horizon; explicit extra constraints if required |
| P02 | Sufficient geometric-flatness construction | Suitable group dimension, orthogonal section, regular implicit equations; topology handled by local sections |
| P05 | Almost-global asymptotic tracking and local exponential error convergence | Fully actuated modeled mechanical system, invariant design metric, navigation function, bounded reference-lift velocity |
| P03 | Exact quotient MDP and value-preserving policy lift | Reward/transition symmetry, free proper action, valid measurable maps; deployment environment must preserve the assumptions |
| P06 | Adaptive tracking analysis and simulated improvement | Stable intended gain signs, approximation/excitation assumptions, regular kinematics; printed attitude proof has unresolved issues |
| P01 | Improved predicted compliance and tested wall interaction | Force extrapolation, actuator bounds, chosen model and weights; no general contact-stability or feasibility theorem |
| P09 | Six-axis compliant hardware interaction with improved allocation modeling | Observer/model accuracy, bandwidth separation, region scheduling and wrench feasibility |
| P07 | Sufficient friction and restoring-moment design conditions | Flat rigid support, known geometry, bounded disturbances, sufficient friction; impact/contact switching simplified |
| P12 | Experimental separation and rejection of disturbance during contact | Independent compensated FT information, known contact point, light/slow arm, consistent frames and filter tuning |
| P04 | Calibrated ground-effect models and tracking improvements | Tested height/speed regime, approximated drag and torque, calibrated platform, empirically adjusted effective inertia |
| P11 | Foundational definitions and thesis roadmap | Only the preview is available; later theorem and numerical-discovery chapters are not evidence in this folder |

Contact introduces both constraints and external generalized forces. A free-flight flatness proof does not survive arbitrary contact merely by adding a desired wrench to an output. The contact model must specify whether position, force, or a hybrid of them is commanded and whether the resulting mechanical equations retain the required inversion structure. Similarly, an impedance filter can modify an otherwise feasible trajectory into one outside actuator or joint limits; P01 addresses prediction and bounds within its model, but not all possible environment dynamics.

The word “symmetry” has different operational meanings across the papers. P02 can use a kinetic/control symmetry with potential symmetry broken, because its proof concerns elimination of shape derivatives in projected mechanical equations. P03 must preserve the tracking MDP's reward and reduced transitions. P05 needs an invariant design metric under a transitive action to transport error dynamics. A ground plane, obstacle, or contact surface can break the learning symmetry while still allowing a mechanical coordinate construction. These distinctions prevent the mistaken inference that the same group reduction is valid in all three contexts.

Three sources of uncertainty also differ. P06 approximates terms missing from inverse dynamics, including deliberate implementation omissions. P09 estimates the total unexplained wrench and can mistakenly comply with modeling error. P12 adds independent contact information to separate interaction from the remaining disturbance. P04 reduces a particular model bias before an incremental correction. The estimates cannot be interchanged without checking what each residual contains and in which frame it is expressed.

### 15.2 Reconstructing a combined engineering design from these papers

A proposed combined system should first state its configuration, generalized velocity, wrench origins, and actuator constraints. Its planner then needs a reference that satisfies the chosen plant's unactuated equations. Its tracker must use the same reference derivatives and pose conventions. Its interaction layer must distinguish a desired compliance response from a rejected disturbance. Its estimator must account for tool weight and motion with enough independent measurements to support the claimed force decomposition. Finally, the allocator must realize the requested wrench within the rotor limits.

This ordering exposes concrete interfaces: P08 supplies feasible state/input references; P05 supplies a tracking law only for fully actuated or appropriately reduced dynamics; P03 supplies a policy representation only under its MDP assumptions; P01 or P09 supplies compliant reference behavior; P12 supplies a sensor-informed interaction estimate; P07 supplies a particular contact geometry and direction-selection strategy; P04 supplies altitude-dependent corrections. These are possible components, not a stability theorem for their composition.

Numerical results should be compared within their definitions. P10's near-machine-precision open-loop residual is a model-consistency result; P03's spread is across training seeds; P09 reports Cartesian and Euler-angle tracking errors; P07 reports normalized pressure sharing; P12 reports force RMS; P04 reports trajectory norms and hover behavior. Ranking those papers by their smallest numerical error would compare different tasks, units, plants, and evidence.

## 16. Coverage, source limitations, and reproducibility

This edition consolidates the twelve supplied files into source-specific technical sections, explanations of their derivations and algorithms, numerical tables, figure interpretations, and the indexes below. It preserves source inconsistencies instead of treating an undocumented correction as the authors' implementation. It is a technical synthesis, not a verbatim transcription of every page or an independent certification of every proof.

| Source | Reference sections | Material the supplied file does not resolve |
|---|---|---|
| P01 | 9–9.3 | Exact handling of prediction-grid versus feedback-rate state shifting; complete deployed code; general recursive-feasibility/contact-stability proof |
| P02 | 5–5.9 | Full planar-manipulator metric, a general implemented numerical section search, nonholonomic extension |
| P03 | 7–7.7 | Full PPO reproduction settings/code; hardware performance; physical frame/transport reconciliation in simplified dynamics |
| P04 | 13–13.10 | Complete fitted coefficients/calibration/filter values; external supplement; exact validity of the payload analogy; unresolved printed conventions |
| P05 | 6–6.6 | Saturated/uncertain-plant extension; general underactuated cascade theorem; hardware validation |
| P06 | 8–8.7 | Complete plant parameters referenced elsewhere; consistent corrected global quaternion proof; implemented resolution of printed sign/dimension issues |
| P07 | 11–11.3 | General impact/contact-switching proof, arbitrary surface geometry, complete dynamic friction behavior |
| P08 | 4–4.11 | Full numerical optimizer configuration, actuator/collision constraint handling, substantial contact-force planning |
| P09 | 10–10.7 | Complete region-boundary and map-continuity specification, external data/code, all printed controller/allocation conventions |
| P10 | 3–3.7 | Complete robust closed-loop hardware implementation and general-arm task flatness outside the special geometry |
| P11 | 14–14.2 | Everything after the start of Definition 2.3 on printed p. 11; listed later chapters and their experiments |
| P12 | 12–12.8 | Numerical covariance tuning, explicit quaternion filter implementation, unknown-contact-point estimation, full arm dynamics |

These are evidence boundaries, not unsaved reading tasks. The source PDFs are sufficient for the methods and claims described here, but not for exact reproduction of every reported implementation. Bibliographies, acknowledgments, author information, and publisher material were considered in identifying provenance and dependencies; they are not transcribed as technical derivations. External links appearing in the papers are not evidence that the linked code, videos, datasets, or cited papers were inspected.

### 16.1 Equation-audit interpretation

A dimensional contradiction can identify an invalid printed expression without determining the authors' intended replacement. For example, a $3\times3$ quaternion propagation matrix cannot multiply a four-component quaternion; several valid replacements exist depending on convention. The corrected EKF gain requires an inverse, but this alone does not determine the deployed covariance tuning. The P09 quadratic objective needs a consistent factor convention, but changing that factor and changing regularization have different effects. Those distinctions are retained in the source sections.

The explicit checks include original-page inspection of the high-risk quaternion, gain, wrench-map, selection, regression, and table expressions; algebraic/numerical checks of sphere-section matrices and derivative identities; numerical recalculation of the passive-alignment bounds and ground-effect improvement percentages; and document checks for source links, heading links, tables, and math delimiters. No plant/controller code was changed, no external experiment was replicated, and no software release or thesis continuation is inferred from the local PDFs.

## 17. Notation, frames, units, and symbol collisions

### 17.1 Geometric and mechanical notation

| Symbol | Meaning here | Important qualification |
|---|---|---|
| $Q,q$ | Configuration manifold and configuration | In quaternion sections $q$ can instead denote a four-component attitude quaternion |
| $TQ,T^*Q$ | Tangent and cotangent bundles | Include the base configuration; not generally global Cartesian products |
| $v,\xi,\nu$ | Velocity, twist, or quasi-velocity | Ordering and reference frame vary across papers |
| $R_{AB}$ | Rotation taking $B$ coordinates into $A$ coordinates | Some sources label the same transformation inconsistently; use the equation's mapping |
| $g\in SE(3)$ | Rigid pose | Distinct from gravitational acceleration $g$ and gain coefficients $g_i$ |
| $\widehat a,[a]_\times$ | Cross-product matrix | $\widehat a b=a\times b$; Lie-group hats may embed a full twist instead |
| $M,J,\mathcal I$ | Generalized mass matrix, rotational inertia, spatial inertia | $J$ also denotes a Jacobian or objective in other sections |
| $\kappa,\flat,\sharp$ | Metric and vector/covector identifications | Depend on the selected physical or design metric |
| $\nabla$ | Covariant derivative/connection | Scalar directional differentiation is a special case; not arbitrary entrywise differentiation |
| $\Phi,G,\pi,\sigma$ | Group action, group, quotient projection, section | $\pi$ also denotes an RL policy, and $G$ can be a gravity/allocation symbol |
| $UQ,F,\Delta$ | Unactuated distribution, control codistribution, derived distribution in P02 | $\Delta$ denotes an OCP residual in P01 |
| $s$ | Thrust direction on $S^2$ in flatness sections | An MDP state in P03; a Laplace variable in transfer functions |
| $\chi$ | Task-induced COM map, virtual admittance state, or parameter vector | P08, P01, and P06 use different meanings; P12 also uses a leading-axis symbol |
| $P,Q,R$ in estimation | Covariance, process covariance, measurement covariance | Distinct from potential $P$, configuration manifold $Q$, and rotation $R$ |
| $\mu$ | Body moment, friction ratio, or other source-specific variable | In P07 $\mu_S$ is available static friction; $\mu$ is required tangential/normal ratio |
| $\lambda$ | Geometric COM offset, action moving frame, regularization, or basis function | Determine from the local source equation rather than a global substitution |
| $\delta$ | Virtual displacement, pressure metric, or tilt angle | P01, P07, and P04 respectively |
| $\gamma$ | Rotor thrust in P01; discount factor in P03 | Their ranges and physical units are unrelated |

### 17.2 Twist and wrench ordering

P10's body twists are ordered linear then angular. P03 defines its body twist as $(\omega,v)$, angular then linear. A $6\times6$ adjoint or inertia matrix copied between these conventions must be permuted. For linear-first twists, with $R$ mapping $B$ into $A$ and $p$ the position of $B$'s origin in $A$, a consistent illustrative transform is

$$\begin{bmatrix}v_A\\\omega_A\end{bmatrix}
=\begin{bmatrix}R&[p]_\times R\\0&R\end{bmatrix}
\begin{bmatrix}v_B\\\omega_B\end{bmatrix},$$

where the spatial/body twist origins are chosen consistently with that adjoint convention. Its dual force transform preserves power. More directly, a wrench at $B$'s origin expressed in $B$ transforms to a wrench about $A$'s origin expressed in $A$ as

$$f_A=Rf_B,\qquad\tau_A=R\tau_B+p\times Rf_B.$$

A mere coordinate rotation about the same origin omits the moment shift. Tool-point velocity formulas in Section 2.2 use the opposite physical operation of evaluating velocity at an offset point; one should not identify that operation with a spatial adjoint without checking the origin and signs.

Quaternion signs $q$ and $-q$ represent the same rotation. A scalar-first Hamilton convention, multiplication side, body/world angular-rate convention, and error definition must all be fixed before differentiating. An additive quaternion update needs normalization or a compatible error-state construction. Euler roll/pitch/yaw error values in a result table are not interchangeable with a rotation logarithm or geodesic angle.

### 17.3 Units and numerical comparisons

| Quantity | Units or constraint | Where it matters |
|---|---|---|
| Force / torque | N / N·m | Tool moment shifts and mixed force/moment residual weights |
| Translational / rotational virtual mass | kg / kg·m² for ordinary displacement/angle coordinates | Equal numerical entries do not give equal physical quantities |
| Translational stiffness / damping | N/m / N·s/m | Static penetration-to-force and transient compliance |
| Rotational stiffness / damping | N·m/rad / N·m·s/rad | Local angular admittance interpretation |
| Rotor speed | Hz, rpm, or rad/s as explicitly stated | Squared-speed coefficients change quadratically with unit conversion |
| P03 position/rate table | cm, cm/s, rad, rad/s | Mean ± standard deviation over ten training seeds |
| P09 pose table | cm and degrees | Separate RMS and maximum errors; not one mixed-unit norm |
| P04 trajectory table | cm | Horizontal, vertical, total RMS, maximum norm, standard deviation of norm |
| P07 pressure metric | Dimensionless | Normalized by measured total force divided among three feet |
| Sphere variables | $s^Ts=1$, $s^T\dot s=0$ | Initial conditions, derivatives, and numerical integration |
| Rotation matrices | $R^TR=I$, $\det R=1$ | Reconstruction, interpolation, and frame changes |

For samples $e_i$, RMS means $\sqrt{N^{-1}\sum_i e_i^2}$ for a scalar, or $\sqrt{N^{-1}\sum_i\|e_i\|^2}$ for a vector norm. Maximum norm and standard deviation of norm are different statistics. When horizontal and vertical RMS use the same samples, total RMS satisfies $E_{\rm all}^2=E_{XY}^2+E_Z^2$ up to reporting precision. It is not $E_{XY}+E_Z$.

## 18. Source-equation and page locator

These are **equation-block locators**, covering the numbered development of each paper, not claims that every equation is a separate new result. Page numbers refer to the supplied PDF from page 1; links in Section 1 identify the files. The indicated reference sections discuss the block's formulas, assumptions, and caveats. Explanatory equations added here have no source-equation number.

| Source | Source equations | PDF pages | Subject | Reference section |
|---|---|---|---|---|
| P01 | (1)–(5) | 3 | Robot, actuator, force, and virtual-admittance dynamics | 9, 9.1 |
| P01 | (6a)–(8) | 4 | OCP, constraints, reference and residual | 9, 9.2 |
| P01 | (9)–(11b) | 5–6 | Experimental impedance parameters and free-flight response analysis | 9, 9.3 |
| P02 | (1)–(3) | 2 | Bundle horizontal/vertical velocity and mechanical definitions | 5.1–5.2 |
| P02 | (4)–(12) | 3 | Dynamics, projected feasibility, underactuation distribution, rocket | 5.2–5.5 |
| P02 | (13)–(18) | 4 | Planar manipulator and quadrotor actions/models | 5.6–5.7 |
| P02 | (19)–(25) | 5 | Orthogonality proof, implicit shape reconstruction, sections/rocket output | 5.3–5.6, 5.9 |
| P02 | (26)–(30) | 6 | Manipulator basis/output and quadrotor north/south sections | 5.6–5.9 |
| P03 | (1)–(5b) | 2 | MDP, homomorphism, policy lift, symmetry | 7.1–7.2, 7.6 |
| P03 | (6)–(23) | 3 | Particle and tracking-MDP construction; symmetry theorem proof | 7.1–7.3, 7.6 |
| P03 | (24)–(42) | 4 | Particle actions, quotient construction, representative independence | 7.2–7.3, 7.6 |
| P03 | (43)–(62) | 5 | Reduced observations, lifted policies, Astrobee/quadrotor models | 7.3, 7.5–7.7 |
| P03 | (63)–(67) | 7–8 | Appendix proof of free/proper inheritance | 7.2, 7.7 |
| P04 | (1)–(2) | 3 | Vehicle force/moment balance and mixer | 13.1, 13.8 |
| P04 | (3)–(5) | 4 | Flight residuals, rank correlation, composite rotor inputs | 13.1, 13.8 |
| P04 | (6)–(10) | 5 | Thrust, lift model, stand/flight identification | 13.2, 13.8 |
| P04 | (11)–(18) | 6 | Ring derivation, leveling torque, coefficient comparisons | 13.2–13.3, 13.7–13.9 |
| P04 | (19)–(24) | 7 | Drag and payload analogy | 13.3–13.4, 13.7 |
| P04 | (25)–(34) | 8 | Effective inertia, flatness approximation, force feedforward | 13.4–13.5, 13.9 |
| P04 | (35)–(43) | 9 | Attitude/rate/torque/motor commands, hover metric | 13.5, 13.7, 13.9–13.10 |
| P05 | (1)–(6) | 2 | Homogeneous metrics, navigation potentials, mechanical model | 6, 6.2, 6.5 |
| P05 | (7)–(11) | 3 | Lifted error, inverse relation, local lift representation | 6.1, 6.4–6.5 |
| P05 | (12)–(22) | 4 | Examples, controller, two-parameter proof, error dynamics | 6.1–6.4 |
| P05 | (23)–(35) | 5 | Sphere/Lie-group specializations and covariant differentiation | 6.3, 6.6 |
| P06 | (1) | 1 | System variables/kinematic setup | 8.1 |
| P06 | (2)–(12) | 2 | Kinematics, Jacobians, dynamics, input map | 8.1–8.2 |
| P06 | (13)–(27) | 3 | Acceleration inverse kinematics and hierarchical control | 8.1–8.2, 8.6–8.7 |
| P06 | (28)–(42) | 4 | Practical approximations, RBF adaptation, inner-loop proof | 8.2–8.3, 8.5, 8.7 |
| P06 | (43)–(51) | 5 | Outer-loop candidate/bounds and cascade perturbation | 8.3, 8.5–8.6 |
| P07 | (1)–(3) | 2 | Friction, rigid dynamics, motion command | 11 |
| P07 | (4)–(6) | 3 | Force PI, hybrid wrench, closed-loop dynamics | 11 |
| P07 | (7)–(13) | 4 | Contact geometry, force decomposition, friction bound | 11, 11.1 |
| P07 | (14)–(19) | 5 | Friction and rotation conditions, pendulum approximation | 11.1–11.3 |
| P07 | (20) | 6 | Pressure-sharing metric | 11.3 |
| P08 | (1)–(6) | 2 | Kinematics, COM coordinates, task/system formulation | 4.1 |
| P08 | (7)–(18) | 3 | Sparse dynamics, attitude bundle and flatness setup | 4.1–4.3 |
| P08 | (19)–(26) | 4 | Dynamic inversion and task spaces | 4.3–4.4 |
| P08 | (27)–(38) | 5 | Task inverse kinematics, DAE, shape ODE and initial conditions | 4.4–4.5, 4.10 |
| P08 | (39)–(41) | 6 | Shooting objective and stable-geometry criterion | 4.6–4.7, 4.10–4.11 |
| P08 | (42)–(45) | 7 | Zero dynamics, energy, task-flat limit | 4.7–4.8, 4.11 |
| P09 | (1) | 4 | Rigid-body model and reversible-rotor wrench | 10.1 |
| P09 | (2)–(3) | 5 | Allocation-map identification | 10.1, 10.4–10.5 |
| P09 | (4)–(6) | 7 | Pose feedback and bounded allocation | 10.2, 10.4–10.5 |
| P09 | (7)–(9) | 8 | Force/momentum observers and admittance | 10.2, 10.6 |
| P10 | (1)–(4) | 2 | Configuration, quasi-velocity, input and special geometry | 3.1–3.2 |
| P10 | (5)–(26) | 3 | Local derivation, body Jacobians, inertia, Coriolis, input map | 3.2–3.3, 3.7 |
| P10 | (27)–(39) | 4 | Inertial decomposition and transformed dynamics | 3.4 |
| P10 | (40)–(57) | 5 | End-effector flatness reconstruction through required torques | 3.5–3.7 |
| P11 | (2.1) | 20 | Commutative diagram and composition of maps | 14.1 |
| P11 | (2.2)–(2.4) | 21 | Tangent/cotangent bundles and vector fields | 14.1 |
| P11 | (2.5)–(2.9) | 22 | Flows, bracket, Lie derivative and differentials | 14.1 |
| P11 | (2.10)–(2.14) | 23 | Dual map, kernel, distributions and annihilators | 14.1 |
| P11 | (2.15)–(2.18) | 24 | Metric, vector/dual norms and angles | 14, 14.1 |
| P12 | (1)–(6) | 3 | Frames, configurations and multibody model | 12.1 |
| P12 | (7)–(13) | 4 | Reduced rigid dynamics and interaction mapping | 12.1, 12.5 |
| P12 | (14)–(17) | 5 | FT sensor, bias and arm-gravity compensation | 12.1, 12.6 |
| P12 | (18)–(27) | 6 | Motion/force errors, feedback and selectors | 12.3, 12.5, 12.7 |
| P12 | (28)–(31) | 7 | Hybrid selection and leading interaction axis | 12.3, 12.5, 12.7 |
| P12 | (32)–(37) | 8 | Force adaptation and disturbance rejection | 12.3, 12.5, 12.7 |
| P12 | (38)–(51) | 9 | Stochastic rigid dynamics and wrench random walks | 12.2, 12.5–12.6 |
| P12 | (52)–(71) | 10 | Stored state, measurements, noise, EKF recursion | 12.2, 12.5–12.6 |
| P12 | (72)–(76) | 11 | Measurement Jacobians/covariances and injected disturbance | 12.6, 12.8 |
| P12 | (77) | 14 | Momentum-observer comparison | 12.8 |

## 19. Figure, table, and experimental-evidence index

This index accounts for the numbered figure/table groups in the supplied papers, including explanatory diagrams as well as results. “Simulation” does not imply hardware validation, and snapshots do not provide a statistical success rate. Figures listed only in the dissertation contents are explicitly distinguished from figures actually present.

| Source | Figures/tables and PDF location | What they contribute and where discussed |
|---|---|---|
| P01 | Figs. 1–3, pp. 1–3 | Baseline/predictive architectures, vehicle frames, controller flow; Section 9 |
| P01 | Fig. 4 and Tables I–II, p. 5 | Representative simulated step response, physical parameters and OCP weights; Sections 9, 9.3 |
| P01 | Figs. 5–6, p. 6 | Nine impedance settings and low-mass motor response; Section 9.3 |
| P01 | Figs. 7–8, p. 7; Fig. 9, p. 8 | Wall snapshots, force/attitude/thrust traces, predicted reference evolution; Section 9.3 |
| P02 | Table I, p. 2; Figs. 1–3, p. 4 | Complete geometric-flatness catalog and rocket/manipulator/quadrotor geometry; Sections 5.5–5.9 |
| P03 | Table I and Fig. 1, p. 6 | All error metrics, learning curves, best-seed evaluation; Sections 7.4, 7.7 |
| P04 | Figs. 1–3, pp. 1, 3 | Near-ground scenario, frames and apparatus; Sections 13.1, 13.8 |
| P04 | Fig. 4, p. 4; Figs. 5–6, p. 5 | Lift fit and torque versus speed, height, tilt; Sections 13.2, 13.8–13.9 |
| P04 | Fig. 7, p. 6; Fig. 8, p. 7 | Allocation-coefficient variation and drag reduction; Sections 13.3, 13.8–13.9 |
| P04 | Figs. 9–10, p. 8 | Payload analogy and effective inertia; Sections 13.4, 13.7 |
| P04 | Fig. 11, p. 9; Table I, p. 10 | Hover comparison and all eight trajectory rows/metrics; Sections 13.6, 13.10 |
| P05 | Fig. 1, p. 1; Fig. 2, p. 6 | Sphere/satellite and aerial-robot numerical tracking; Sections 6.3, 6.6 |
| P06 | Fig. 1, p. 2; Fig. 2, p. 3 | Arm/vehicle frames and control hierarchy; Sections 8.1, 8.7 |
| P06 | Table 1, p. 5; Figs. 3–5, p. 6 | Controller gains; desired 3D tool path, end-effector pose-error norm, motion-loop errors with/without adaptation; Sections 8.4, 8.7 |
| P07 | Fig. 1, p. 1; Figs. 2–3, p. 2 | Interaction geometry, pendulum analogy, coordinate frames; Section 11 |
| P07 | Figs. 4–5, p. 3; Figs. 6–7, p. 4 | Hybrid controller, support sets, force plane, geometric angle bound; Sections 11–11.2 |
| P07 | Figs. 8–9, p. 5 | Friction curve, apparatus, four groups and three pressure metrics; Section 11.3 |
| P07 | Figs. 10–11, p. 6 | Successful-group friction ratios and geometric gap; Section 11.3 |
| P08 | Figs. 1–3, pp. 1, 2, 4 | General arm, COM cascade and task spaces; Sections 4.1–4.4 |
| P08 | Fig. 4, p. 6; Fig. 5, p. 7 | Feasible trajectory family/cost and favorable geometries; Sections 4.6–4.8, 4.10–4.11 |
| P08 | Fig. 6, p. 8 | Dynamic grasping, spray painting, six-axis pick/place simulations; Section 4.9 |
| P09 | Fig. 1, p. 3; Figs. 2–3, p. 6 | Vehicle frames, ten-region map, R3 validation violins; Sections 10.1, 10.4–10.5 |
| P09 | Fig. 4, p. 8; Tables 1–2, pp. 9–10 | Full controller, all pose metrics and task-specific admittance parameters; Sections 10.2–10.6 |
| P09 | Figs. 5–6, pp. 10–11 | Straight/tilted point-contact snapshots and traces; Sections 10.3, 10.7 |
| P09 | Figs. 7–9, pp. 12–13 | Sliding poses, contact loss, three allocation comparisons; Sections 10.3–10.7 |
| P09 | Figs. 10–11, p. 14 | Peg approach/insertion and compliance traces; Sections 10.3, 10.7 |
| P10 | Fig. 1, p. 2; Figs. 2–4, p. 6 | Three-body geometry, videography simulation, reconstructed state/input; Sections 3.1, 3.5–3.7 |
| P11 | Table/illustration lists, pp. 11–12 | Names/locations of later thesis results, not the results themselves; Section 14.2 |
| P12 | Fig. 1, p. 2; Table 1, p. 4; Fig. 2, p. 5 | Platform, symbols/frames, and force/motion-estimation architecture; Sections 12.1–12.3, 12.5 |
| P12 | Figs. 3–4, pp. 7–8 | Near-surface direction selection and force adaptation; Section 12.7 |
| P12 | Figs. 5–7, p. 12 | Fixed-arm L-shaped task, injected disturbance estimate, force response; Sections 12.4, 12.8 |
| P12 | Figs. 8–9, p. 13 | Moving-arm triangular task and force response; Section 12.8 |
| P12 | Figs. 10–12, pp. 13–14 | Perpendicular surfaces, fan response, normal-force tracking; Section 12.8 |
| P12 | Figs. 13–14, pp. 14–15 | EKF/momentum-observer speed/noise comparison; Section 12.8 |

The numeric tables and selected labeled figure values are transcribed into their source sections. Unlabeled curves are interpreted qualitatively rather than converted into invented precise samples. Reconstructing full time-series datasets would require the original data or an explicitly separate digitization task.

## 20. Revision record

- **2026-10-09:** saved the initial source-by-source compilation and the resumed equation-audit findings at the user's request.
- **2026-10-10:** integrated those audits and expanded Sections 6–12, including proof explanations, observer derivations, optimization details, and experimental metrics; saved another requested checkpoint.
- **2026-10-17:** expanded P04/P11 and the early-paper reconstruction details; integrated the last metrics; corrected allocation-trial labels and clarified RL-table statistics; added the assumptions comparison, notation guide, equation/page locator, and figure/table index. Replaced the superseded checkpoint task lists with the source limitations in Section 16.

The reference is consolidated around the supplied versions. Missing thesis chapters, external supplements, unreported parameters, and unresolved printed equations remain identified as source limitations rather than being filled with unsupported content.


#include 
// transforms: device pointer to N * L 4x4 matrices (row-major, float32).
// output: device pointer to N * L 4x4 world transforms.
// L: links per chain, N: instances (seeds or agents).
__global__ void batched_fk(const float* transforms, float* output, int L) {
  int idx = blockIdx.x;               // instance index
  if (idx >= gridDim.x) return;
  extern __shared__ float sA[];       // shared memory for current transform
  // load base transform (identity)
  float cur[16];                      // local accumulator (4x4)
  // initialize cur to identity
  for (int i=0;i<16;++i) cur[i]=(i%5==0)?1.0f:0.0f;
  const float* instance = transforms + idx * L * 16;
  float* out_inst = output + idx * L * 16;
  for (int l=0;l
\chapter{Chapter 9: When Things Break (And They Will)}
\section{Section 1: Singularity Hell}
\subsection{Item 1:  Detecting singularities with API tools}
Building on the Jacobian computation and numerical stability issues discussed earlier in the book, this subsection focuses on practical, API-friendly ways to detect impending singularities in running systems. The goal is to give engineers concrete measures, code, and operational guidelines that integrate cleanly with existing kinematics libraries.

Concept: what we need from an API
\begin{itemize}
\item A Jacobian matrix for the task-space frame of interest, computed at the current joint configuration. In Pinocchio this is obtained via \lstinline|pinocchio.getFrameJacobian(model,data,frameId,ReferenceFrame.LOCAL_WORLD_ALIGNED)| after forward kinematics. In MoveIt/ROS use \lstinline|RobotState::getJacobian(...)|. The Jacobian must be expressed in consistent units for linear and angular rows.
\item A lightweight, numerically robust test to run inside a control loop (e.g., 100–1000 Hz), or at planner checkpoints, that returns: (a) whether the configuration is singular or near-singular, (b) diagnostic measures for decision-making, and (c) confidence metrics for avoiding false alarms.
\end{itemize}

Theory: reliable scalar measures from the Jacobian
\begin{itemize}
\item Singular values. Let $J\in\mathbb{R}^{m\times n}$ with singular values $\sigma_1\ge\sigma_2\ge\cdots\ge\sigma_r\ge0$. Small $\sigma_i$ indicate directions with vanishing task-space gain.
\item Condition number: \begin{equation}[H]\label{eq:cond}
\kappa(J)=\frac{\sigma_1}{\sigma_r},
\end{equation} where a large $\kappa$ denotes ill-conditioning. Use the ratio to detect near singularity when $r=\min(m,n)$.
\item Yoshikawa manipulability: \begin{equation}[H]\label{eq:manip}
w(J)=\sqrt{\det\!\left(JJ^\top\right)}=\prod_{i=1}^{r}\sigma_i.
\end{equation} A small $w$ signals loss of reachable velocities.
\item Rank deficiency via thresholding: count singular values $\sigma_i$ with $\sigma_i<\epsilon$; if count>0 treat as degenerate subspace.
\end{itemize}

Important scaling note (units): If $J$ mixes linear and angular velocity rows, singular values are unit-dependent. Normalize angular rows by a characteristic length $L$ before SVD. Define a scaled Jacobian $J_s = \operatorname{diag}(1,1,1, L, L, L)\,J$ when rows are [linear; angular]. Choose $L$ from task geometry (e.g., 0.2–1.0 m for typical manipulators).

Example: thresholds and interpretation for a 6-DOF arm
\begin{itemize}
\item Use $\sigma_{\min}$ (smallest nonzero singular value) as primary indicator. Typical operational thresholds:
  \begin{itemize}
  \item safe: $\sigma_{\min}>10^{-2}$ (units-matched)
  \item caution: $10^{-3}<\sigma_{\min}\le10^{-2}$
  \item near-singular: $\sigma_{\min}\le10^{-3}$
  \end{itemize}
\item Alternative relative threshold: $\sigma_{\min}<\alpha\ \sigma_{\max}$ with $\alpha\in[10^{-6},10^{-3}]$ to avoid unit issues.
\item Manipulability $w<10^{-4}$ often indicates severe loss of dexterity for typical industrial arms, but scale depends on task units and normalization.
\end{itemize}

Practical algorithm (conceptual)
\begin{enumerate}
\item Compute forward kinematics and obtain Jacobian $J$ for end-effector.
\item Scale $J$ to form $J_s$ to resolve unit mismatch.
\item Compute economy SVD of $J_s$ to obtain singular values $\{\sigma_i\}$.
\item Compute diagnostic scalars: $\sigma_{\min}$, $\kappa(J_s)$, and $w$.
\item Compare against thresholds and return status plus suggested mitigation (slow down, avoid commanded direction, switch to null-space motion, replan).
\end{enumerate}

Code: lightweight Python detector that works with any library returning a Jacobian
\begin{lstlisting}[language=Python,caption={Detect singularity from a Jacobian matrix (numpy).},label={lst:svd_detect}]
import numpy as np

def detect_singularity(J, scale_L=0.5, rel_thresh=1e-4, abs_thresh=1e-6):
    """
    J: m x n numpy Jacobian (rows: linear then angular) or already scaled.
    scale_L: characteristic length for angular rows (meters).
    rel_thresh: relative small-singular threshold (sigma_min < rel_thresh * sigma_max).
    abs_thresh: absolute sigma_min threshold.
    Returns dict with sigma, cond, manipulability, rank_deficiency, status.
    """
    J = np.asarray(J, dtype=float)
    m, n = J.shape
    # If 6-row Jacobian, scale angular rows by L to convert to linear units.
    if m == 6:
        S = np.ones((6,1))
        S[3:,0] = scale_L  # scale angular rows
        Js = (S * J)        # broadcast multiply rows
    else:
        Js = J.copy()
    # economy SVD for speed (thin SVD)
    U, s, Vt = np.linalg.svd(Js, full_matrices=False)
    sigma_max = s[0] if s.size>0 else 0.0
    sigma_min = s[-1] if s.size>0 else 0.0
    manipulability = float(np.prod(s))      # product of singular values
    cond_number = float(sigma_max / max(sigma_min, np.finfo(float).eps))
    # thresholds: absolute AND relative to avoid unit issues
    near_singular = (sigma_min < abs_thresh) or (sigma_min < rel_thresh * sigma_max)
    rank_def = int(np.sum(s < max(abs_thresh, rel_thresh * sigma_max)))
    return {
        'sigma': s,
        'sigma_min': sigma_min,
        'sigma_max': sigma_max,
        'condition_number': cond_number,
        'manipulability': manipulability,
        'rank_deficiency': rank_def,
        'near_singular': bool(near_singular)
    }

# Example usage:
# J = get_jacobian_from_library(...)  # e.g., pinocchio.getFrameJacobian(...)
# info = detect_singularity(J, scale_L=0.3)
# if info['near_singular']: trigger_recovery()
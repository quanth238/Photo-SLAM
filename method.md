\begin{document}

\maketitle

\begin{abstract}
  To be written
\end{abstract}

% ============================================================
% 3. Overview Pipeline
% ============================================================
\section{Overview Pipeline}
\label{sec:overview_pipeline}
We summarize the baseline system (Photo-SLAM) to make the data flow, shared states, and our modification point unambiguous. Photo-SLAM maintains a shared \emph{hyper-primitives map} across four concurrent threads (localization, geometry mapping, photorealistic mapping, loop closure) \cite{huang2024photoslam}. Each hyper-primitive stores (i) an explicit geometric anchor used by feature-based SLAM, namely a 3D point with an associated ORB descriptor for establishing 2D--2D and 2D--3D correspondences, and (ii) an explicit 3D Gaussian parameterization for splatting-based rendering, including rotation, scaling, density/opacity, and spherical harmonics (SH) coefficients \cite{huang2024photoslam}. This joint representation is the key to Photo-SLAM’s ability to run a classical correspondence-based SLAM back-end while continuously optimizing a photorealistic 3DGS map online.

Photo-SLAM starts from a standard geometric bootstrapping step. Once sufficient 2D--2D correspondences between adjacent frames allow estimating an initial transformation, the system initializes the hyper-primitives map via triangulation and starts 2D--3D pose tracking \cite{huang2024photoslam}. \needsverif{The paper states the initialization requirement as “sufficient 2D--2D correspondences” but does not fully specify the exact numeric thresholds/policies for initialization and keyframe insertion; we will follow the released implementation and document the concrete policies.} During online operation, the localization thread processes each incoming frame by matching its 2D ORB keypoints to 3D map points and running a motion-only bundle adjustment that minimizes reprojection error with a robust Huber cost, solved by Levenberg--Marquardt \cite{huang2024photoslam}. When a keyframe is created, the geometry mapping thread performs local bundle adjustment over a window of covisible keyframes and points (a factor-graph optimization) to refine keyframe poses and 3D points and to increment the map with sparse hyper-primitives \cite{huang2024photoslam}.

In parallel, the photorealistic mapping thread renders keyframes using a tile-based 3DGS renderer and optimizes the Gaussian parameters $\Theta$ by minimizing a photometric loss combining $\ell_1$ and SSIM \cite{huang2024photoslam}:
\begin{equation}
\mathcal{L}_{photo}(\Theta)=(1-\lambda)\|I_r(\Theta)-I_{gt}\|_1+\lambda\left(1-\mathrm{SSIM}(I_r(\Theta),I_{gt})\right),
\label{eq:photoslam_photo_loss}
\end{equation}
where $I_r(\Theta)$ is the rendered image and $I_{gt}$ is the keyframe image. Because geometry mapping intentionally stays sparse for real-time feasibility, Photo-SLAM densifies the Gaussian map during photorealistic optimization via (i) gradient-driven split/clone operations (as in 3DGS-style densification) and (ii) a \emph{geometry-based densification} (Geo) branch \cite{huang2024photoslam}. Geo is triggered when a keyframe is created and converts \emph{inactive} 2D ORB features (keypoints without associated 3D points) into temporary hyper-primitives for photorealistic mapping: for RGB-D, depth directly projects them; for stereo, depth comes from stereo matching; for monocular, depth is heuristically inferred from nearby active features \cite{huang2024photoslam}. Photo-SLAM further stabilizes incremental optimization via Gaussian-Pyramid (GP) learning, a coarse-to-fine continuation scheme that progressively shifts supervision from coarse pyramid levels to fine levels \cite{huang2024photoslam}. Finally, loop closure corrects local keyframes and hyper-primitives by a similarity transformation, after which photorealistic optimization reduces drift-induced ghosting artifacts \cite{huang2024photoslam}.

\textbf{Intervention points and variants.}
\OURMETHOD consists of two mechanisms with clearly separated scope. First, \emph{\OURMETHOD (mapping-only)} replaces the monocular heuristic depth inside Photo-SLAM Geo densification with EDGS-inspired correspondence triangulation to seed temporary hyper-primitives for photorealistic mapping; this variant does \emph{not} change the per-frame tracking thread nor the geometric BA objective, and is therefore expected to affect mapping metrics (time-to-quality, PSNR/SSIM/LPIPS) rather than trajectory metrics. Second, \emph{DenseMatchFactor (full system)} reuses the same correspondences to augment the geometry-mapping local BA objective (Sec.~\ref{sec:method_dense_factor}, Eq.~\eqref{eq:local_ba_augmented}); this is the only component that can directly affect pose/trajectory metrics (ATE/RPE). Loop closure remains unchanged.


% ============================================================
% 4. Method (Our improvement)
% ============================================================
\section{Method}
\label{sec:method}

\begin{figure}[t]
    \centering
    \includegraphics[width=0.95\textwidth]{pipeline.png}
    \caption{Overview of the proposed pipeline architecture.}
    \label{fig:pipeline}
\end{figure}

\subsection{Problem statement and SLAM-specific constraints}
% \paragraph{Overview.}
% Upon keyframe insertion, Photo-SLAM geometry mapping commits its standard local BA update and publishes an immutable snapshot of the new keyframe (image, intrinsics, refined pose, and covisible neighbors) \cite{huang2024photoslam}. \OURMETHOD then runs asynchronously on this snapshot: for each neighbor pair $(i,j)$ it computes correspondences, performs budgeted oversampling and EDGS-style triangulation, and emits (i) a \emph{seed packet} for photorealistic mapping and (ii) optionally a \emph{pose-factor packet} for DenseMatchFactor (Sec.~\ref{sec:method_dense_factor}). The photorealistic mapping thread integrates seed packets at iteration boundaries and optimizes Gaussian parameters with the unchanged Photo-SLAM photometric loss and GP learning schedule \cite{huang2024photoslam}. Geometry mapping consumes pose-factor packets when they arrive and triggers a bounded post-BA refinement of the augmented objective (Eq.~\eqref{eq:local_ba_augmented}) without blocking real-time tracking. Backlog control (Sec.~\ref{sec:method_backlog_contract}) ensures \OURMETHOD never blocks the tracking thread and degrades gracefully under overload.

\label{sec:method_constraints}
Photo-SLAM’s Geo branch is designed to quickly add temporary hyper-primitives from inactive 2D features, but in monocular settings its depth initialization relies on a local heuristic from nearby active features \cite{huang2024photoslam}. This can place new primitives far from their true surface locations, increasing the optimization distance that Gaussians must travel before contributing usefully to rendering, and potentially forcing additional split/clone iterations to recover missing detail under a fixed real-time budget \cite{huang2024photoslam}. In contrast, EDGS demonstrates that initializing Gaussians by triangulating pixels from dense image correspondences can provide a stronger one-step geometric approximation that dramatically shortens the 3DGS optimization path and reduces dependence on iterative densification \cite{kotovenko2025edgs}. The core challenge is that EDGS is developed for offline reconstruction and relies on expensive dense matching, whereas an online SLAM system must respect strict latency and concurrency constraints.

\OURMETHOD addresses this gap by converting EDGS-style correspondence triangulation into a \emph{budgeted, keyframe-triggered, asynchronous} operator that produces triangulated seed hyper-primitives for photorealistic mapping without modifying the SLAM back-end. Two SLAM-specific constraints are made explicit. First, correspondence computation must fit a strict latency/throughput budget (Sec.~\ref{sec:method_budget}) to avoid unbounded mapping lag. Second, integration must be deterministic and crash-free under concurrency. To achieve this, we implement an asynchronous read-only worker, \emph{\OURMETHOD}, that consumes immutable keyframe snapshots and emits immutable \emph{seed packets} (and, when DenseMatchFactor is enabled, immutable \emph{pose-factor packets}). We enforce a single-writer contract over the Gaussian parameter arrays used by the splatting renderer: \OURMETHOD never mutates the global Gaussian state, while the photorealistic mapping thread is the only writer that appends seeds and updates Gaussian parameters. \needsverif{The exact synchronization boundary (where the baseline code already guarantees a consistent view of keyframe pose/image tensors) must be aligned with the released Photo-SLAM implementation to avoid write-write races.}

\subsection{\OURMETHOD inputs: keyframe neighbors and dense/semi-dense correspondences}
\label{sec:method_corr_latency}
EDGS selects, for each reference image, a set of neighboring views based on camera proximity and overlap (measured via projection matrices) and computes dense pixel correspondences using a pretrained matcher, producing a dense warp field and a confidence map \cite{kotovenko2025edgs}. In online SLAM, we already maintain a covisibility structure through local mapping and bundle adjustment \cite{huang2024photoslam}, so \OURMETHOD operates on keyframes and their covisible neighbors. Concretely, when a new keyframe $i$ is committed by geometry mapping, \OURMETHOD chooses up to $k$ neighbor keyframes $\mathcal{N}(i)=\{j_1,\dots,j_k\}$ from the covisibility set available after local BA \cite{huang2024photoslam}. \needsverif{We will specify the exact neighbor selection rule used in the implementation; the intended design is to prefer high-overlap covisible neighbors while avoiding extremely short baselines, to mitigate depth uncertainty from near-degenerate triangulation.}

For each keyframe-neighbor pair $(i,j)$, \OURMETHOD invokes a correspondence module $\mathcal{M}$ that returns a set of matches with confidence:
\begin{equation}
\mathcal{M}(I_i, I_j) \rightarrow \mathcal{C}_{ij} = \{(\mathbf{u}_i^m, \mathbf{u}_j^m, s_{ij}^m)\}_{m=1}^{M_{ij}},
\label{eq:corr_set}
\end{equation}
where $\mathbf{u}_i^m,\mathbf{u}_j^m$ are sub-pixel 2D coordinates and $s_{ij}^m \ge 0$ is a confidence score. This interface intentionally covers both (i) EDGS-style dense matchers that predict a warp/confidence per pixel \cite{kotovenko2025edgs} and (ii) lightweight semi-dense or sparse matchers that directly output a large set of correspondences with scores. EDGS primarily uses RoMa for offline quality \cite{kotovenko2025edgs}, but RoMa-class dense matchers are expensive for online robotics: RoMa supplementary reports 198.8\,ms per pair at $560{\times}560$ (batch size 8) on an RTX6000 GPU \cite{edstedt2024roma_supp}, and EfficientLoFTR’s benchmark table reports 302.7\,ms per pair for RoMa at $640{\times}480$ on an RTX3090 \cite{wang2024efficientloftr}. In \OURMETHOD, the triangulation module is agnostic to the specific matcher; we instantiate $\mathcal{M}$ with a lightweight alternative to meet online budgets, e.g., EfficientLoFTR, whose optimized variant reports 35.6/27.0\,ms per pair (FP32/mixed precision) at $640{\times}480$ on an RTX3090 in the same benchmark setting \cite{wang2024efficientloftr}. As a practical reference point, the sparse SuperPoint+LightGlue pipeline is reported at 31.9/30.7\,ms in that table \cite{wang2024efficientloftr,lindenberger2023lightglue}. \needsverif{We will profile end-to-end \OURMETHOD latency on the target platform, including image transfers, inference, and post-processing, and report measured milliseconds per keyframe pair and the resulting keyframe-to-map-update lag distribution.}

\paragraph{Why a learned matcher is not redundant with ORB (and deployment modes).}
Photo-SLAM already extracts ORB features for the tracking thread, which is latency-critical and runs per incoming frame \cite{huang2024photoslam}. OURS introduces a learned matcher only on the keyframe path (triggered at keyframe creation) and uses it for two purposes: (i) EDGS-style triangulated seed packets that shorten the photorealistic optimization path, and (ii) pose-factor packets that directly enter the local BA objective (Eq.~(11)). Thus, the additional compute is not on the per-frame critical path and is amortized by reusing the same correspondence computation for both mapping and pose refinement.

To accommodate different hardware budgets, the correspondence module $M$ is a pluggable component. We primarily study a semi-dense lightweight matcher (EfficientLoFTR) for its speed--robustness trade-off, but the same interface also supports sparse pipelines (e.g., SuperPoint+LightGlue) or lightweight matchers designed for latency-sensitive applications. \needsverif{We will profile end-to-end runtime and peak GPU memory for each instantiation of $M$ and report the resulting trade-offs on desktop and embedded platforms.}


\subsection{Budgeted correspondence sampling (EDGS-style oversample \texorpdfstring{$\rightarrow$}{->} geometric selection)}
\label{sec:method_sampling}
EDGS initializes splats by sampling correspondences from confidence-weighted dense matches and then retaining a subset that is most geometrically consistent under reprojection, which reduces the effect of outliers without requiring a dense densification loop \cite{kotovenko2025edgs}. We adopt the same high-level pattern in a budgeted online setting. For each pair $(i,j)$, \OURMETHOD first draws an \emph{oversampled} index set of size $N'$, without replacement, from a confidence-weighted distribution:
\begin{equation}
p(m)=\frac{s_{ij}^m}{\sum_{m'} s_{ij}^{m'}},\qquad \{m_\ell\}_{\ell=1}^{N'}\sim p(m)\ \text{(without replacement)},\qquad N'=\min(\gamma N, M_{ij}),
\label{eq:conf_sampling_discrete}
\end{equation}
where $N$ is the target number of seeds contributed by the pair and $\gamma>1$ is an expansion factor (EDGS uses an analogous oversampling step prior to geometric selection) \cite{kotovenko2025edgs}. This stage is purely budget control: $N$ and $k$ directly bound the total number of triangulations and thus the mapping compute cost. \needsverif{We will report the exact $(k,N,\gamma)$ used in all experiments and include a sensitivity analysis analogous in spirit to EDGS’s hyperparameter study.}

After triangulation (Sec.~\ref{sec:method_triangulation_ls}), \OURMETHOD selects exactly $N$ seeds among the $N'$ candidates by ranking a geometric consistency score computed from reprojection residuals, matching the EDGS intent of prioritizing geometrically plausible seeds over merely high-confidence correspondences \cite{kotovenko2025edgs}. \needsverif{We will specify whether the selection score uses max residual, sum residual, or a robust aggregation, and we will verify which variant best preserves online stability under SLAM motion patterns.}

\subsection{Triangulation formulation (EDGS least squares, implementation-aligned)}
\label{sec:method_triangulation_ls}
\paragraph{Notation.}
We denote by $\mathbf{P}^i$ the full pinhole projection matrix constructed from the refined keyframe pose and intrinsics in Photo-SLAM (i.e., $\mathbf{P}^i=\mathbf{K}^i[\mathbf{R}_i|\mathbf{t}_i]$), and use the same projection operator $\pi(\mathbf{P}^i\tilde{\mathbf{X}})$ consistently in both triangulation and pose residuals. \needsverif{We will document the exact camera convention (OpenCV/OpenGL, pixel vs normalized coordinates) used by the released Photo-SLAM implementation.}

Given a correspondence $(\mathbf{u}_i,\mathbf{u}_j)$, we triangulate a 3D seed position following EDGS’s least-squares triangulation derived from the standard projection constraints \cite{kotovenko2025edgs}. Let $\mathbf{P}^i$ and $\mathbf{P}^j$ denote the per-view homogeneous projection transforms used to project 3D points into each image (EDGS formulates the system in terms of projection matrices and solves it by least squares) \cite{kotovenko2025edgs}. For pixel coordinates $(u_i,v_i)$ and $(u_j,v_j)$, EDGS constructs four linear constraints by rearranging normalized projection equations, which can be written in the implementation-aligned row form:
\begin{equation}
A_1=\mathbf{P}^i_{:,0}-u_i\,\mathbf{P}^i_{:,2},\ \
A_2=\mathbf{P}^i_{:,1}-v_i\,\mathbf{P}^i_{:,2},\ \
A_3=\mathbf{P}^j_{:,0}-u_j\,\mathbf{P}^j_{:,2},\ \
A_4=\mathbf{P}^j_{:,1}-v_j\,\mathbf{P}^j_{:,2},
\label{eq:edgs_rows}
\end{equation}
stacked as $A=[A_1^\top;A_2^\top;A_3^\top;A_4^\top]\in\mathbb{R}^{4\times 4}$. Writing $A=[A_{xyz}\mid \mathbf{a}_1]$ with $A_{xyz}\in\mathbb{R}^{4\times 3}$ and $\mathbf{a}_1\in\mathbb{R}^{4}$, the inhomogeneous 3D position $\mathbf{x}^\star\in\mathbb{R}^3$ is obtained by:
\begin{equation}
\mathbf{x}^\star = \arg\min_{\mathbf{x}\in\mathbb{R}^3}\ \|A_{xyz}\mathbf{x}+\mathbf{a}_1\|_2^2,
\qquad \tilde{\mathbf{X}}=[\mathbf{x}^\star;1].
\label{eq:edgs_lstsq_final}
\end{equation}
This is equivalent to solving a least-squares system with right-hand side $b=-\mathbf{a}_1$, consistent with the EDGS triangulation derivation and its \texttt{lstsq}-style implementation \cite{kotovenko2025edgs}. \needsverif{We must verify coordinate conventions end-to-end: (i) whether $\mathbf{u}$ is in pixel coordinates or normalized coordinates at this stage, (ii) how $\mathbf{P}$ is constructed from Photo-SLAM poses/intrinsics, and (iii) consistency with the projection function $\pi(\cdot)$ used by Photo-SLAM’s reprojection residual. We will provide a synthetic sanity check where triangulated points reproject to the input correspondences within numerical tolerance.}

To support EDGS-style geometric selection, for each triangulated candidate we compute a reprojection consistency score using the same camera model as Photo-SLAM \cite{huang2024photoslam}. Denoting the projection to image coordinates by $\pi(\cdot)$, we evaluate
\begin{equation}
e_i=\|\mathbf{u}_i-\pi(\mathbf{P}^i\tilde{\mathbf{X}})\|_2,\qquad
e_j=\|\mathbf{u}_j-\pi(\mathbf{P}^j\tilde{\mathbf{X}})\|_2,\qquad
e=\max(e_i,e_j),
\label{eq:reproj_error}
\end{equation}
and retain $N$ seeds with the smallest $e$ among the $N'$ oversampled candidates (Sec.~\ref{sec:method_sampling}). We additionally enforce cheirality by rejecting candidates whose depth is negative in either view. These checks are purely geometric validity/consistency constraints and are aligned with the EDGS principle that good initialization reduces the subsequent optimization path length \cite{kotovenko2025edgs}. \needsverif{We will report the distribution of $e$ for accepted seeds and verify that cheirality rejection is sufficient to prevent catastrophic outliers in practice, without introducing brittle thresholds.}

\subsection{System integration: post-BA triggering and single-writer map updates}
\label{sec:method_system_contract}
\OURMETHOD is triggered only when a keyframe is created and only after geometry mapping has committed its current local-BA update, so that triangulation uses the best available refined poses at that time \cite{huang2024photoslam}. \OURMETHOD then runs asynchronously: it reads immutable keyframe data (images, intrinsics, refined poses, and the selected neighbor set) and produces an immutable seed packet containing (i) triangulated seed positions in the global/map frame and (ii) initial Gaussian attributes required by the splatting renderer (Sec.~\ref{sec:overview_pipeline}).

\paragraph{Seed Gaussian initialization.}
For each accepted triangulated seed, we initialize the Gaussian mean to the triangulated 3D position in the map frame. We initialize color/SH coefficients from the reference keyframe pixels at the matched locations (e.g., using the view-$i$ observation, or a simple average across the pair), and initialize rotation, scale, and opacity using the same default initialization scheme adopted by Photo-SLAM for temporary hyper-primitives created by Geo densification. \needsverif{We will document the exact initialization constants and whether SH is initialized from a single view or multi-view averaging, following the released Photo-SLAM implementation.} The initialized seeds are then refined by the unchanged Photo-SLAM photometric objective and GP learning schedule.


The photorealistic mapping thread integrates seed packets at iteration boundaries by appending them to the Gaussian parameter arrays and then optimizing them with the same photometric objective and GP learning schedule as baseline Photo-SLAM \cite{huang2024photoslam}. \needsverif{We will document the exact data structure used for the seed queue (e.g., bounded SPSC queue) and the snapshot mechanism that guarantees keyframe pose/image consistency, and we will report profiling showing that tracking latency (motion-only BA) is unchanged compared to the Photo-SLAM baseline.}

Importantly, \OURMETHOD is designed so that tracking does not depend on \OURMETHOD outputs. Tracking continues to use the baseline 2D--3D correspondence pipeline on established map points \cite{huang2024photoslam}, and \OURMETHOD seeds are introduced to accelerate and improve photorealistic mapping quality. Any trajectory improvement will only be attributed and claimed for the full variant that explicitly augments the pose objective via DenseMatchFactor (Eq.~\eqref{eq:local_ba_augmented}); the mapping-only \OURMETHOD variant is evaluated solely on mapping quality and time-to-quality metrics.

\subsection{Real-time feasibility as an explicit budget and throughput condition}
\label{sec:method_budget}
Real-time feasibility is enforced by an explicit budget on correspondence computation per keyframe. Let $T_{\mathcal{M}}(r)$ denote the measured runtime of matcher $\mathcal{M}$ at resolution $r$ on the target platform, and let $k$ be the number of neighbor pairs processed per keyframe. We require
\begin{equation}
k\,T_{\mathcal{M}}(r)\le B,
\label{eq:budget}
\end{equation}
where $B$ is the per-keyframe correspondence budget (in milliseconds) allocated to \OURMETHOD. This condition guides the choice of matcher class and operating resolution. RoMa runtimes reported in the literature (e.g., 198.8\,ms at $560{\times}560$ on RTX6000 \cite{edstedt2024roma_supp} and 302.7\,ms at $640{\times}480$ on RTX3090 \cite{wang2024efficientloftr}) illustrate that RoMa-class dense matchers are incompatible with tight online budgets, whereas efficient semi-dense or sparse pipelines can plausibly satisfy Eq.~\ref{eq:budget} (e.g., 35.6/27.0\,ms for optimized EfficientLoFTR, 31.9/30.7\,ms for SuperPoint+LightGlue in the same benchmark setting) \cite{wang2024efficientloftr}. \needsverif{Because \OURMETHOD runs asynchronously, the practical condition is throughput: \OURMETHOD must process keyframes faster than their creation rate to avoid queue growth. We will measure keyframe rate and \OURMETHOD throughput jointly and report end-to-end keyframe-to-seed-integration latency statistics.}

\subsubsection{Backlog Control and Graceful Degradation (Timeliness Contract)}
\label{sec:method_backlog_contract}
While Eq.~\eqref{eq:budget} bounds the per-keyframe compute of correspondence estimation, the practical real-time requirement in an asynchronous SLAM system is queue stability: the \OURMETHOD service rate must exceed the keyframe arrival rate, otherwise the backlog grows unbounded and induces stale map/constraint updates. This timeliness--concurrency bottleneck has been identified as a primary systems challenge for visual SLAM on resource-constrained platforms \cite{semenova2022slam_bottlenecks}. We therefore make the overload behavior explicit and deterministic.

Let $f_{\mathrm{kf}}$ denote the keyframe creation rate (keyframes/s) and let $T_{\mathrm{srv}}$ denote the measured end-to-end \OURMETHOD service time per keyframe (including image transfer, matcher inference, post-processing, and triangulation for $k$ neighbors). Queue stability requires
\begin{equation}
\eta \doteq f_{\mathrm{kf}}\,T_{\mathrm{srv}} < 1.
\label{eq:queue_stability}
\end{equation}
To enforce stability, we use \emph{bounded} queues with capacity $Q_{\max}$ for both seed packets and pose-factor packets. When a queue is full, \OURMETHOD \emph{never blocks tracking}; instead it performs overload shedding by discarding packets according to a fixed policy (e.g., drop-oldest to preserve recency). \needsverif{We will finalize and document the exact drop policy and $Q_{\max}$ in the released code.}

We further define a graceful-degradation hierarchy: if the system is overloaded ($\eta \ge 1$ or sustained high queue occupancy), \OURMETHOD reduces its workload by decreasing $(k, r, N)$ and/or skipping \OURMETHOD for selected keyframes, while leaving the Photo-SLAM tracking thread unchanged. \needsverif{We will report the measured queue occupancy, drop rate, and the resulting keyframe-to-integration lag distribution under normal and stress-test conditions.}


By producing geometrically better-initialized seeds within a strict budget, \OURMETHOD targets improved time-to-quality for photorealistic mapping under a fixed compute envelope, consistent with Photo-SLAM’s observation that rendering time and the number of visible hyper-primitives directly constrain how many optimization iterations can be afforded online \cite{huang2024photoslam}, and with EDGS’s finding that stronger initialization shortens the optimization path in 3DGS \cite{kotovenko2025edgs}.

% ============================================================
% 2.X DenseMatchFactor: coupling dense correspondences to pose BA
% ============================================================
\subsection{DenseMatchFactor: Coupling Dense Correspondences with Pose Optimization}
\label{sec:method_dense_factor}
The mapping-only variant of \OURMETHOD improves \emph{time-to-quality} by seeding Gaussians close to the true surfaces, shortening the photorealistic optimization path as motivated by EDGS \cite{kotovenko2025edgs}. However, in Photo-SLAM the camera trajectory is primarily governed by the geometric SLAM back-end (2D--3D correspondences and local BA), while photorealistic mapping consumes the resulting poses \cite{huang2024photoslam}. This creates the classical decoupling: a better map appearance does not necessarily imply a better trajectory. To explicitly target pose drift, we reuse the \emph{same} budgeted dense/semi-dense correspondences already computed by \OURMETHOD to create additional geometric constraints on keyframe poses, inspired by the success of dense constraints in deep SLAM systems (e.g., DROID-SLAM) and recent hybrid frameworks that fuse dense learned constraints into factor graph optimization \cite{teed2021droidslam,zhou2024dbafusion}. Importantly, we keep the per-frame tracking thread and the loop-closure pipeline unchanged; DenseMatchFactor is consumed only by the geometry mapping back-end and therefore never blocks real-time tracking \cite{huang2024photoslam}.

\paragraph{Dense match factor construction.}
For each keyframe-neighbor pair $(i,j)$ processed by \OURMETHOD (Sec.~\ref{sec:method_corr_latency}), we already obtain a correspondence set $\mathcal{C}_{ij}=\{(\mathbf{u}_i^m,\mathbf{u}_j^m,s_{ij}^m)\}$ and a sampled subset (Sec.~\ref{sec:method_sampling}). We further associate each sampled correspondence with a latent 3D point $\mathbf{X}_{ij}^m\in\mathbb{R}^3$ initialized by our EDGS-style least-squares triangulation (Sec.~\ref{sec:method_triangulation_ls}). We then define a bidirectional reprojection residual for the pair:
\begin{equation}
\mathbf{r}_{ij}^m(\mathbf{T}_i,\mathbf{T}_j,\mathbf{X}_{ij}^m)=
\begin{bmatrix}
\mathbf{u}_i^m-\pi(\mathbf{P}_i,\mathbf{X}_{ij}^m) \\
\mathbf{u}_j^m-\pi(\mathbf{P}_j,\mathbf{X}_{ij}^m)
\end{bmatrix}\in\mathbb{R}^{4},
\label{eq:dense_reproj_residual}
\end{equation}
where $\pi(\cdot)$ is the same camera projection used by Photo-SLAM’s reprojection errors \cite{huang2024photoslam}. We weight each residual by a confidence-derived scalar
\begin{equation}
w_{ij}^m=g(s_{ij}^m),\qquad g(\cdot)\ \text{monotone increasing},
\label{eq:conf_weight}
\end{equation}
and use a robust Huber loss $\rho(\cdot)$ consistent with Photo-SLAM’s motion-only BA to reduce sensitivity to outliers \cite{huang2024photoslam}:
\begin{equation}
\mathcal{L}_{dense}=\sum_{(i,j)\in\mathcal{E}_{kf}}\sum_{m\in\mathcal{S}_{ij}}
\rho\!\left(w_{ij}^m\left\|\mathbf{r}_{ij}^m(\mathbf{T}_i,\mathbf{T}_j,\mathbf{X}_{ij}^m)\right\|_2^2\right),
\label{eq:dense_factor_loss}
\end{equation}
where $\mathcal{E}_{kf}$ denotes the set of keyframe-neighbor pairs selected from covisibility (Sec.~\ref{sec:method_corr_latency}) and $\mathcal{S}_{ij}$ is the sampled index set for that pair (Sec.~\ref{sec:method_sampling}). This formulation makes the coupling explicit: $\mathcal{L}_{dense}$ directly constrains the pose variables $\mathbf{T}_i,\mathbf{T}_j$.

\paragraph{Integration into Photo-SLAM local BA (LM solver augmentation).}
Photo-SLAM’s geometry mapping performs local bundle adjustment (factor-graph optimization) to refine keyframe poses and 3D points, using Levenberg--Marquardt \cite{huang2024photoslam}. We augment the local BA objective by adding $\mathcal{L}_{dense}$:
\begin{equation}
\min_{\{\mathbf{T}_k\},\{\mathbf{X}_p\},\{\mathbf{X}_{ij}^m\}}
\ \mathcal{L}_{ORB}(\{\mathbf{T}_k\},\{\mathbf{X}_p\})\ +\ \lambda_d\,\mathcal{L}_{dense}(\{\mathbf{T}_k\},\{\mathbf{T}_j\},\{\mathbf{X}_{ij}^m\}),
\label{eq:local_ba_augmented}
\end{equation}
where $\mathcal{L}_{ORB}$ is the baseline geometric BA objective induced by ORB map points and their reprojection constraints \cite{huang2024photoslam}. \paragraph{Residual balancing and stability of joint optimization.}
A direct sum of sparse ORB reprojection errors and semi-dense match reprojection errors can be unstable if their noise scales are mismatched. To make $\lambda_d$ principled, we set it by residual-scale normalization, interpreting each residual family as having an effective pixel noise level. Concretely, let $\sigma_{\mathrm{orb}}$ be a robust scale estimate (e.g., MAD) of the inlier ORB reprojection residuals after the baseline local BA update, and let $\sigma_{\mathrm{dense}}$ be a robust scale estimate of dense reprojection residuals (Eq.~\eqref{eq:queue_stability}) evaluated at the triangulated initialization before the post-BA refinement. We then set
\begin{equation}
\lambda_d = \alpha \cdot \frac{\sigma_{\mathrm{orb}}^2}{\sigma_{\mathrm{dense}}^2} \cdot \frac{|\mathcal{R}_{\mathrm{orb}}|}{|\mathcal{R}_{\mathrm{dense}}|},
\label{eq:lambda_scale_norm}
\end{equation}
where $|\mathcal{R}_{\mathrm{orb}}|$ and $|\mathcal{R}_{\mathrm{dense}}|$ denote the number of residual blocks in each family and $\alpha$ is a unitless scalar close to $1$. \needsverif{We will report the exact definition of $\sigma_{\mathrm{orb}}$ and $\sigma_{\mathrm{dense}}$ and validate the stability of Eq.~\eqref{eq:lambda_scale_norm} across monocular/stereo/RGB-D settings.}

In addition, we bound the influence of confidence-derived weights by normalizing $w_{ij}^m$ within each packet so that the average weight is $1$ (preventing uncalibrated confidence scales from dominating the Hessian). \needsverif{We will specify the exact normalization used and include an ablation showing sensitivity to this design choice.}

Finally, DenseMatchFactor is executed as a bounded post-BA refinement with a strict iteration cap, leveraging the trust-region behavior of Levenberg--Marquardt to prevent large destabilizing steps. \needsverif{We will report the iteration cap and confirm that tracking latency is unchanged.}


\paragraph{Scheduling and concurrency (non-blocking contract).}
To preserve Photo-SLAM’s real-time tracking, \OURMETHOD remains a read-only asynchronous worker. Besides the Gaussian seed packet, it also produces an immutable \emph{pose-factor packet} containing $\{(\mathbf{u}_i^m,\mathbf{u}_j^m,w_{ij}^m,\mathbf{X}_{ij}^m)\}$ for each processed pair. Geometry mapping is the single writer for pose/map-point states and consumes pose-factor packets at well-defined synchronization points. Concretely, geometry mapping triggers the bounded post-BA refinement \emph{upon receiving} a pose-factor packet for the newly inserted keyframe (possibly delayed due to asynchronous execution), and runs a small number of LM iterations on Eq.~\eqref{eq:local_ba_augmented} over the same local window. \needsverif{We will specify the exact trigger policy, iteration cap, and window definition in the released implementation, and show via profiling that the per-frame tracking latency is unchanged compared to Photo-SLAM.}

\paragraph{Complexity and budget consistency.}
DenseMatchFactor introduces no additional matcher calls: it reuses the correspondences already computed under the \OURMETHOD budget (Eq.~\eqref{eq:budget}) in Sec.~\ref{sec:method_budget}). The incremental cost is dominated by evaluating $O(kN)$ reprojection residuals per keyframe and the corresponding LM linearization. \needsverif{We will report the measured incremental geometry mapping time per keyframe and demonstrate that \OURMETHOD throughput remains above the keyframe creation rate to prevent backlog.} Since $\mathcal{L}_{dense}$ enters the pose objective explicitly (Eq.~\eqref{eq:local_ba_augmented}), this extension enables a principled evaluation of trajectory improvements (ATE/RPE), unlike the mapping-only \OURMETHOD variant which does not modify the pose residuals.

% ============================================================
% Experiment Plan (IROS-style, short and mechanism-aligned)
% ============================================================
\section{Experimental Plan}
\label{sec:exp_plan}
We evaluate both photorealistic mapping and trajectory accuracy, with a strict requirement that any reported ATE/RPE gains are attributable to explicit pose-objective modifications (Eq.~\eqref{eq:local_ba_augmented}).

\paragraph{Datasets and protocols.}
We follow the Photo-SLAM evaluation setting on Replica (NICE-SLAM version), TUM RGB-D, and EuRoC MAV, covering monocular, RGB-D, and stereo regimes \cite{huang2024photoslam}. \needsverif{We will document the exact sequence lists and camera configurations used by the Photo-SLAM scripts to ensure protocol consistency.} For EuRoC MAV we use the standard dataset with ground-truth trajectories \cite{burri2016euroc}. For TUM RGB-D we use the official benchmark and trajectory evaluation tool \cite{sturm2012tumrgbd}. We report results averaged over multiple runs to reduce nondeterminism, consistent with Photo-SLAM’s public evaluation guidance \cite{huang2024photoslam}.

\paragraph{Systems compared (mechanism-driven ablations).}
We compare: (i) Photo-SLAM baseline \cite{huang2024photoslam}; (ii) Photo-SLAM + \OURMETHOD (mapping-only), which improves time-to-quality but does not alter pose objectives; (iii) \OURMETHOD full (\OURMETHOD + DenseMatchFactor), which modifies the geometry mapping LM objective via Eq.~\eqref{eq:local_ba_augmented} and therefore can affect ATE/RPE; and (iv) \OURMETHOD w/o dense factors (ablation $\lambda_d=0$) to isolate the pose-coupling contribution. As external reference points for coupled 3DGS SLAM, we optionally include GS-SLAM and SplaTAM on RGB-D benchmarks where configurations are comparable \cite{yan2023gsslam,keetha2024splatam}. \needsverif{We will only report cross-method comparisons on datasets/sensor modalities supported by each method and under matched evaluation protocols.}

\paragraph{Metrics.}
For trajectory, we report ATE RMSE and relative pose error (RPE), computed with the official tools for TUM RGB-D and EuRoC \cite{sturm2012tumrgbd,burri2016euroc}. For photorealistic mapping, we report PSNR/SSIM/LPIPS over held-out views and time-to-quality curves at fixed compute budgets, consistent with Photo-SLAM’s focus on online mapping quality \cite{huang2024photoslam}. For robustness, we report tracking failure rate (number of lost frames or relocalization events) and sensitivity to low-texture sequences. For real-time feasibility, we report per-frame tracking FPS, geometry mapping latency per keyframe, \OURMETHOD throughput, and GPU memory usage.

\paragraph{Key ablations and sensitivity.}
We sweep the dense-factor strength $\lambda_d$, the number of neighbor pairs $k$, and sampled correspondences per pair $N$ (and $\gamma$ for oversampling) to reveal the accuracy--latency trade-off. We additionally ablate whether $\mathbf{X}_{ij}^m$ are optimized as variables in Eq.~\eqref{eq:local_ba_augmented} or treated as fixed triangulated points. \needsverif{We will determine which variant provides the best stability/runtime balance and report the chosen setting.}

\paragraph{Profiling and real-time evidence.}
We profile end-to-end \OURMETHOD latency on the target platform (desktop GPU and an embedded platform), including image transfer, matcher inference, and post-processing, and report how the chosen lightweight matcher satisfies the strict budget in Eq.~\eqref{eq:budget}. \needsverif{We will provide measured milliseconds per keyframe pair and demonstrate that tracking latency matches the Photo-SLAM baseline.} We also report the incremental LM time added by DenseMatchFactor and show that the system maintains real-time tracking, aligning with Photo-SLAM’s deployment motivation on embedded devices \cite{huang2024photoslam}.

\paragraph{Backlog and timeliness evaluation.}
Beyond average latency, we report queue stability indicators implied by Eq.~\eqref{eq:queue_stability}: (i) keyframe creation rate $f_{\mathrm{kf}}$, (ii) \OURMETHOD service time $T_{\mathrm{srv}}$ (end-to-end), (iii) queue occupancy over time, (iv) drop rate under the bounded-queue policy, and (v) the distribution of keyframe-to-integration lag for both seed packets and pose-factor packets. \needsverif{We will include stress tests that increase keyframe rate (or reduce available GPU budget) to validate graceful degradation without blocking tracking.}

\paragraph{DenseMatchFactor stability.}
We evaluate stability by sweeping $\lambda_d$ around the scale-normalized setting in Eq.~\eqref{eq:lambda_scale_norm} and reporting ATE/RPE, convergence behavior of local BA, and runtime. We also report the empirical residual scales $(\sigma_{\mathrm{orb}}, \sigma_{\mathrm{dense}})$ to justify the balancing rule, and include an ablation that disables weight normalization within packets to quantify sensitivity. \needsverif{We will additionally report failure cases (if any) where dense factors degrade performance, and analyze their characteristics (e.g., low texture, repeated patterns).}




\bibliographystyle{plain}
\bibliography{references}

\end{document}
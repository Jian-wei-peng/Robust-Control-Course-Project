# 课程项目方案：基于 LQR 与扰动观测器的差速移动机器人鲁棒轨迹跟踪控制

**Robust Trajectory Tracking Control of a Differential Drive Mobile Robot Using LQR and a Disturbance Observer**

## 1. Introduction

### 1.1 Background

轮式移动机器人（WMR）广泛应用于仓储物流、服务机器人、移动操作平台和自动驾驶小车等场景。轨迹跟踪控制是移动机器人导航与运动控制中的基础问题，其目标是使机器人位姿

$$
q=[x,\ y,\ \theta]^T
$$

准确跟踪给定参考轨迹

$$
q_r=[x_r,\ y_r,\ \theta_r]^T
$$

实际系统中，机器人往往会受到以下不确定因素影响：

- 搬运载荷导致质量 $m$ 和转动惯量 $I$ 变化；
- 不同地面材料导致摩擦和阻尼参数变化；
- 地面坡度、碰撞、风阻等造成外部扰动；
- 执行器动力学和建模误差导致控制输入与实际加速度不完全一致。

标准 LQR 控制器可以在名义模型下取得较好的跟踪性能，但当模型参数变化或存在未知扰动时，纯 LQR 的性能会下降。因此，本项目在 LQR 轨迹跟踪控制器的基础上叠加扰动观测器（Disturbance Observer, DOB），实时估计集总扰动并进行前馈补偿，从而提高闭环系统的鲁棒性。

### 1.2 Course Topic

本项目对应课程给定主题中的：

- **Robust control of nonlinear systems**；
- **Applications of robust control**；
- **Any other topic related to robust control**。

差速移动机器人本身是非线性系统。本项目使用参考轨迹附近的线性化误差模型设计 LQR 和 DOB 控制器，并在非线性动力学模型上进行 Python 仿真验证，因此既有理论设计，也有实际系统背景。

### 1.3 Main Idea

本项目采用两层思路：

1. 使用线性化误差模型设计标准 LQR：

   $$
   u_{LQR}=-K\xi
   $$

   其中 $\xi$ 是轨迹跟踪误差状态。

2. 将参数摄动、摩擦变化和外部扰动统一表示为输入通道中的集总扰动 $d$，设计扰动观测器估计 $\hat d$，并加入补偿项：

   $$
   u=-K\xi-\hat d
   $$

直观理解是：LQR 负责稳定名义系统并优化跟踪性能；DOB 负责估计未知扰动并反向抵消，从而提升鲁棒性。

## 2. Literature Review

移动机器人轨迹跟踪控制已有大量研究，常见方法包括运动学控制、PID 控制、LQR、非线性控制、滑模控制、自适应控制和鲁棒控制等。

- **运动学轨迹跟踪控制**：Kanayama 等提出的局部坐标误差变换是差速移动机器人轨迹跟踪中的经典方法。该方法将全局位姿误差投影到机器人坐标系下，便于分析误差收敛。
- **LQR 控制**：LQR 通过求解代数 Riccati 方程获得最优状态反馈增益，可以系统地平衡跟踪误差和控制能量，适合多变量耦合系统。
- **鲁棒控制方法**：$H_\infty$、滑模控制、自适应控制等方法可以提升系统对不确定性和外部扰动的抑制能力，但部分方法理论推导较复杂，或会引入高频抖振问题。
- **扰动观测器 DOB**：DOB 是工程控制中常用的鲁棒控制增强模块。它不改变主控制器的基本结构，而是通过估计集总扰动并进行补偿，提高系统抗扰性能。对于 matched disturbance，即扰动和控制输入作用在相同通道上的情形，DOB 特别有效。

相比直接使用微分博弈或 $H_\infty$ Riccati 方程，本项目采用 **LQR + DOB**。该方案理论复杂度适中，便于稳定性分析和数值仿真，也能清楚展示鲁棒控制的工程意义。

## 3. System Modeling

### 3.1 Kinematic Model

差速移动机器人的运动学模型为

$$
\dot{x}=v\cos\theta
$$

$$
\dot{y}=v\sin\theta
$$

$$
\dot{\theta}=\omega
$$

其中 $v$ 是线速度，$\omega$ 是角速度。

定义速度向量

$$
\eta=[v,\ \omega]^T
$$

参考速度为

$$
\eta_r=[v_r,\ \omega_r]^T
$$

### 3.2 Dynamic Model with Disturbance

考虑简化动力学模型：

$$
M\dot{\eta}+D\eta+\tau_d=\tau
$$

其中

$$
M=\mathrm{diag}(m,\ I)
$$

$$
D=\mathrm{diag}(d_v,\ d_\omega)
$$

- $m$：机器人质量；
- $I$：绕垂直轴的转动惯量；
- $d_v,d_\omega$：线速度和角速度阻尼系数；
- $\tau=[\tau_v,\ \tau_\omega]^T$：实际驱动力和力矩；
- $\tau_d=[\tau_{d,v},\ \tau_{d,\omega}]^T$：外部扰动或未知阻力。

名义参数记为 $M_0,D_0$。实际系统可写为

$$
(M_0+\Delta M)\dot{\eta}+(D_0+\Delta D)\eta+\tau_d=\tau
$$

其中 $\Delta M,\Delta D$ 表示质量、转动惯量和阻尼的不确定性。

## 4. Error Dynamics

### 4.1 Kanayama Tracking Error

采用局部坐标误差：

$$
\begin{bmatrix}
x_e\\
y_e\\
\theta_e
\end{bmatrix}
=
\begin{bmatrix}
\cos\theta & \sin\theta & 0\\
-\sin\theta & \cos\theta & 0\\
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
x_r-x\\
y_r-y\\
\theta_r-\theta
\end{bmatrix}
$$

速度误差定义为

$$
v_e=v_r-v,\quad \omega_e=\omega_r-\omega
$$

全状态误差为

$$
\xi=[x_e,\ y_e,\ \theta_e,\ v_e,\ \omega_e]^T
$$

仿真中需要将角度误差 $\theta_e$ wrap 到 $(-\pi,\pi]$，避免角度跨越 $\pi$ 时出现不真实的跳变。

### 4.2 Linearized Error Model

在参考轨迹附近

$$
x_e=y_e=\theta_e=v_e=\omega_e=0
$$

对误差动力学进行一阶线性化。位置和姿态误差近似满足：

$$
\dot{x}_e=\omega_r y_e+v_e
$$

$$
\dot{y}_e=-\omega_r x_e+v_r\theta_e
$$

$$
\dot{\theta}_e=\omega_e
$$

令实际输入写成

$$
\tau=\tau_{ff}+\delta\tau
$$

其中前馈项使用名义模型：

$$
\tau_{ff}=M_0\dot{\eta}_r+D_0\eta_r
$$

由

$$
M_0\dot{\eta}+D_0\eta+\tau_d=\tau
$$

可得名义速度误差近似：

$$
\dot{\eta}_e=-M_0^{-1}D_0\eta_e-M_0^{-1}\delta\tau+M_0^{-1}\tau_d
$$

为了得到标准状态空间形式，定义误差通道虚拟控制输入

$$
u=-\delta\tau
$$

则线性化误差模型可写为

$$
\dot{\xi}=A\xi+B(u+d)
$$

其中 $d\in\mathbb{R}^2$ 是输入通道中的集总扰动，包括外部扰动、摩擦变化和参数不确定性带来的等效影响。

系统矩阵为

$$
A=
\begin{bmatrix}
0 & \omega_r & 0 & 1 & 0\\
-\omega_r & 0 & v_r & 0 & 0\\
0 & 0 & 0 & 0 & 1\\
0 & 0 & 0 & -\frac{d_v}{m} & 0\\
0 & 0 & 0 & 0 & -\frac{d_\omega}{I}
\end{bmatrix}
$$

$$
B=
\begin{bmatrix}
0 & 0\\
0 & 0\\
0 & 0\\
\frac{1}{m} & 0\\
0 & \frac{1}{I}
\end{bmatrix}
$$

这里的 $u$ 是误差模型中的虚拟输入。实际反馈力矩为

$$
\delta\tau=-u
$$

因此若最终设计

$$
u=-K\xi-\hat d
$$

则实际输入为

$$
\tau=\tau_{ff}+K\xi+\hat d
$$

该符号约定表示：当 $v<v_r$ 时，$v_e>0$，反馈项 $K\xi$ 会倾向于增大实际驱动力。

### 4.3 Reference Trajectory Selection

由于 $A$ 依赖 $v_r,\omega_r$，若参考轨迹速度随时间变化，则线性化模型为 LTV 系统。为了使 LQR 和 DOB 设计保持简洁，本项目主仿真采用常速圆轨迹：

$$
x_r(t)=R_c\cos(\Omega t)
$$

$$
y_r(t)=R_c\sin(\Omega t)
$$

$$
\theta_r(t)=\Omega t+\frac{\pi}{2}
$$

对应

$$
v_r=R_c\Omega,\quad \omega_r=\Omega
$$

此时 $v_r,\omega_r$ 为常数，线性化误差模型是 LTI 系统，可以直接设计定常 LQR 和定常 DOB。

## 5. Controller Design

### 5.1 Baseline LQR Controller

首先忽略扰动 $d$，考虑名义线性系统：

$$
\dot{\xi}=A\xi+Bu
$$

定义二次型性能指标：

$$
J=\int_0^\infty(\xi^TQ\xi+u^TRu)dt
$$

其中

$$
Q=Q^T\succeq0,\quad R=R^T\succ0
$$

求解连续代数 Riccati 方程：

$$
A^TP+PA-PBR^{-1}B^TP+Q=0
$$

得到 LQR 增益：

$$
K=R^{-1}B^TP
$$

基础控制律为

$$
u_{LQR}=-K\xi
$$

若 $(A,B)$ 可稳定化且 $(Q^{1/2},A)$ 可检测，则闭环矩阵

$$
A_K=A-BK
$$

是 Hurwitz 矩阵，名义线性闭环系统渐近稳定。

### 5.2 Disturbance Model

考虑扰动后的误差模型：

$$
\dot{\xi}=A\xi+B(u+d)
$$

其中 $d=[d_1,\ d_2]^T$ 表示 matched lumped disturbance。这里的 matched 表示扰动和控制输入通过同一个矩阵 $B$ 进入系统。

实际含义包括：

- 外部力和力矩扰动；
- 负载变化造成的等效加速度误差；
- 摩擦变化造成的等效输入偏差；
- 前馈模型不准确造成的残差。

假设 $d$ 是常值或慢变信号：

$$
\dot d \approx 0
$$

这是 DOB 设计中常用的工程假设。对于阶跃扰动、分段常值扰动和慢变摩擦变化，该假设较为合理。

### 5.3 Augmented Disturbance Observer

构造增广状态：

$$
x_a=
\begin{bmatrix}
\xi\\
d
\end{bmatrix}
$$

增广系统为

$$
\dot{x}_a=
A_a x_a+B_a u
$$

其中

$$
A_a=
\begin{bmatrix}
A & B\\
0_{2\times5} & 0_{2\times2}
\end{bmatrix}
$$

$$
B_a=
\begin{bmatrix}
B\\
0_{2\times2}
\end{bmatrix}
$$

若假设跟踪误差状态 $\xi$ 可测，则输出为

$$
y=C_a x_a
$$

其中

$$
C_a=
\begin{bmatrix}
I_5 & 0_{5\times2}
\end{bmatrix}
$$

设计 Luenberger 型增广观测器：

$$
\dot{\hat{x}}_a=
A_a\hat{x}_a+B_a u
 +L(y-C_a\hat{x}_a)
$$

其中

$$
\hat{x}_a=
\begin{bmatrix}
\hat{\xi}\\
\hat d
\end{bmatrix}
$$

$L$ 是观测器增益矩阵。可以通过极点配置或 Kalman filter 思路选择 $L$，使

$$
A_a-LC_a
$$

为 Hurwitz 矩阵。

实际设计前需要检查增广系统 $(A_a,C_a)$ 是否可观或至少可检测。若全状态误差 $\xi$ 可测，且扰动通过速度误差通道进入系统，则该增广观测器通常可以估计常值或慢变 matched disturbance。

扰动估计值 $\hat d$ 由观测器状态的最后两维给出。

### 5.4 LQR + DOB Control Law

在基础 LQR 上加入扰动补偿：

$$
u=-K\xi-\hat d
$$

代入扰动系统：

$$
\dot{\xi}=A\xi+B(-K\xi-\hat d+d)
$$

得到

$$
\dot{\xi}=(A-BK)\xi+B(d-\hat d)
$$

定义扰动估计误差：

$$
\tilde d=d-\hat d
$$

则闭环误差系统为

$$
\dot{\xi}=A_K\xi+B\tilde d
$$

这说明 LQR + DOB 的鲁棒性来源非常清楚：若 DOB 能够使 $\tilde d$ 快速收敛到零，则闭环系统会恢复接近名义 LQR 的性能。

对应实际机器人输入为

$$
\tau=\tau_{ff}+K\xi+\hat d
$$

## 6. Stability Analysis

### 6.1 Nominal LQR Stability

对名义闭环系统

$$
\dot{\xi}=A_K\xi
$$

取 Lyapunov 函数：

$$
V=\xi^TP\xi
$$

其中 $P=P^T\succ0$ 是 CARE 的解。由 Riccati 方程可得

$$
\dot V
=-\xi^T(Q+K^TRK)\xi
$$

因此在无扰动且模型准确时，名义线性误差系统渐近稳定。

### 6.2 Observer Error Dynamics

定义增广观测误差：

$$
\tilde{x}_a=x_a-\hat{x}_a
$$

当 $d$ 为常值时，观测误差满足：

$$
\dot{\tilde{x}}_a=(A_a-LC_a)\tilde{x}_a
$$

若选择 $L$ 使 $A_a-LC_a$ Hurwitz，则

$$
\tilde{x}_a(t)\to0
$$

因此

$$
\tilde d(t)=d(t)-\hat d(t)\to0
$$

### 6.3 Robustness Interpretation

LQR + DOB 闭环系统为

$$
\dot{\xi}=A_K\xi+B\tilde d
$$

由于 $A_K$ Hurwitz，系统对输入 $\tilde d$ 是输入到状态稳定（ISS）的。也就是说：

- 如果 $\tilde d\to0$，则 $\xi\to0$；
- 如果 $d$ 慢变且 $\tilde d$ 有界，则 $\xi$ 最终有界；
- DOB 收敛越快，扰动对跟踪误差的影响越小。

更具体地，存在正常数 $c_1,c_2$，使得

$$
\|\xi(t)\|
\le
c_1e^{-c_2t}\|\xi(0)\|
+c_1\int_0^t e^{-c_2(t-\tau)}
\|B\tilde d(\tau)\|d\tau
$$

因此，DOB 的扰动估计误差越小，轨迹跟踪误差越接近名义 LQR 情况。

该稳定性分析不需要微分博弈理论，也不需要广义 Riccati 方程。它只依赖两个条件：

1. LQR 使 $A-BK$ Hurwitz；
2. DOB 使 $A_a-LC_a$ Hurwitz。

## 7. Python Simulation Plan

### 7.1 Software Environment

本项目使用 Python 完成数值仿真，不使用 MATLAB/Simulink。建议使用：

- `numpy`：矩阵计算；
- `scipy.linalg.solve_continuous_are`：求解 LQR Riccati 方程；
- `scipy.signal.place_poles` 或 Kalman filter 方法：设计 DOB 观测器增益；
- `scipy.integrate.solve_ivp`：仿真非线性机器人动力学和观测器；
- `matplotlib`：绘制轨迹、误差和输入曲线；
- `pandas`：整理仿真指标表格。

### 7.2 Nonlinear Plant for Simulation

虽然控制器基于线性化误差模型设计，仿真应使用非线性动力学模型：

$$
\dot{x}=v\cos\theta
$$

$$
\dot{y}=v\sin\theta
$$

$$
\dot{\theta}=\omega
$$

$$
\dot{\eta}=M(t)^{-1}\left(\tau-D(t)\eta-\tau_d(t)\right)
$$

其中 $M(t),D(t)$ 可随时间变化，以模拟负载变化和摩擦变化。

控制输入为

$$
\tau=\tau_{ff}+K\xi+\hat d
$$

其中

$$
\tau_{ff}=M_0\dot{\eta}_r+D_0\eta_r
$$

对于常速圆轨迹，$\dot{\eta}_r=0$，因此

$$
\tau_{ff}=D_0\eta_r
$$

### 7.3 Controllers for Comparison

仿真比较两个控制器：

1. **Pure LQR**

   $$
   u=-K\xi
   $$

   $$
   \tau=\tau_{ff}+K\xi
   $$

2. **LQR + DOB**

   $$
   u=-K\xi-\hat d
   $$

   $$
   \tau=\tau_{ff}+K\xi+\hat d
   $$

两个控制器使用相同的 LQR 增益 $K$。区别在于第二个控制器额外使用 DOB 估计并补偿集总扰动。

### 7.4 Simulation Scenarios

#### Scenario 1: Nominal Tracking

名义参数：

$$
M(t)=M_0,\quad D(t)=D_0,\quad \tau_d(t)=0
$$

预期结果：

- Pure LQR 和 LQR + DOB 都能稳定跟踪圆轨迹；
- DOB 估计值应接近零；
- 两者 RMSE 接近。

#### Scenario 2: Payload Change

在 $t=5s$ 时模拟机器人抓取重物：

$$
m(t)=
\begin{cases}
m_0,& t<5\\
m_0+\Delta m,& t\ge5
\end{cases}
$$

$$
I(t)=
\begin{cases}
I_0,& t<5\\
I_0+\Delta I,& t\ge5
\end{cases}
$$

控制器仍使用名义参数 $M_0,D_0$ 计算。该场景用于验证模型失配下的鲁棒性。

预期结果：

- Pure LQR 会出现更明显的瞬态误差和恢复时间；
- LQR + DOB 能估计负载变化造成的等效扰动；
- LQR + DOB 的 RMSE、ITAE 和峰值误差应更小。

#### Scenario 3: Friction Change and External Disturbance

在 $t=10s$ 时模拟地面摩擦变化：

$$
D(t)=
\begin{cases}
D_0,& t<10\\
D_0+\Delta D,& t\ge10
\end{cases}
$$

同时加入外部扰动：

$$
\tau_d(t)=\tau_{step}(t)+\tau_{noise}(t)
$$

其中 $\tau_{step}$ 是阶跃扰动，$\tau_{noise}$ 使用带限噪声或分段常值噪声。

预期结果：

- Pure LQR 仍应保持基本稳定，但误差峰值、RMSE 或恢复时间更大；
- LQR + DOB 对阶跃扰动和慢变摩擦变化的抑制更明显；
- DOB 对高频噪声不应设计得过快，否则可能放大测量噪声。

### 7.5 Suggested Nominal Parameters

可选名义参数：

$$
m_0=5\ \mathrm{kg},\quad
I_0=0.5\ \mathrm{kg\,m^2}
$$

$$
d_v=1.0,\quad
d_\omega=0.5
$$

圆轨迹参数：

$$
R_c=2\ \mathrm{m},\quad
\Omega=0.2\ \mathrm{rad/s}
$$

因此

$$
v_r=0.4\ \mathrm{m/s},\quad
\omega_r=0.2\ \mathrm{rad/s}
$$

LQR 权重可从以下设置开始：

$$
Q=\mathrm{diag}(20,\ 20,\ 10,\ 2,\ 2)
$$

$$
R=\mathrm{diag}(1,\ 1)
$$

DOB 观测器极点应比 LQR 闭环极点更快，但不宜过快，以免放大噪声。可以先选择为 LQR 闭环主导极点实部的 3 到 5 倍。

## 8. Evaluation Metrics

### 8.1 Position RMSE

$$
\mathrm{RMSE}=
\sqrt{
\frac{1}{N}\sum_{k=1}^{N}
\left(x_e^2(k)+y_e^2(k)\right)
}
$$

### 8.2 ITAE

$$
\mathrm{ITAE}
=
\int_0^T
t\sqrt{x_e^2(t)+y_e^2(t)}dt
$$

### 8.3 Peak Error

定义位置误差：

$$
e_p(t)=\sqrt{x_e^2(t)+y_e^2(t)}
$$

记录

$$
\max_t e_p(t)
$$

用于比较扰动瞬间的最大偏离。

### 8.4 Control Effort

反馈控制能量：

$$
\mathrm{CE}
=
\int_0^T
\left(
\delta\tau_v^2(t)+\delta\tau_\omega^2(t)
\right)dt
$$

其中

$$
\delta\tau=K\xi
$$

对于 LQR + DOB，也可以额外统计含扰动补偿的反馈输入：

$$
\delta\tau=K\xi+\hat d
$$

### 8.5 Disturbance Estimation Error

若仿真中知道真实等效扰动 $d(t)$，可以计算：

$$
e_d(t)=d(t)-\hat d(t)
$$

并记录

$$
\mathrm{RMSE}_d=
\sqrt{
\frac{1}{N}\sum_{k=1}^{N}
\|e_d(k)\|^2
}
$$

该指标用于展示 DOB 的估计效果。

## 9. Expected Results

预期结论如下：

- 在名义工况下，Pure LQR 和 LQR + DOB 性能接近，DOB 估计扰动接近零；
- 在负载变化、摩擦变化和外部阶跃扰动下，LQR + DOB 的峰值误差和恢复时间应小于 Pure LQR；
- LQR + DOB 可能需要更大的控制输入，这是扰动补偿带来的代价；
- DOB 对慢变和分段常值扰动效果较好，对高频噪声需要谨慎设计观测器带宽；
- 本方案避免了微分博弈和 GARE，理论复杂度更适合课程 project，但仍能体现鲁棒控制思想。

最终报告和 slides 中建议展示：

- 参考轨迹与实际轨迹对比；
- $x_e,y_e,\theta_e$ 随时间变化；
- $v_e,\omega_e$ 随时间变化；
- 控制输入 $\tau_v,\tau_\omega$；
- DOB 估计扰动 $\hat d$ 与真实等效扰动的对比；
- 三种工况下 Pure LQR 与 LQR + DOB 的指标表格。

## 10. Project Feasibility

本方案符合课程 project 要求：

1. **Introduction**：移动机器人轨迹跟踪具有明确工程背景。
2. **Literature review**：可讨论运动学控制、LQR、鲁棒控制和 DOB。
3. **Methodology**：建立非线性移动机器人模型，线性化误差动力学，设计 LQR，并叠加 DOB 增强鲁棒性。
4. **Stability analysis**：名义 LQR 通过 Riccati 方程保证稳定；DOB 通过观测器误差系统 Hurwitz 保证扰动估计收敛；整体闭环可解释为稳定 LQR 系统受估计误差输入驱动。
5. **Case study/simulation**：可在 Python 中对非线性机器人模型进行仿真，并比较 Pure LQR 与 LQR + DOB 在实际扰动下的性能。
6. **References**：可引用移动机器人轨迹跟踪、LQR、DOB 和鲁棒控制相关文献。

相比 LQR/$H_\infty$ 或微分博弈方案，本方案更容易完成，推导更清楚，仿真风险更低，同时仍然属于鲁棒控制应用。

## 11. Code Implementation

### 11.1 Code Architecture

本项目代码采用 **YAML 配置 + Python 模块 + 实验脚本** 的结构，不使用 CLI。所有实验都可以通过 `experiments/` 目录下的脚本复现。

```text
Robust-Control-Course-Project/
├── configs/
│   ├── default.yaml
│   ├── nominal.yaml
│   ├── step_disturbance.yaml
│   ├── piecewise_disturbance.yaml
│   ├── payload_change.yaml
│   └── noise_disturbance.yaml
├── src/
│   └── robust_robot/
│       ├── config.py
│       ├── dynamics.py
│       ├── lqr.py
│       ├── dob.py
│       ├── controllers.py
│       ├── scenarios.py
│       ├── simulation.py
│       ├── analysis.py
│       └── plotting.py
├── experiments/
│   ├── run_nominal.py
│   ├── run_step_disturbance.py
│   ├── run_piecewise_disturbance.py
│   ├── run_payload_change.py
│   ├── run_noise_disturbance.py
│   ├── run_dob_sweep.py
│   └── run_all.py
└── results/
    ├── raw/
    ├── figures/
    └── tables/
```

各模块功能如下：

| 文件 | 功能 |
|---|---|
| `config.py` | 读取和合并 YAML 配置，创建输出目录，保存最终配置 |
| `dynamics.py` | 圆轨迹、Kanayama 误差、线性化模型、非线性机器人动力学 |
| `lqr.py` | LQR Riccati 方程求解、可控性检查、闭环极点计算 |
| `dob.py` | 增广扰动观测器模型、可观性检查、观测器极点配置 |
| `controllers.py` | Pure LQR 与 LQR + DOB 的实际力矩输入组合 |
| `scenarios.py` | nominal、阶跃扰动、分段常值扰动、载荷变化、噪声扰动 |
| `simulation.py` | 调用 `solve_ivp` 进行非线性闭环仿真 |
| `analysis.py` | RMSE、ITAE、峰值误差、控制能量、扰动估计误差 |
| `plotting.py` | 轨迹图、误差图、控制输入图、扰动估计图、指标对比图 |

### 11.2 Symbol and Variable Mapping

代码变量命名尽量与本文数学符号保持一致：

| 数学符号 | 代码变量 | 说明 |
|---|---|---|
| $q=[x,y,\theta]^T$ | `state[:3]` | 机器人位姿 |
| $\eta=[v,\omega]^T$ | `state[3:5]` | 机器人速度 |
| $q_r$ | `q_r` | 参考位姿 |
| $\eta_r$ | `eta_r` | 参考速度 |
| $\xi$ | `xi` | 跟踪误差状态 |
| $A,B$ | `A`, `B` | 线性化误差模型矩阵 |
| $Q,R$ | `Q`, `R` | LQR 权重矩阵 |
| $P$ | `P` | Riccati 方程解 |
| $K$ | `K` | LQR 状态反馈增益 |
| $A_K=A-BK$ | `A_K` | LQR 闭环矩阵 |
| $M,D$ | `M`, `D` | 实际惯性和阻尼矩阵 |
| $M_0,D_0$ | `M0`, `D0` | 名义惯性和阻尼矩阵 |
| $\tau$ | `tau` | 实际驱动力和力矩 |
| $\tau_{ff}$ | `tau_ff` | 名义前馈输入 |
| $\delta\tau$ | `delta_tau` | 反馈修正力矩 |
| $\tau_d$ | `tau_d` | 外部物理扰动 |
| $d=[d_1,d_2]^T$ | `d` | 输入通道等效集总扰动 |
| $\hat d$ | `d_hat` | DOB 估计扰动 |
| $\tilde d=d-\hat d$ | `d_tilde` | 扰动估计误差 |
| $x_a=[\xi,d]^T$ | `x_a` | 增广状态 |
| $\hat{x}_a=[\hat{\xi},\hat d]^T$ | `x_a_hat` | 增广观测器状态 |
| $A_a,B_a,C_a$ | `A_a`, `B_a`, `C_a` | 增广观测器矩阵 |
| $L$ | `L` | DOB 观测器增益 |

需要特别注意：代码中保留了误差模型虚拟输入 `u` 与实际机器人输入 `tau` 的区别。

Pure LQR 中：

$$
u=-K\xi,\quad
\tau=\tau_{ff}+K\xi
$$

LQR + DOB 中：

$$
u=-K\xi-\hat d,\quad
\tau=\tau_{ff}+K\xi+\hat d
$$

### 11.3 How to Run Experiments

依赖安装：

```bash
pip install -r requirements.txt
```

若使用已有 conda 环境 `pyrobot`，可以直接运行：

```bash
conda run -n pyrobot python experiments/run_all.py
```

单独运行不同工况：

```bash
conda run -n pyrobot python experiments/run_nominal.py
conda run -n pyrobot python experiments/run_step_disturbance.py
conda run -n pyrobot python experiments/run_piecewise_disturbance.py
conda run -n pyrobot python experiments/run_payload_change.py
conda run -n pyrobot python experiments/run_noise_disturbance.py
```

运行 DOB 带宽扫描实验：

```bash
conda run -n pyrobot python experiments/run_dob_sweep.py
```

### 11.4 Output Files

每次运行会在 `results/` 下保存结果：

```text
results/
├── raw/
│   └── scenario_controller/
│       ├── config.yaml
│       ├── system_checks.json
│       ├── raw_data.npz
│       ├── trajectory.png
│       ├── errors.png
│       ├── velocity_errors.png
│       ├── controls.png
│       └── disturbance_estimate.png
├── tables/
│   ├── summary_metrics.csv
│   └── dob_sweep_metrics.csv
└── figures/
    ├── summary_metrics.png
    └── dob_sweep.png
```

其中 `system_checks.json` 保存可控性、可观性、LQR 闭环极点和 DOB 观测器极点；`summary_metrics.csv` 保存 Pure LQR 与 LQR + DOB 在所有工况下的指标对比。

### 11.5 Reproducibility

所有工况参数都由 `configs/*.yaml` 给出。每个实验运行时会把合并后的配置保存到对应结果目录下的 `config.yaml`，因此可以根据保存配置复现实验结果。噪声扰动使用固定随机种子 `simulation.seed` 和采样保持方式生成，避免连续白噪声直接进入 ODE 导致不可复现。

## 12. References

[1] Y. Kanayama, Y. Kimura, F. Miyazaki, and T. Noguchi, "A stable tracking control method for an autonomous mobile robot," *Proceedings of IEEE International Conference on Robotics and Automation*, 1990.

[2] B. D. O. Anderson and J. B. Moore, *Optimal Control: Linear Quadratic Methods*. Prentice Hall, 1990.

[3] K. Ogata, *Modern Control Engineering*. Prentice Hall, 2010.

[4] K. Ohnishi, M. Shibata, and T. Murakami, "Motion control for advanced mechatronics," *IEEE/ASME Transactions on Mechatronics*, 1996.

[5] W.-H. Chen, J. Yang, L. Guo, and S. Li, "Disturbance-observer-based control and related methods: An overview," *IEEE Transactions on Industrial Electronics*, 2016.

[6] H. K. Khalil, *Nonlinear Systems*. Prentice Hall, 2002.

[7] R. Siegwart, I. R. Nourbakhsh, and D. Scaramuzza, *Introduction to Autonomous Mobile Robots*. MIT Press, 2011.

# KF
$$
\begin{align}
\hat x^-_k &= Fx_{k-1} + Gu_k + w_k  \\
\hat{P}^-_k &= FP_{k-1}F^T + Q \\
\hat x &= \hat x_k + K_k(z_k - H\hat x_k) \\
P_k &= (I - K_k H)\hat P_k^-(I - K_k H)^T +K_kRK_k^T \\
&= (I- K_kH)\hat P_k^- \\
K_k&= \hat P_k^-H^T(H\hat P_k^-H^T + R)^{-1}

\end{align}
$$

# EKF


$$
\begin{align}
[q]_L = \left[ \begin{array}{ccc} &s & -v^T \\
&v & sI +[v]_\times\end{array} \right] \\
[q]_R = \left[ \begin{array}{ccc} &s & -v^T \\
&v & sI -[v]_\times\end{array} \right]\\
\hat q_k = q_{k-1} \otimes \Delta q\\
\end{align}
$$
$$
\begin{align}
\therefore \frac{\partial q_{k}}{\partial q_{k-1}} &= [\Delta q]_R = M_1 \\
\\
\therefore \frac {\partial {q_k}}{\partial bias_{k-1}}& = [q_{k-1}]_L \left[ \begin{array}{ccc} 0 & 0& 0 \\
1 & 0 & 0 \\
0 & 1& 0 \\
0 & 0 & 1\end{array}\right]* (-\frac{dt}{2}) = M_2 \\
\frac{\partial q_k}{\partial bias_{k-1}} &= \left[ \begin{array}{ccccccc} 0 & 0& 0&0&1&0&0 \\
0 & 0& 0&0&0 & 1& 0 \\
0 & 0& 0&0&0 & 0& 1 \end{array}\right] = [0 \ I] \\
F&= \left[ \begin{array}{} M_1 & M_2 \\
0& I\end{array}\right], 7\times7
\end{align}
$$
$$
\begin{align}
\hat q_k &= [s, x, y, z]^T \\
z &= \begin{bmatrix} 2xz + 2sy \\
2yz - 2sx \\
s^2 -x^2 -y^2 + z^2
\end{bmatrix} \\
H&= 2\begin{bmatrix} -y & z & -s & x & 0 & 0 & 0 \\
x & s & z & y & 0 & 0 & 0  \\
s & -x & -y & z & 0 & 0 & 0 
\end{bmatrix}
\end{align}
$$$$
\begin{align}
K &= \hat P H (H \hat P h^T + R)^{-1} \\
x  &= \hat x + K (z - h(\hat x)) = \hat x + K(z - R(q)^Tg_w) \\
P &= (I - KH)\hat P
\end{align}
$$

# ESKF
A
a
a
a
a

a
a
a

a
$$\begin{align}
\delta \theta_k &= R(\Delta \theta)^T \delta \theta_{k-1} - dt b_{k-1} + w_t\\
F&= \begin{bmatrix} 
R(\Delta \theta)^T &-dt I_{3\times3 }\\
0 &I_{3\times3}
\end{bmatrix}\end{align}
$$

$$
\begin{align}
z &= R(q_{true})^T g_w \\
& = (R(q_{nom})(I + [\delta \theta]_\times))^T g_w \\
& = R(q_{nom})^Tg_w - [\delta \theta]_\times R(q_{nom})^T g_w \\
&=R(q_{nom})^Tg_w + [R(q_{nom})g_w^T]_\times\delta \theta\\

H &= [R(q_{nom})g_w^T], 0_{3\times3}] \\
\end{align}
$$

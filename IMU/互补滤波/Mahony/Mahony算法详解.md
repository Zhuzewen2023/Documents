# Mahony算法
Mahony算法是Madgwick算法的工程简化版本，两者相比，Mahony算法消耗更小的计算资源，而Madgwick算法的融合误差更小一些。Mahony算法利用航姿反馈、加速度和磁力计算系统反馈总误差，通过PI（Proportional Integral，比例积分）控制的方式，利用反馈总误差修正角速度输入值，最后利用修正后的角速度推算出航姿，如图4-34所示。

![1771572961002](image/Mahony算法/1771572961002.png)

# PI控制
## 什么叫 PI 控制步骤（用人话 + 标准离散流程）

PI 控制的核心一句话：

> **P（比例项）负责“立刻纠错”，I（积分项）负责“记住长期偏差并消掉”。**

---

### 1) P 和 I 各自干什么
- **P（Proportional）比例项**：误差越大，纠正动作越大（立即响应）
  $$
  u_P = K_P\,e
  $$

- **I（Integral）积分项**：把误差随时间累积，用来消除“长期不为 0 的误差”（常值偏差）
  $$
  I \leftarrow I + e\,\Delta t,\qquad u_I = K_I\,I
  $$

- **总输出**
  $$
  u = u_P + u_I
  $$

---

### 2) 标准 PI 控制“步骤”（离散实现：每帧/每个采样周期都跑一遍）
设：
- 目标/参考值：$r_t$
- 系统输出/测量值：$y_t$
- 误差：$e_t = r_t - y_t$
- 采样周期：$\Delta t$

每一帧的步骤：

1. **获取测量/状态**：得到 $y_t$
2. **计算误差**
   $$
   e_t = r_t - y_t
   $$
3. **比例项（P）立即响应**
   $$
   u_{P,t} = K_P\,e_t
   $$
4. **积分器更新（I 的“记账本”）**
   $$
   I_t = I_{t-1} + e_t\,\Delta t
   $$
   工程上常加限幅避免积分跑飞：
   $$
   I_t \leftarrow \text{clamp}(I_t)
   $$
5. **积分项输出**
   $$
   u_{I,t} = K_I\,I_t
   $$
6. **合成控制输出**
   $$
   u_t = u_{P,t} + u_{I,t}
   $$
7. **把 $u_t$ 施加到系统**（用于下一次更新/控制）

这就是“PI 控制步骤”。

---

### 3) 把 PI 映射到 Madgwick（为什么说它像 PI）
Madgwick 的“误差”来自姿态约束（重力/地磁对齐误差）。

#### P 项：立刻纠姿态（快）
纠偏方向来自梯度：
$$
\hat{\dot q}_{e,t}=\frac{\nabla f}{\|\nabla f\|}
$$
用力度系数 $\beta$ 立刻拉回（相当于 P 增益）：
$$
\dot q = \dot q_\omega - \beta\,\hat{\dot q}_{e,t}
$$

#### I 项：慢慢学陀螺 bias（消长期漂移）
把“长期纠偏需求”积分起来当作 bias 估计（相当于 I 增益）：
$$
\omega_{b,t}=\omega_{b,t-1}+\zeta\,\omega_{e,t}\Delta t
$$
再从陀螺里扣掉：
$$
\omega_{c,t} = \omega_t - \omega_{b,t}
$$

---

### 4) 一个直觉类比（帮助记住 P vs I）
- **P**：车偏离车道，立刻打方向盘回去（快，但可能抖）
- **I**：发现方向盘总得往右拧才能直行 → 说明车轮定位偏了 → 把这个“长期需要拧的量”记下来做校准（慢，但能消掉长期偏差）

# 直觉理解 PI：一个“纠偏 + 记账”的人

把系统想成一辆会受风吹的车（或一个会漂的陀螺积分姿态），你要让它一直走在中线（误差为 0）。

---

## 1) P：立刻纠偏（像你看见偏了就打方向盘）
- 你偏离中线多少，就立刻打方向盘多少。
- 偏得越多，打得越猛；偏得越少，打得越轻。

人话：
> **现在偏了多少，我现在就纠多少。**

优点：反应快。  
缺点：如果一直有“恒定外力”（比如一直侧风、一直零偏），你即使一直在纠，也可能会留下一个“永远偏一点点”的稳态误差。

---

## 2) I：长期记账（像你发现方向盘总得歪着打，干脆把它记下来当“校准量”）
想象你开车发现一个现象：

> 就算你已经回到中线，你的方向盘也必须一直往右打一点点才能走直。

这说明什么？
- 不是你手抖
- 是有个“长期偏置”（侧风、车轮定位偏、路面坡度）

于是你做的“聪明事”是：
- 把“我长期需要往右打的那一点点”记下来
- 以后不用每次都靠瞬时纠偏硬撑，让系统自带一个“抵消偏置”的常量

人话：
> **如果我总是在同一边纠偏，那肯定不是随机噪声，是系统有偏置；我把这偏置学出来。**

这就是积分项：把误差长期累积起来，专门用来吃掉“常值偏置”，让稳态误差逼近 0。

---

## 3) 为什么要 P + I，而不是只用一个？
### 只有 P（只会立刻纠偏）：
- 偏了能拉回来，但面对恒定偏置时，往往会“永远差一点点”
- 你需要一直用一个非零误差来“提供纠偏力”

直觉：
> **靠误差当燃料**，误差不为 0 才有输出去顶住偏置。

### 加上 I（会记账）：
- I 会不断累积，直到“顶住偏置”
- 一旦偏置被顶住，误差就可以回到 0
- 误差为 0 后，积分器也不再增长（它停在那个刚好抵消偏置的常量上）

直觉：
> **把“长期需要的纠偏量”固化成常量补偿**，让误差可以回到 0。

---

## 4) 一个特别直观的类比：P 是“手动纠偏”，I 是“自动校准”
- **P**：你开车时实时修正方向  
- **I**：你发现车子天生跑偏，于是把方向盘“校准”一点，让它自己走直

---

## 5) 映射到 Madgwick（你会更有感觉）
- **P 项**（\(\beta\) 那条）：加计/磁计看到姿态不对，立刻把姿态拉回（快速纠偏）
- **I 项**（\(\zeta\) 那条）：如果系统长期都在同方向纠偏，说明陀螺有 bias，于是把这种“长期纠偏需求”积起来当作 bias，直接从陀螺里扣掉（慢速校准）

---

## 6) 一句话结论
> **P：现在错了就马上纠。  
> I：总是往同一边错，就把这“长期偏”学出来，直接抵消掉。**

如果你愿意，我可以用“侧风车”的 10 秒数值例子演示：只用 P 会留稳态误差，加上 I 后误差被逼近 0，而且 I 会停在一个常数上（你会直观看到它为什么不会无限长）。

# Mahony算法实现步骤解析

系统的输入为磁力计测量值 $[m_x,m_y,m_z]^T$、加速度计测量值 $[a_x,a_y,a_z]^T$ 和陀螺仪测量值 $[\omega_x,\omega_y,\omega_z]^T$，系统的输出是融合得到的航姿 ${}^{S}_{E}\mathbf{q}_t=[q_0,(q_1,q_2,q_3)]^T$。

我们需要先对输入的磁力计测量值和加速度计测量值进行归一化，如式（4-94）和式（4-95）所示。

式 (4-94) 与式 (4-95) 是对磁力计向量 $\mathbf m=[m_x,m_y,m_z]^T$ 和加速度计向量 $\mathbf a=[a_x,a_y,a_z]^T$ 做 **归一化**（单位化）：

## 磁力计归一化（4-94）
$$
{}^{S}\mathbf m
=\frac{\mathbf m}{\|\mathbf m\|}
=
\begin{bmatrix}
\dfrac{m_x}{\sqrt{m_x^2+m_y^2+m_z^2}}\\[6pt]
\dfrac{m_y}{\sqrt{m_x^2+m_y^2+m_z^2}}\\[6pt]
\dfrac{m_z}{\sqrt{m_x^2+m_y^2+m_z^2}}
\end{bmatrix}
\qquad (4\text{-}94)
$$

## 加速度计归一化（4-95）
$$
{}^{S}\mathbf a
=\frac{\mathbf a}{\|\mathbf a\|}
=
\begin{bmatrix}
\dfrac{a_x}{\sqrt{a_x^2+a_y^2+a_z^2}}\\[6pt]
\dfrac{a_y}{\sqrt{a_x^2+a_y^2+a_z^2}}\\[6pt]
\dfrac{a_z}{\sqrt{a_x^2+a_y^2+a_z^2}}
\end{bmatrix}
\qquad (4\text{-}95)
$$

利用反馈回来的航姿信息 ${}^{S}_{E}q_{t-1}$，将归一化磁力计测量量 ${}^{S}m$ 从传感器坐标系 $S$ 反投影为世界坐标系 $E$ 中的 ${}^{E}\hat h$，如式（4-96）所示，并将反投影得到的磁力中的 $x$ 轴和 $y$ 轴的分量合成到 $x$ 轴上，最后就得到地磁先验信息 ${}^{E}\hat b$，如式（4-97）所示。这就是上面 Madgwick 算法中提到的磁力修正过程，用实时预估的地磁值来替换固定的地磁值，这里不再赘述。进一步，用航姿信息 ${}^{S}_{E}q_{t-1}$，将先验地磁信息 ${}^{E}\hat b$ 从世界坐标系 $E$ 投影为传感器坐标系 $S$ 中的 ${}^{S}\hat b$，如式（4-98）所示。

---

# 公式

## (4-96)
$$
{}^{E}\hat{\mathbf h}
=
\begin{bmatrix}
h_x\\
h_y\\
h_z
\end{bmatrix}
=
\begin{bmatrix}
{}^{S}m_x(1-2q_2^2-2q_3^2)+{}^{S}m_y(2q_1q_2-2q_0q_3)+{}^{S}m_z(2q_1q_3+2q_0q_2)\\
{}^{S}m_x(2q_1q_2+2q_0q_3)+{}^{S}m_y(1-2q_1^2-2q_3^2)+{}^{S}m_z(2q_2q_3-2q_0q_1)\\
{}^{S}m_x(2q_1q_3-2q_0q_2)+{}^{S}m_y(2q_2q_3+2q_0q_1)+{}^{S}m_z(1-2q_1^2-2q_2^2)
\end{bmatrix}
\qquad (4\text{-}96)
$$

## (4-97)
$$
{}^{E}\hat{\mathbf b}
=
\begin{bmatrix}
b_x\\
b_y\\
b_z
\end{bmatrix}
=
\begin{bmatrix}
\sqrt{h_x^2+h_y^2}\\
0\\
h_z
\end{bmatrix}
\qquad (4\text{-}97)
$$

## (4-98)
$$
{}^{S}\hat{\mathbf b}
=
\begin{bmatrix}
b_x(1-2q_2^2-2q_3^2)+b_z(2q_1q_3-2q_0q_2)\\
b_x(2q_1q_2-2q_0q_3)+b_z(2q_0q_1+2q_2q_3)\\
b_x(2q_0q_2+2q_1q_3)+b_z(1-2q_1^2-2q_2^2)
\end{bmatrix}
\qquad (4\text{-}98)
$$

---

## 一句人话总结
先用上一帧姿态 ${}^{S}_{E}q_{t-1}$ 把磁力计测量 ${}^{S}m$ “旋到”世界系得到 ${}^{E}\hat h$，再把水平分量合并成 $b_x=\sqrt{h_x^2+h_y^2}$ 并令 $b_y=0$ 得到世界系地磁先验 ${}^{E}\hat b$，最后再把 ${}^{E}\hat b$ “旋回”传感器系得到 ${}^{S}\hat b$ 供后续误差函数使用。

---

# (4-96)~(4-98) 在做什么（地磁修正）

一句话：用上一帧姿态 $\,{}^{S}_{E}\hat q_{t-1}\,$ 把磁力计测量从传感器系 $S$ “旋到”世界系 $E$，得到 $\,{}^{E}\hat{\mathbf h}\,$；再把水平分量合并成 $b_x$ 并强制 $b_y=0$ 得到世界系地磁先验 $\,{}^{E}\hat{\mathbf b}\,$；最后再把 $\,{}^{E}\hat{\mathbf b}\,$ “旋回”传感器系，得到 $\,{}^{S}\hat{\mathbf b}\,$ 给后续误差函数使用。

---

## 0) 符号
- 上一帧姿态四元数：$\,{}^{S}_{E}\hat q_{t-1}=[q_0,q_1,q_2,q_3]^T$
- 归一化磁力计测量（$S$系）：$\,{}^{S}\hat{\mathbf m}=[m_x,m_y,m_z]^T$
- 反投影到世界系的磁场：$\,{}^{E}\hat{\mathbf h}=[h_x,h_y,h_z]^T$
- 世界系地磁先验：$\,{}^{E}\hat{\mathbf b}=[b_x,b_y,b_z]^T$
- 旋转矩阵：$R(q)$（由四元数 $q$ 构造）
- $q^*$：四元数共轭

---

## 1) 式 (4-96)：把磁测量从 $S$ 旋到 $E$
图里那一长串展开式，本质等价于矩阵写法：

$$
{}^{E}\hat{\mathbf h}=R\!\left({}^{S}_{E}\hat q_{t-1}\right)\,{}^{S}\hat{\mathbf m}
\qquad (4\text{-}96)
$$

等价的四元数“夹心旋转”写法是（把向量当作纯虚四元数 $[0,\mathbf v]$）：

$$
[0,{}^{E}\hat{\mathbf h}]
=
{}^{S}_{E}\hat q_{t-1}\ \otimes\ [0,{}^{S}\hat{\mathbf m}]\ \otimes\ \left({}^{S}_{E}\hat q_{t-1}\right)^*
$$

直觉：用上一帧姿态把 $S$ 系里测到的磁场方向搬到 $E$ 系里。

---

## 2) 式 (4-97)：规整地磁先验（合并水平分量并令 $b_y=0$）
$$
{}^{E}\hat{\mathbf b}
=
\begin{bmatrix}
b_x\\ b_y\\ b_z
\end{bmatrix}
=
\begin{bmatrix}
\sqrt{h_x^2+h_y^2}\\
0\\
h_z
\end{bmatrix}
\qquad (4\text{-}97)
$$

解释（人话）：
- $\sqrt{h_x^2+h_y^2}$ 是磁场在水平面的幅值（把 $x,y$ 两个水平分量合成一个“水平强度”）
- 令 $b_y=0$ 是一种坐标系选择（gauge fixing）：你可以绕世界系 $z$ 轴转一下坐标，使地磁水平投影落在 $x$ 轴方向上，从而 $y$ 分量为 0（不是说物理上没有 $y$ 分量）。

---

## 3) 式 (4-98)：把世界系地磁先验再旋回传感器系
矩阵形式：
$$
{}^{S}\hat{\mathbf b}
=
R\!\left({}^{S}_{E}\hat q_{t-1}\right)^T\,{}^{E}\hat{\mathbf b}
\qquad (4\text{-}98)
$$

等价的四元数形式：
$$
[0,{}^{S}\hat{\mathbf b}]
=
\left({}^{S}_{E}\hat q_{t-1}\right)^*\ \otimes\ [0,{}^{E}\hat{\mathbf b}]\ \otimes\ {}^{S}_{E}\hat q_{t-1}
$$

直觉：后面误差函数通常在传感器系里对比“预测 vs 测量”，所以把先验 $\hat{\mathbf b}$ 搬回 $S$ 系方便计算。

---

## 4) 一条流水线总结
1. 读磁：$\,{}^{S}\hat{\mathbf m}$
2. 旋到 $E$：$\,{}^{E}\hat{\mathbf h}=R(q_{t-1})\,{}^{S}\hat{\mathbf m}$
3. 规整先验：$\,{}^{E}\hat{\mathbf b}=[\sqrt{h_x^2+h_y^2},0,h_z]^T$
4. 旋回 $S$：$\,{}^{S}\hat{\mathbf b}=R(q_{t-1})^T\,{}^{E}\hat{\mathbf b}$

将从传感器直接测量得到的 ${}^{S}\mathbf m$ 与上面计算出的 ${}^{S}\hat{\mathbf b}$ 做叉乘，就得到磁力计提供的误差反馈 ${}^{S}\mathbf e_m$，如式（4-99）所示。

$$
{}^{S}\mathbf e_m
=
{}^{S}\mathbf m \times {}^{S}\hat{\mathbf b}
=
\begin{bmatrix}
{}^{S}m_y\,{}^{S}\hat b_z - {}^{S}m_z\,{}^{S}\hat b_y\\
{}^{S}m_z\,{}^{S}\hat b_x - {}^{S}m_x\,{}^{S}\hat b_z\\
{}^{S}m_x\,{}^{S}\hat b_y - {}^{S}m_y\,{}^{S}\hat b_x
\end{bmatrix}
\qquad (4\text{-}99)
$$

同样的方式，计算加速度计提供的误差反馈 ${}^{S}\mathbf e_a$。由于先验重力信息很稳定，所以不需要像上面先验地磁信息那样进行修正，直接选用常数值就行了，归一化重力加速度如式（4-100）所示。

$$
{}^{E}\hat{\mathbf g}
=
\begin{bmatrix}
0\\
0\\
1
\end{bmatrix}
\qquad (4\text{-}100)
$$

用航姿信息 ${}^{S}_{E}\mathbf q_{t-1}$ 将重力先验信息 ${}^{E}\hat{\mathbf g}$ 从世界坐标系 $E$ 投影为传感器坐标系 $S$ 中的 ${}^{S}\hat{\mathbf g}$，如式（4-101）所示。

$$
{}^{S}\hat{\mathbf g}
=
\begin{bmatrix}
2q_1q_3 - 2q_0q_2\\
2q_0q_1 + 2q_2q_3\\
1 - 2q_1^2 - 2q_2^2
\end{bmatrix}
\qquad (4\text{-}101)
$$

将从传感器直接测量得到的 ${}^{S}\mathbf a$ 与上面计算出的 ${}^{S}\hat{\mathbf g}$ 做叉乘，就是加速度计提供的误差反馈 ${}^{S}\mathbf e_a$，如式（4-102）所示。

$$
{}^{S}\mathbf e_a
=
{}^{S}\mathbf a \times {}^{S}\hat{\mathbf g}
=
\begin{bmatrix}
{}^{S}a_y\,{}^{S}\hat g_z - {}^{S}a_z\,{}^{S}\hat g_y\\
{}^{S}a_z\,{}^{S}\hat g_x - {}^{S}a_x\,{}^{S}\hat g_z\\
{}^{S}a_x\,{}^{S}\hat g_y - {}^{S}a_y\,{}^{S}\hat g_x
\end{bmatrix}
\qquad (4\text{-}102)
$$

将 ${}^{S}\mathbf e_m$ 和 ${}^{S}\mathbf e_a$ 加起来就是总误差反馈 ${}^{S}\mathbf e$，如式（4-103）所示。同时通过积分计算 ${}^{S}\mathbf e$ 的积分值，如式（4-104）所示。

$$
{}^{S}\mathbf e
=
{}^{S}\mathbf e_m + {}^{S}\mathbf e_a
\qquad (4\text{-}103)
$$

$$
\Sigma({}^{S}\mathbf e)
=
\Sigma({}^{S}\mathbf e) + K_i \cdot {}^{S}\mathbf e \cdot \Delta t
\qquad (4\text{-}104)
$$

然后通过 PI 控制的方式，利用误差反馈修正角速度输入值，如式（4-105）所示。

$$
{}^{S}\boldsymbol{\omega}_c
=
{}^{S}\boldsymbol{\omega}
+
K_p \cdot {}^{S}\mathbf e
+
\Sigma({}^{S}\mathbf e)
\qquad (4\text{-}105)
$$

接着，利用前一时刻的航姿 ${}^{S}_{E}\mathbf q_{t-1}$ 和修正后的角速度 ${}^{S}\boldsymbol{\omega}_c$ 推导当前时刻航姿 ${}^{S}_{E}\mathbf q_t$，如式（4-106）所示。

$$
\begin{bmatrix}
q_0\\
q_1\\
q_2\\
q_3
\end{bmatrix}
=
\begin{bmatrix}
q_0\\
q_1\\
q_2\\
q_3
\end{bmatrix}
+
\frac{1}{2}
\begin{bmatrix}
- q_1\cdot \omega_{c,x} - q_2\cdot \omega_{c,y} - q_3\cdot \omega_{c,z}\\
\ \ q_0\cdot \omega_{c,x} + q_2\cdot \omega_{c,z} - q_3\cdot \omega_{c,y}\\
\ \ q_0\cdot \omega_{c,y} - q_1\cdot \omega_{c,z} + q_3\cdot \omega_{c,x}\\
\ \ q_0\cdot \omega_{c,z} + q_1\cdot \omega_{c,y} - q_2\cdot \omega_{c,x}
\end{bmatrix}
\Delta t
\qquad (4\text{-}106)
$$

最后，将得到的航姿四元数归一化，就可以作为融合结果输出了，如式（4-107）所示。

$$
{}^{S}_{E}\mathbf q_t
=
\frac{\mathbf q}{\|\mathbf q\|}
=
\begin{bmatrix}
\dfrac{q_0}{\sqrt{q_0^2+q_1^2+q_2^2+q_3^2}}\\[6pt]
\dfrac{q_1}{\sqrt{q_0^2+q_1^2+q_2^2+q_3^2}}\\[6pt]
\dfrac{q_2}{\sqrt{q_0^2+q_1^2+q_2^2+q_3^2}}\\[6pt]
\dfrac{q_3}{\sqrt{q_0^2+q_1^2+q_2^2+q_3^2}}
\end{bmatrix}
\qquad (4\text{-}107)
$$


# 1) (4-101) 在算什么：把“世界系重力方向”变到传感器系
世界系里，重力先验定义成一个单位向量：

$$
{}^{E}\hat{\mathbf g}=\begin{bmatrix}0\\0\\1\end{bmatrix}
$$

你当前有上一帧姿态四元数 ${}^{S}_{E}\mathbf q_{t-1}$，它描述了 **世界系 $E$ 到传感器系 $S$ 的旋转**（按你书的符号）。

所以把世界系的重力方向“投影到”传感器系，就是：

- “如果姿态是 $q$，那在传感器坐标里，重力应该朝哪儿？”

式 (4-101) 给了展开结果：

$$
{}^{S}\hat{\mathbf g}
=
\begin{bmatrix}
2q_1q_3 - 2q_0q_2\\
2q_0q_1 + 2q_2q_3\\
1 - 2q_1^2 - 2q_2^2
\end{bmatrix}
\qquad (4\text{-}101)
$$

**直觉：**  
这就是“用姿态预测出来的重力方向”（预测值）。

---

# 2) (4-102) 在算什么：用叉乘得到“重力方向的误差轴”
加速度计测到（归一化后）：

$$
{}^{S}\mathbf a
$$

在静止或低线加速度时，${}^{S}\mathbf a$ 应该和 ${}^{S}\hat{\mathbf g}$ 指向一致（同向或反向取决于你对加速度计符号定义，但这里默认一致）。

Mahony 用叉乘做误差：

$$
{}^{S}\mathbf e_a
=
{}^{S}\mathbf a \times {}^{S}\hat{\mathbf g}
\qquad (4\text{-}102)
$$

叉乘的性质很关键：

- 结果向量 **垂直于** 两个向量所在平面
- 方向遵循右手定则
- 模长是：
  $$
  \|e_a\|=\|a\|\,\|\hat g\|\sin\theta = \sin\theta
  $$
  因为都归一化了

也就是说：

> ${}^{S}\mathbf e_a$ 给你一个“旋转轴 + 角度大小（小角度时≈角度）”，告诉你要绕哪个轴转，才能把预测重力对齐到测量重力。

**人话：**  
- $a$ 和 $\hat g$ 越对齐，夹角 $\theta$ 越小 → 误差越小  
- 误差向量的方向就是“该往哪边拧”才能对齐

---

# 3) (4-103) 把磁和重力误差加起来：总姿态误差
磁力计同理会产生一个误差向量 ${}^{S}\mathbf e_m$（也是叉乘形式）。

然后直接相加：

$$
{}^{S}\mathbf e
=
{}^{S}\mathbf e_m + {}^{S}\mathbf e_a
\qquad (4\text{-}103)
$$

**直觉：**  
你把“重力告诉你的纠偏方向”和“地磁告诉你的纠偏方向”加权合成（这里权重隐含在传感器归一化、以及后面的 $K_p,K_i$ 里；更工程化的做法通常还会给磁、加计单独权重）。

---

# 4) (4-104) 积分：把“长期不为 0 的误差”累积起来（I 项）
$$
\Sigma({}^{S}\mathbf e)
=
\Sigma({}^{S}\mathbf e) + K_i \cdot {}^{S}\mathbf e \cdot \Delta t
\qquad (4\text{-}104)
$$

这就是标准 I 项积分器。

**直觉：为什么要积分？**
- 如果陀螺有零偏，系统会长期朝某个方向漂
- 那么你会发现 ${}^{S}\mathbf e$ **长期带同号均值**
- 积分器会把这个“长期偏差”累积成一个慢变化的补偿量
- 本质上：**I 项在估计/补偿陀螺 bias**

---

# 5) (4-105) PI 修正角速度：把误差变成“该加在陀螺上的角速度”
$$
{}^{S}\boldsymbol{\omega}_c
=
{}^{S}\boldsymbol{\omega}
+
K_p \cdot {}^{S}\mathbf e
+
\Sigma({}^{S}\mathbf e)
\qquad (4\text{-}105)
$$

这里的关键点是：  
${}^{S}\mathbf e$ 的单位/物理意义是什么？

- 因为 ${}^{S}\mathbf e$ 是由叉乘构造的（小角度时近似角度误差）
- 乘上 $K_p$（单位约等于 $1/s$）后，就变成一个**角速度修正量**
- 积分项 $\Sigma({}^{S}\mathbf e)$ 也相当于一个慢变化的角速度补偿（对 bias）

所以 (4-105) 的物理意义是：

> “真实角速度” ≈ “陀螺测到的角速度” + “P项纠偏角速度” + “I项长期补偿角速度”

最终用 $\omega_c$ 去积分姿态（你后面的 4-106）。

---

# 6) 你卡住的点：为什么是“叉乘误差”就能直接加到角速度上？
最直觉的说法：

- ${}^{S}\mathbf e$ 给的是“**你姿态此刻偏到哪边去了**”（偏差轴）
- 如果你希望它在未来 1 秒内回正一部分，你就需要一个角速度去拧回去
- $K_p$ 就是在规定“回正速度”：
  - $K_p$ 大：回正快（但更容易抖）
  - $K_p$ 小：回正慢（但更稳）
- I 项则是把“总要拧回去的那部分”记下来当偏置补偿

---

# 7) 一句话总结这段（101~105）
1) 用 $q_{t-1}$ 预测重力方向 ${}^{S}\hat g$  
2) 用叉乘 $a\times \hat g$ 得到“该往哪边拧”的误差轴 $e_a$  
3) 磁也做同样的误差 $e_m$，两者相加得总误差 $e$  
4) 对 $e$ 做 PI，得到一个“纠偏角速度”  
5) 把它加到陀螺上得到修正角速度 $\omega_c$，再用 $\omega_c$ 去积分姿态



到这里，Mahony 算法的计算步骤就讲完了。系统中的 $K_p$ 和 $K_i$ 是 PI 控制的参数，需要在实际运行中进行参数整定。这里只对 Mahony 算法的实现过程进行了分析，关于 Mahony 算法的数学推导证明过程不做讲解。想深入研究的朋友，可以阅读原论文 [9]。如果要研究 Mahony 算法的数学理论过程，还可以阅读 Mahony 算法源代码 [6]。




[1]参见http://www.invensense.com

[2]参见http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html

[3]参见https://bitbucket.org/alberto_pretto/imu_tk

[4]参见https://github.com/gaowenliang/imu_utils

[5]参见http://www.github.com/ccny-ros-pkg/imu_tools

[6]参见http://www.github.com/PaulStoffregen/MahonyAHRS

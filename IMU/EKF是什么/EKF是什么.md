# EKF 的一句话版本

EKF 就是在循环做两件事：

* 预测：用模型把状态往前推一步（我猜现在是什么）

* 纠正：用传感器观测把这个猜测拉回来一点（我修正一下）

> 区别于普通“直接平均”：EKF 会同时维护一个东西叫 不确定度，用它来决定“信模型多一点还是信观测多一点”。


# 用最小例子讲：估计“房间温度”

你想知道真实温度 𝑥, 但你只有一个温度计读数 𝑧，它有噪声。


## 1.1 你手里维护两样东西

* $\hat{x}$：你当前认为的温度（比如 25.0℃）

* $P$：你对这个温度的“不确定度”（比如“我可能错个 1℃”）


## 1.2 每来一次新读数，你做一次循环

### A) 预测（模型）

如果你认为温度变化很慢，可以用简单模型：

$x_k = x_{k-1} + \omega$

意思：下一秒温度≈上一秒温度，但会有一点随机变化$\omega$ 

于是你预测：

![alt text](image-1.png)

* 过程噪声

![alt text](image.png)
![alt text](image-2.png)
![alt text](image-3.png)


### B) 纠正（用温度计读数）

![alt text](image-4.png)

> 温度计读数 - 预测值 = 残差

* $P^-$: 更新前（只做了预测/时间推进后的）不确定度
* $P$: 更新后（融合了量测后的）不确定度
* $K$: 卡尔曼增益（0~1之间的权重，标量情况下）

不确定度用协方差 𝑃表示。你可以把它当成“我对自己估计值有多没把握”。

预测阶段：你只靠模型推，误差会被过程噪声推大，所以得到$P^-$（通常变大）。

更新阶段：你引入了一次观测（传感器量测），等于多了一条信息约束，信息变多 → 不确定度下降，所以得到 𝑃（会变小）。

在标量情况下，0≤K≤1（在正常噪声设定下），所以：

$(1-K) <= 1 \Rightarrow P <= P^- $

直观理解：

𝐾大：更信量测 → 修正更猛 → 不确定度降得更多

𝐾小：更信预测 → 修正很弱 → 不确定度几乎不变

# EKF扩展在哪里？
上面例子里模型是线性的（加法）。

但 IMU/姿态里模型是非线性的，比如：
![alt text](image-5.png)
这时你仍然想做同样的两步：预测 + 纠正。
问题是：**不确定度**𝑃怎么传播？
线性时用$FPF^T$，非线性时 EKF 就用在**当前点附近的局部线性化**来得到一个近似的 𝐹（雅可比）。

所以 EKF = “卡尔曼滤波 + 用雅可比做局部线性化”。
## 四元数更新$q_k = q_{k-1} \otimes exp(\frac{1}{2}\omega\Delta{t})$是啥意思
![alt text](image-6.png)
### 为什么是$\delta{q} \approx exp(\frac{1}{2}\omega\Delta{t})$
#### 轴角形式
![alt text](image-9.png)
#### 群（group）与“群元素”
你可以把“群”理解成：

有一堆东西（集合），你规定一个“把两个东西合成一个东西”的操作（比如加法、乘法、矩阵相乘、旋转叠加），
只要这操作满足 4 个条件，这套系统就叫群。

图里写的“∘”（小圆圈）就是“这个合成运算”的占位符。
它不一定是乘法，也不一定是加法——你怎么定义都行，只要满足下面 4 条。
![alt text](image-10.png)
![alt text](image-11.png)
![alt text](image-12.png)
![alt text](image-13.png)
#### 旋转李群$SO(3)$
![alt text](image-14.png)
##### 怎么理解正交矩阵
正交矩阵就是“不拉伸、不压扁、不剪切”的线性变换矩阵——只会把坐标轴转一下（或者再加个镜像翻转）。
![alt text](image-15.png)
![alt text](image-16.png)
##### 怎么理解行列式
行列式在几何上代表体积/面积缩放倍数，正负号代表有没有镜像反转
![alt text](image-17.png)
正交矩阵不改变长度和角度，所以也不改变面积/体积的大小，det只能是±1

#### 切空间
![alt text](image-18.png)

#### 李代数
![alt text](image-19.png)

切空间/李代数 = “在某个姿态附近，用 3 个数表示的微小旋转（增量）的线性世界”。

## 重力投影：$a \approx R(q)^Tg$是啥意思
![alt text](image-7.png)
### 为什么$R(q)对q非线性$
![alt text](image-8.png)

## 不确定度P怎么传播
![alt text](image-20.png)


**变换会放大/旋转误差** + **每一步又会新增噪声**

### 变换会放大/旋转误差

* 你把一个带误差的量“变换/积分”到下一步时，旧误差会被当前的变换关系 传播（甚至被放大、耦合）；
* 每一步传感器都会产生新的随机误差（噪声），所以每一步都要 再加一坨新噪声。
![alt text](image-21.png)

* 奇异值就是矩阵对某个方向“拉伸/压缩”的倍数

![alt text](image-22.png)

![alt text](image-23.png)

![alt text](image-24.png)

#### Cov协方差
![alt text](image-25.png)

#### 为什么线性变换后协方差变成$FPF^T$
![alt text](image-26.png)

### 每一步还会新增噪声
现实里不是纯变换，还有过程噪声（IMU白噪声、bias随机游走）
![alt text](image-27.png)

### 一维例子
![alt text](image-28.png)

### 真实更新状态非线性

#### 怎么理解这句话

![alt text](image-29.png)

#### $u$到底是什么？

![alt text](image-30.png)

#### 为什么说“状态更新是非线性” 

![alt text](image-31.png)

##### 四元数形式

![alt text](image-32.png)

##### 旋转矩阵形式

![alt text](image-33.png)

![alt text](image-34.png)

###### $(·)\hat{}$是什么？
把向量变成“叉乘矩阵”（反对称矩阵）
* 什么叫叉乘？
叉乘（cross product，也叫向量积）是三维空间里两个向量的运算：

给定 a、b（都是 3D 向量），叉乘得到一个新的向量：

$c = a \times b$

它有三个核心含义：


1) 方向（最重要的直观）

a×b 的方向垂直于由 𝑎和 𝑏张成的平面，方向由右手定则决定：右手四指从 a 旋到 𝑏，大拇指方向就是 𝑎×𝑏。

![alt text](image-35.png)

![alt text](image-36.png)


###### 小旋转向量是什么？

![alt text](image-37.png)

###### 怎么把小旋转向量转成旋转矩阵？

![alt text](image-38.png)

###### 小角度（$\theta$很小）怎么做才稳定？

![alt text](image-39.png)

###### exp(·)在这里做什么：把“小旋转”变成旋转矩阵

![alt text](image-40.png)

###### 右乘表示把小旋转叠加到当前姿态

![alt text](image-41.png)

###### 为什么说非线性

![alt text](image-42.png)



## EKF怎么做？用雅可比把非线性“局部线性化”

![alt text](image-43.png)

### 一维直观理解

![alt text](image-45.png)

### 多维就是“切平面/切超平面”

![alt text](image-46.png)

### $z_k = h(x_k) + v$ 是什么？

![alt text](image-47.png)

### $H$是什么？

![alt text](image-48.png)


## EKF = 卡尔曼滤波 + 雅可比做局部线性化

![alt text](image-44.png)

# EKF完整步骤

## 预测

### 状态预测

![alt text](image-49.png)

### 状态线性化（雅可比）

![alt text](image-50.png)

### 协方差预测

![alt text](image-51.png)

## 更新

### 观测预测

![alt text](image-52.png)

### 观测线性化（雅可比）

![alt text](image-53.png)

### 残差

![alt text](image-54.png)

### 残差协方差

![alt text](image-55.png)

你用$\hat{x_k}^-$去预测观测$\hat{z_k}$，但$\hat{x_k}^-$本身不是准的，他有协方差$P^-$

把状态不确定性通过观测映射到量测空间，就是：

$Var(h(x)) \approx HP^-H^T$

$R$来自传感器本身的噪声

$S$就是预测误差投影到量测空间的那部分不确定+传感器自身噪声



### 增益

![alt text](image-56.png)

增益就是把“量测残差”变成“状态修正量”的映射矩阵，并自动做“信谁多一点”的权衡。

#### 最直观的一维标量情况（建立直觉）

![alt text](image-59.png)

#### 把“残差变白”视角，$S^{-1}$是在做归一化

![alt text](image-60.png)

#### “先把状态误差投影到量测空间，再拉回去”视角

![alt text](image-61.png)

##### 先把符号放到同一个“误差世界”里

![alt text](image-62.png)

##### 为什么使用$H$

###### 从观测模型开始

![alt text](image-63.png)

###### 为什么会变成$r \approx H\delta{x} + v$

![alt text](image-64.png)

##### 看Kalman增益公式直观理解

![alt text](image-65.png)

* 如果某个状态分量与残差几乎不相关$\Rightarrow$对应$P^-H$那一行很小$\Rightarrow$K那一行很小$\Rightarrow$**基本不更新该分量**

* 如果相关性很强$\Rightarrow$K大$\Rightarrow$**更新幅度大**



### 状态更新

![alt text](image-57.png)

![alt text](image-66.png)

### 协方差更新

![alt text](image-58.png)

![alt text](image-67.png)

#### 误差缩放矩阵

![alt text](image-68.png)


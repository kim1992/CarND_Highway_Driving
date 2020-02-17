---
typora-root-url: ./
---

# Prediction

## 1. OVERVIEW 

### Input & Output 

A **prediction module** uses a map and data from sensor fusion to generate predictions for what all other **dynamic objects** in view are likely to do. 

####  预测：地图 + 传感器融合



## 2. APPROACHES

![approaches](/approaches.png)

#### For example：

![ex1](/ex1.png)

![ex1](/ex2.png)

![ex3](/ex3.png)

#### Analysis：

1. Determining **maximum safe turning speed** on a wet road.

   *Model-based*. 基于模型的方法结合我们的物理知识（摩擦，力等），以准确（或几乎准确）计算出车辆何时开始在潮湿的路面上打滑。

2. Predicting the behavior of an **unidentified object** sitting on the road.

   *Data-driven*. 由于我们甚至都不知道这个对象是什么，因此基于模型的预测方法几乎是不可能的。

3. Predicting the behavior of a vehicle on a two lane highway in light traffic.

   Both good. (1) 我们在公路行驶情况下需要建模的行为很少，而且物理学都非常了解，因此model-based 基于模型的方法可以起作用。(2) 在类似情况下收集大量训练数据相对容易，因此data-driven 纯数据驱动的方法也可以工作。



### 2.1 Data-driven approaches（数据驱动法）

Data-driven approaches solve the prediction problem in two phases:

- Off-line training（离线训练）**机器学习**
- On-line training（在线预测）

#### 2.1.1 Off-line training

![offline](/offline.png)

##### 1. 获取轨迹数据（<u>**需要大量数据**</u>）

##### 2. 数据清理

##### 3. 定义相似度

- 与human common-sense相符的相似性定义。

##### 4. 无监督学习聚类

- 机器学习算法对观测到的轨迹进行聚类。

##### 5. 定义原型轨迹

- 为每个cluster识别少量的典型“原型”轨迹。

#### 2.1.2 On-line prediction

![online](/online.png)

算法一旦被训练完成，该算法便可以用来实际预测车辆的轨迹。

##### 1. 观测局部轨迹*partial trajectory*

- 当目标车辆行驶时，可以想到它在其后面留下了“局部轨迹”。

##### 2. 比较原型轨迹*prototype trajectories*（通过与off-line时相同的similarity来测量）

- 将此局部轨迹*partial*与原型轨迹*prototyp*e的相应部分进行比较。
- 当这些局部轨迹更相似时，其similarity相对于其他轨迹应会增加。

##### 3. 生成预测（将最相似的轨迹the most similar作为result）

- 对于每个聚类，我们确定最可能的原型轨迹。 
- 广播*broadcast*这些轨迹的每一个以及相关的概率。

#### **Data-driven的缺点：**

- 仅仅依赖于历史的事实和数据，进行对未来行为的预测。
- 然而事实上，我们往往需要考虑 *driver的行为习惯、物理学、车辆动力学*



### 2.2 Model-based approach

![model-based](/model-based.png)

##### 1. 识别常见的驾驶行为（变道、转弯、过路）

##### 2. 定义每个过程模型***process model*** （详细解释见 Process model）

- 驾驶员行为 vs 每个模型的预期

##### 3. 更新置信度（观测结果 vs 过程模型的output）

- Probability：利用**Multimodal Estimation**多模型算法计算出各种行为的可能性。
  - AMM
- classification：对驾驶员意图*driver intent*进行分类。
  - Naive Bayes Classifier

##### 4. 生成轨迹

- Trajectory generation is straightforward once we have a process model.

  **简单地反复迭代**模型，至完全覆盖所需的时间轴

- Note that each iteration of the process model will necessarily add uncertainty to our prediction.

  过程模型的每次迭代都必然会给我们的预测增加不确定性。

  

## 3. FRENET COORDINATES

![frenet](/frenet.png)

It looks straight!



## 4. PROCESS MODEL（过程模型）

![process model1](/process model1.png)

Process Models are first used to compare a **target vehicle's observed behavior** to the behavior we would expect for **each of the maneuvers we've created models for**. 

被观测到的目标车辆的行为 vs 模型创建的每种操作所期望的行为

![process model 2](/process model 2.jpg)

左图：一辆汽车的两个图像。 在时间k-1处，我们预测了汽车要直走还是右走的位置。 然后在时间k，我们查看汽车的实际位置。 

右图：显示了该车的观测到的s坐标以及我们当时预期该车所在位置的概率分布。 在这种情况下，我们观察到的s处，向右转弯比直行的概率更高。



## 5. MULTIMODAL ESTIMATION（多模型估计）

处理不确定性，维护一些预期，预期里包含driver接下来的行为倾向。

- Autonomous multiple model algorithm（AMM）

![amm](/amm.png)

- 公式：（实则为**朴素贝叶斯分类算法 Naive Bayes Classifier**）

  ![nb1](/nb1.png)

  如果我们忽略分母（因为它仅用于normalization），则可以使用

  ![nb2](/nb2.png)

  

  where the **μ~k~^(i)^** is the probability that model number **i** is the correct model at time **k** and **L~k~^(i)^** is the **likelihood** for that model (as computed by comparison to process model).

   **μ~k~^(i)^** 是模型编号**i**在时间**k**处为正确模型的概率，**L~k~^(i)^**是该模型的可能性（通过与过程模型进行比较计算得出）。

  
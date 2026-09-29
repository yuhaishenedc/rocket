#include <float.h>
#include <math.h>
#include <string.h>

#define RIDGE_N 60				// 岭回归模型的输入特征总数，同时也是法方程矩阵维度
#define RIDGE_NORMAL_RHS 3

enum
{
	RIDGE_OK = 0,
	RIDGE_EINVAL = -1,
	RIDGE_ENUMERIC = -2,
	RIDGE_ENOTSPD = -3
};

typedef struct
{
	float factor[RIDGE_N][RIDGE_N];
	float rhs[RIDGE_N][RIDGE_NORMAL_RHS];
	float inverseDiagonal[RIDGE_N];
}ridge_normal_workspace_f32;

typedef char ridge_float_must_be_32_bits[(sizeof(float) == 4) ? 1 : -1];

/*
 * @brief 判断单精度数值是否为有限值
 * @param value 待检查的单精度数值
 * @note 不依赖平台特定的isfinite实现
 */
static int RidgeFiniteF32(float value)
{
	return value == value && value <= FLT_MAX && value >= -FLT_MAX;
}

/*
 * @brief 使用单精度标量Cholesky求解三个右端的岭回归法方程
 * @param xtx 按行存储的X转置X矩阵下三角
 * @param xty 按行存储的X转置Y矩阵
 * @param lambda 岭回归对角正则化系数
 * @param coefficients 输出的60乘3回归系数矩阵
 * @param work Cholesky分解和三右端求解工作区
 * @note 仅在返回RIDGE_OK时发布新的回归系数
 */
static int RidgeSolveNormal(const float *xtx, const float *xty, float lambda,
	float *coefficients,
	ridge_normal_workspace_f32 *work)
{
	if (NULL == xtx || NULL == xty || NULL == coefficients || NULL == work ||
		!RidgeFiniteF32(lambda) || lambda <= 0.0F)
	{
		return RIDGE_EINVAL;
	}

	/*--------------------复制法方程下三角并加入岭回归正则项--------------------*/
	for (int row = 0; row < RIDGE_N; row++)
	{
		for (int col = 0; col <= row; col++)
		{
			float value = xtx[row * RIDGE_N + col];
			if (row == col)
			{
				value += lambda;
			}
			if (!RidgeFiniteF32(value))
			{
				return RIDGE_ENUMERIC;
			}
			work->factor[row][col] = value;
		}
		for (int outputIndex = 0; outputIndex < RIDGE_NORMAL_RHS; outputIndex++)
		{
			float value = xty[row * RIDGE_NORMAL_RHS + outputIndex];
			if (!RidgeFiniteF32(value))
			{
				return RIDGE_ENUMERIC;
			}
			work->rhs[row][outputIndex] = value;
		}
	}

	/*--------------------原地计算完整矩阵下三角的Cholesky因子--------------------*/
	for (int row = 0; row < RIDGE_N; row++)
	{
		for (int col = 0; col <= row; col++)
		{
			float value = work->factor[row][col];
			for (int inner = 0; inner < col; inner++)
			{
				value -= work->factor[row][inner] * work->factor[col][inner];
			}

			if (!RidgeFiniteF32(value))
			{
				return RIDGE_ENUMERIC;
			}

			if (row == col)
			{
				if (value <= 0.0F)
				{
					return RIDGE_ENOTSPD;
				}
				float diagonal = sqrtf(value);
				float inverseDiagonal = 1.0F / diagonal;
				if (!RidgeFiniteF32(inverseDiagonal))
				{
					return RIDGE_ENUMERIC;
				}
				work->factor[row][col] = diagonal;
				work->inverseDiagonal[row] = inverseDiagonal;
			}
			else
			{
				value *= work->inverseDiagonal[col];
				if (!RidgeFiniteF32(value))
				{
					return RIDGE_ENUMERIC;
				}
				work->factor[row][col] = value;
			}
		}
	}

	/*--------------------对三个右端执行前向替代--------------------*/
	for (int row = 0; row < RIDGE_N; row++)
	{
		for (int outputIndex = 0; outputIndex < RIDGE_NORMAL_RHS; outputIndex++)
		{
			float value = work->rhs[row][outputIndex];
			for (int inner = 0; inner < row; inner++)
			{
				value -= work->factor[row][inner] * work->rhs[inner][outputIndex];
			}
			value *= work->inverseDiagonal[row];
			if (!RidgeFiniteF32(value))
			{
				return RIDGE_ENUMERIC;
			}
			work->rhs[row][outputIndex] = value;
		}
	}

	/*--------------------对三个右端执行回代--------------------*/
	for (int row = RIDGE_N - 1; row >= 0; row--)
	{
		for (int outputIndex = 0; outputIndex < RIDGE_NORMAL_RHS; outputIndex++)
		{
			float value = work->rhs[row][outputIndex];
			for (int inner = row + 1; inner < RIDGE_N; inner++)
			{
				value -= work->factor[inner][row] * work->rhs[inner][outputIndex];
			}
			value *= work->inverseDiagonal[row];
			if (!RidgeFiniteF32(value))
			{
				return RIDGE_ENUMERIC;
			}
			work->rhs[row][outputIndex] = value;
		}
	}

	for (int row = 0; row < RIDGE_N; row++)
	{
		for (int outputIndex = 0; outputIndex < RIDGE_NORMAL_RHS; outputIndex++)
		{
			coefficients[row * RIDGE_NORMAL_RHS + outputIndex] =
				work->rhs[row][outputIndex];
		}
	}

	return RIDGE_OK;
}

#define		SHIP_WINDOW_SIZE_30S	(151)
#define		SHIP_WINDOW_SIZE_20S	(101)
#define		SHIP_WINDOW_SIZE_10S	(51)
#define		LOST_NUM			    (50)
#define		SHIP_POOL_CAPACITY		(100)
#define		SHIP_MAX_OUTPUT_POOLS	(2)
#define		SHIP_MAX_TRACKED_POOLS	(4)
#define		SHIP_REGRESSION_WIN		(20)
#define		SHIP_SIGNAL_DIM			(3)
typedef char ship_feature_count_must_match_ridge_n[
	(SHIP_REGRESSION_WIN * SHIP_SIGNAL_DIM == RIDGE_N) ? 1 : -1];
typedef struct
{
	float predAngle[SHIP_WINDOW_SIZE_20S];
	double tPred[SHIP_WINDOW_SIZE_20S];
}PredResult;
typedef struct
{
	double pool[SHIP_POOL_CAPACITY];
	int poolSize;
}ClusterPool;
typedef struct
{
	/*--------------------每200ms是否更新判断--------------------*/
    double dTimeStore;                              	// 200ms内最近一次更新数据时间
    float dAttangleStore[3];                        	// 200ms内最近一次更新船体姿态数据
    unsigned char bUpdate200ms;                     	// 200ms内数据是否更新过

	/*--------------------30s内存储的原始数据（环形缓冲区）--------------------*/
	int headPhysicalIndex;						    // 环形缓冲区最旧样本的物理索引
	int cnt;										    // 已存储数据计数
	double dTimeStore30s[SHIP_WINDOW_SIZE_30S];     	// 30s内存储的原始时间数据
	float Attangle30s[SHIP_WINDOW_SIZE_30S][3];		// 30s内船姿态（滚转、俯仰、偏航）
	float trainData[SHIP_WINDOW_SIZE_30S][SHIP_SIGNAL_DIM];	// 30s平滑后姿态角数据（滚转/俯仰/偏航）

    unsigned char bUseCheck;                        // 船姿数据启用标志字
    
    /*--------------------峰值+周期法--------------------*/
	int method1StartLogicalIndex;
	float detPitch[SHIP_WINDOW_SIZE_10S];			// 近10s内均值归零俯仰角数据
	double dTCross[SHIP_WINDOW_SIZE_10S];		    // 近10s内穿越时间点
	double validPeriod[SHIP_WINDOW_SIZE_10S];	    // 近10s内数据周期
	
	float MatrixB[RIDGE_N][RIDGE_NORMAL_RHS];
	int ridgeSolveStatus;
	float normalMatrix[RIDGE_N][RIDGE_N];			// XT*X
	float normalRhs[RIDGE_N][RIDGE_NORMAL_RHS];		// XT*Y
	float PredData[SHIP_WINDOW_SIZE_30S][SHIP_SIGNAL_DIM];
    
	float PredSlope[SHIP_WINDOW_SIZE_30S];
	float dNeg[SHIP_WINDOW_SIZE_30S];
	int NegIdx[SHIP_WINDOW_SIZE_30S];
	int valleyIdx[SHIP_MAX_OUTPUT_POOLS];
	int breakPoints[SHIP_WINDOW_SIZE_30S];
	int segEnds[SHIP_WINDOW_SIZE_30S];
	float segSlopes[SHIP_WINDOW_SIZE_30S];
	double tzPredAll[SHIP_MAX_OUTPUT_POOLS];
	double tzFiltered[SHIP_MAX_OUTPUT_POOLS];

	PredResult stPredResult;

	int poolCount;
	ClusterPool activePools[SHIP_MAX_TRACKED_POOLS];
	
	double currentTzPreds[SHIP_MAX_TRACKED_POOLS];
	int keepIdxArr[SHIP_MAX_TRACKED_POOLS];
	double timeToTargets[SHIP_MAX_TRACKED_POOLS];

	int sortIdx[SHIP_MAX_TRACKED_POOLS];

	double closestTz[SHIP_MAX_OUTPUT_POOLS];
	ClusterPool newPools[SHIP_MAX_OUTPUT_POOLS];

	/*--------------------输出预测值--------------------*/
	double tDown1;
	double tDown2;

}ST_SHIP_PRIV;
ST_SHIP_PRIV s_stShipPriv = { 0 };

typedef struct 
{
	unsigned char bUpdate_ship;
	double t_fly;
	double AttAngle_ship[3];
}IF_CombinedNavi;
IF_CombinedNavi g_CombinedNaviInput = { 0 };

#define TRUE 1
#define FALSE 0
#define CONTROL_PERIOD 0.02
#define SHIP_SAMPLE_INTERVAL_CYCLES 10
#define PI 3.1415926


/*
 * @brief 将环形缓冲区的逻辑索引转换为物理数组索引
 * @param logicalIndex 相对于最旧有效样本的逻辑索引
 * @note 调用方必须保证逻辑索引位于有效样本范围内，写入尾部时允许等于cnt
 */
static inline int ShipLogicalToPhysicalIndex(int logicalIndex)
{
	int physicalIndex = s_stShipPriv.headPhysicalIndex + logicalIndex;

	if (physicalIndex >= SHIP_WINDOW_SIZE_30S)
	{
		physicalIndex -= SHIP_WINDOW_SIZE_30S;
	}

	return physicalIndex;
}

/*
 * @brief 获取指定逻辑位置的采样时间
 * @param logicalIndex 有效样本的逻辑索引，范围为0到cnt-1
 * @note 函数只读取数据，不检查索引是否越界
 */
static inline double ShipTimeAt(int logicalIndex)
{
	return s_stShipPriv.dTimeStore30s[ShipLogicalToPhysicalIndex(logicalIndex)];
}

/*
 * @brief 获取指定逻辑位置的三维姿态数据
 * @param logicalIndex 有效样本的逻辑索引，范围为0到cnt-1
 * @note 返回环形缓冲区内部存储地址，不得在对应样本被覆盖后继续使用
 */
static inline float *ShipAttangleAt(int logicalIndex)
{
	return s_stShipPriv.Attangle30s[ShipLogicalToPhysicalIndex(logicalIndex)];
}

/*
 * @brief 获取指定逻辑位置的三维平滑姿态角训练数据
 * @param logicalIndex 有效样本的逻辑索引，范围为0到cnt-1
 * @note 返回环形缓冲区内部存储地址，不得在对应样本被覆盖后继续使用
 */
static inline float *ShipTrainDataAt(int logicalIndex)
{
	return s_stShipPriv.trainData[ShipLogicalToPhysicalIndex(logicalIndex)];
}

/*
 * @brief 计算当前窗口内能够构造多少条有效岭回归训练数据
 * @param 无
 * @note 每条训练行由连续20个三维特征和后续一个三维目标组成
 */
static int ShipRegressionRowCount(void)
{
	return s_stShipPriv.cnt > SHIP_REGRESSION_WIN ? s_stShipPriv.cnt - SHIP_REGRESSION_WIN : 0;
}

/*
 * @brief 向持久方程增加或减去一条训练行贡献
 * @param regressionRow 待处理训练行的逻辑索引，取值范围[0,sampleNum-SHIP_REGRESSION_WIN)
 * @param scale 贡献方向，1表示加入，-1表示减去
 * @note 同时更新XT*X下三角和XT*Y
 */
static void ShipAccumulateRegressionRow(int regressionRow, float scale)
{
	int rowCount = ShipRegressionRowCount();

	if (regressionRow < 0 || regressionRow >= rowCount)
	{
		return;
	}

	/*--------------------把连续20个时刻的三维姿态角展开为1*60向量--------------------*/
	float feature[RIDGE_N] = { 0 };
	int featureIndex = 0;
	for (int historyIndex = 0; historyIndex < SHIP_REGRESSION_WIN; historyIndex++)
	{
		const float *history = ShipTrainDataAt(regressionRow + historyIndex);
		for (int channelIndex = 0; channelIndex < SHIP_SIGNAL_DIM; channelIndex++)
		{
			feature[featureIndex++] = history[channelIndex];
		}
	}

	/*--------------------增量更新岭回归方程XT*Y矩阵--------------------*/
	/*
		XT为60*m，Y为m*3，新增行或删除行对m进行加1或减1，因此可以按新增或删除直接更新其影响的所有60*3个元素
	*/
	const float *target = ShipTrainDataAt(regressionRow + SHIP_REGRESSION_WIN);	// 紧邻上面20个时刻的下一时刻
	for (int row = 0; row < RIDGE_N; row++)
	{
		float scaledFeature = scale * feature[row];

		for (int outputIndex = 0; outputIndex < RIDGE_NORMAL_RHS; outputIndex++)
		{
			s_stShipPriv.normalRhs[row][outputIndex] += scaledFeature * target[outputIndex];
		}
	}

	/*--------------------增量更新岭回归方程XT*X矩阵的下三角--------------------*/
	for (int row = 0; row < RIDGE_N; row++)
	{
		float scaledFeature = scale * feature[row];
		for (int col = 0; col <= row; col++)
		{
			s_stShipPriv.normalMatrix[row][col] += scaledFeature * feature[col];
		}
	}
}

/*
 * @brief 批量增加或减去连续回归训练行贡献
 * @param firstRow 首个训练行逻辑索引
 * @param lastRow 最后一个训练行逻辑索引
 * @param scale 贡献方向，1表示加入，-1表示减去
 * @note 输入范围会被限制在当前有效训练行范围内
 */
static void ShipAccumulateRegressionRows(int firstRow, int lastRow, float scale)
{
	int rowCount = ShipRegressionRowCount();

	if (firstRow < 0)
	{
		firstRow = 0;
	}
	if (lastRow >= rowCount)
	{
		lastRow = rowCount - 1;
	}

	for (int regressionRow = firstRow; regressionRow <= lastRow; regressionRow++)
	{
		ShipAccumulateRegressionRow(regressionRow, scale);
	}
}

/*
 * @brief 更新包含指定平滑样本范围的全部训练行贡献
 * @param firstTrainIndex 首个发生变化的平滑样本逻辑索引
 * @param lastTrainIndex 最后一个发生变化的平滑样本逻辑索引
 * @param scale 贡献方向，1表示加入，-1表示减去
 * @note 一个平滑样本可能同时作为多个训练行的特征或目标
 */
static void ShipAccumulateRowsAffectedByTrainRange(int firstTrainIndex, int lastTrainIndex, float scale)
{
	/*
		假设当前窗口内有m个平滑样本，逻辑索引为0~m-1，则训练行逻辑索引为0~m-SHIP_REGRESSION_WIN-1
		对于任意一个平滑样本逻辑索引i，它可能作为训练行的特征出现在逻辑索引为i-SHIP_REGRESSION_WIN+1~i的训练行中
		它也可能作为训练行的目标出现在逻辑索引为i-SHIP_REGRESSION_WIN的训练行中
		因此，所有受影响的训练行逻辑索引范围为[firstTrainIndex-SHIP_REGRESSION_WIN,lastTrainIndex]
	*/
	ShipAccumulateRegressionRows(firstTrainIndex - SHIP_REGRESSION_WIN, lastTrainIndex, scale);
}

/*
 * @brief 更新指定逻辑位置的三维平滑姿态角训练数据
 * @param logicalIndex 待更新样本的逻辑索引
 * @note 使用当前位置前后各最多两个样本计算姿态角的滑动平均
 */
static void ShipUpdateTrainData(int logicalIndex)
{
	if (logicalIndex < 0 || logicalIndex >= s_stShipPriv.cnt)
	{
		return;
	}

	int leftLogicalIndex = logicalIndex > 1 ? logicalIndex - 2 : 0;
	int rightLogicalIndex = logicalIndex + 2 < s_stShipPriv.cnt ? logicalIndex + 2 : s_stShipPriv.cnt - 1;
	float angleSum[SHIP_SIGNAL_DIM] = { 0.0F };
	for (int sampleLogicalIndex = leftLogicalIndex; sampleLogicalIndex <= rightLogicalIndex; sampleLogicalIndex++)
	{
		float *angle = ShipAttangleAt(sampleLogicalIndex);
		for (int channelIndex = 0; channelIndex < SHIP_SIGNAL_DIM; channelIndex++)
		{
			angleSum[channelIndex] += angle[channelIndex];
		}
	}

	float scale = 1.0F / (float)(rightLogicalIndex - leftLogicalIndex + 1);
	float *train = ShipTrainDataAt(logicalIndex);
	for (int channelIndex = 0; channelIndex < SHIP_SIGNAL_DIM; channelIndex++)
	{
		train[channelIndex] = angleSum[channelIndex] * scale;
	}
}

/*
 * @brief 执行船姿数据采集、窗口维护、特征处理和预测计算
 * @param 无
 * @note 当前按20ms周期调用并每200ms写入一次最新样本；测试信号代码启用时会覆盖外部输入
 */
void testShip(void)
{
	unsigned char sampleInserted = FALSE;

#if 1
    static int timeCounter = 0;
    timeCounter++;
    g_CombinedNaviInput.bUpdate_ship = TRUE;
    g_CombinedNaviInput.t_fly = timeCounter * CONTROL_PERIOD;
    g_CombinedNaviInput.AttAngle_ship[0] = 1.5 * sin( 2 * PI / 10 * CONTROL_PERIOD * timeCounter);
    g_CombinedNaviInput.AttAngle_ship[1] = 1.5 * sin( 2 * PI / 10 * CONTROL_PERIOD * timeCounter);
    g_CombinedNaviInput.AttAngle_ship[2] = 1.5 * sin( 2 * PI / 10 * CONTROL_PERIOD * timeCounter);
#endif
    
    /*--------------------记录最后一次有效数据--------------------*/
	if (TRUE == g_CombinedNaviInput.bUpdate_ship)
	{
        s_stShipPriv.dTimeStore = g_CombinedNaviInput.t_fly;
		for (int channelIndex = 0; channelIndex < 3; channelIndex++)
		{
			s_stShipPriv.dAttangleStore[channelIndex] =
				(float)g_CombinedNaviInput.AttAngle_ship[channelIndex];
		}
        s_stShipPriv.bUpdate200ms = TRUE;
	}

    /*--------------------数据降频，200ms数据更新一次--------------------*/
	static int counter = 0;
	if (++counter >= SHIP_SAMPLE_INTERVAL_CYCLES)
	{
		counter = 0;

		/*--------------------数据存储及异常处理--------------------*/
		if (TRUE == s_stShipPriv.bUpdate200ms)
		{
			/*--------------------仅在新样本入队前清除30s以前的数据--------------------*/
			int removeCount = 0;
			while (removeCount < s_stShipPriv.cnt && s_stShipPriv.dTimeStore - ShipTimeAt(removeCount) > 30.0)
			{
				removeCount++;
			}
			if (removeCount > 0)
			{
				/*--------------------减去随头部样本离开窗口的回归行贡献--------------------*/
				ShipAccumulateRegressionRows(0, removeCount - 1, -1.0F);

				int newHeadPhysicalIndex = s_stShipPriv.headPhysicalIndex + removeCount;

				s_stShipPriv.headPhysicalIndex = newHeadPhysicalIndex >= SHIP_WINDOW_SIZE_30S
					? newHeadPhysicalIndex - SHIP_WINDOW_SIZE_30S : newHeadPhysicalIndex;
				s_stShipPriv.cnt -= removeCount;
				s_stShipPriv.method1StartLogicalIndex = s_stShipPriv.method1StartLogicalIndex > removeCount
					? s_stShipPriv.method1StartLogicalIndex - removeCount : 0;
			}

			/*--------------------修改右边界平滑数据前减去其旧训练行贡献--------------------*/
			/*
				s_stShipPriv.cnt - 2 和 s_stShipPriv.cnt - 1 在新加入数据后，平滑结果改变
			*/
			ShipAccumulateRowsAffectedByTrainRange(s_stShipPriv.cnt - 2, s_stShipPriv.cnt - 1, -1.0F);

			/*--------------------将逻辑尾部转换为物理写入位置--------------------*/
			int writePhysicalIndex = ShipLogicalToPhysicalIndex(s_stShipPriv.cnt);
			int previousPhysicalIndex = -1;
			if (s_stShipPriv.cnt > 0)
			{
				previousPhysicalIndex = ShipLogicalToPhysicalIndex(s_stShipPriv.cnt - 1);
			}
			s_stShipPriv.cnt++;

			/*--------------------写入本周期更新值--------------------*/
			s_stShipPriv.dTimeStore30s[writePhysicalIndex] = s_stShipPriv.dTimeStore;
			s_stShipPriv.Attangle30s[writePhysicalIndex][0] = s_stShipPriv.dAttangleStore[0];
			s_stShipPriv.Attangle30s[writePhysicalIndex][1] = s_stShipPriv.dAttangleStore[2];
			s_stShipPriv.Attangle30s[writePhysicalIndex][2] = s_stShipPriv.dAttangleStore[1];
			for (int channelIndex = 0; channelIndex < 3; channelIndex++)
			{
				/*--------------------异常数据剔除--------------------*/
				if (previousPhysicalIndex >= 0
					&& fabsf(s_stShipPriv.Attangle30s[writePhysicalIndex][channelIndex] -
						s_stShipPriv.Attangle30s[previousPhysicalIndex][channelIndex]) > 3.0F)
				{
					s_stShipPriv.Attangle30s[writePhysicalIndex][channelIndex] = s_stShipPriv.Attangle30s[previousPhysicalIndex][channelIndex];
				}
			}

			/*--------------------加入数据后更新右边界平滑姿态角--------------------*/
			ShipUpdateTrainData(s_stShipPriv.cnt - 3);
			ShipUpdateTrainData(s_stShipPriv.cnt - 2);
			ShipUpdateTrainData(s_stShipPriv.cnt - 1);

			/*--------------------加入右边界更新值和新增训练行贡献--------------------*/
			ShipAccumulateRowsAffectedByTrainRange(s_stShipPriv.cnt - 3,
				s_stShipPriv.cnt - 1, 1.0F);

			/*--------------------更新method1近10s数据起始索引--------------------*/
			while (s_stShipPriv.method1StartLogicalIndex < s_stShipPriv.cnt &&
				   ShipTimeAt(s_stShipPriv.cnt - 1) - ShipTimeAt(s_stShipPriv.method1StartLogicalIndex) > 10.0)
			{
				s_stShipPriv.method1StartLogicalIndex++;
			}

			sampleInserted = TRUE;
		}
		s_stShipPriv.bUpdate200ms = FALSE;
	}

	/*--------------------数据个数不足，不进入后续判断--------------------*/
	if (s_stShipPriv.cnt < 2)
	{
		s_stShipPriv.tDown1 = -1.0;
		s_stShipPriv.tDown2 = -1.0;
		return;
	}

	/*--------------------没有新样本时复用上次预测结果--------------------*/
	if (FALSE == sampleInserted)
	{
		return;
	}

	/*--------------------检查数据是否启用（仅判别一次）--------------------*/
	if (FALSE == s_stShipPriv.bUseCheck)
	{
		double latestTime = ShipTimeAt(s_stShipPriv.cnt - 1);

		/*--------------------历史数据不足5秒，保持未启用状态--------------------*/
		if (latestTime - ShipTimeAt(0) < 5.0)
		{
			return;
		}

		/*--------------------启用前检查最新数据是否已经超时--------------------*/
		if (g_CombinedNaviInput.t_fly - latestTime > 1.0)
		{
			return;
		}

		/*--------------------从最新数据向前检查最近5秒内是否存在超过1秒的数据断点--------------------*/
		for (int logicalIndex = s_stShipPriv.cnt - 1; logicalIndex > 0; logicalIndex--)
		{
			double currentTime = ShipTimeAt(logicalIndex);

			if (latestTime - currentTime > 5.0)
			{
				break;
			}

			if (currentTime - ShipTimeAt(logicalIndex - 1) > 1.0)
			{
				return;
			}
		}

		/*--------------------启动条件全部满足后锁存，后续周期不再重复检查--------------------*/
		s_stShipPriv.bUseCheck = TRUE;
		return;
	}

	/*--------------------检查启动条件（存储船姿数据大于10s）--------------------*/
	if (ShipTimeAt(s_stShipPriv.cnt - 1) - ShipTimeAt(0) < 10)
	{
		return;
	}

	/*--------------------计算相位调整标志字--------------------*/
	int tgoFlag = 1;
	int pitchSmallCount = 0;
	for (int logicalIndex = 0; logicalIndex < s_stShipPriv.cnt; logicalIndex++)
	{
		if (fabsf(ShipTrainDataAt(logicalIndex)[1]) < 0.5F)
		{
			pitchSmallCount++;
		}
	}
	if (pitchSmallCount * 200 > s_stShipPriv.cnt * 199)	// 无需调整
	{
		tgoFlag = 0;
	}
	if (FALSE == tgoFlag)
	{
		return;
	}

	/*--------------------固定60维Cholesky分解和三右端求解工作区--------------------*/
	static ridge_normal_workspace_f32 ridgeWork;

	/*--------------------峰值法+周期法--------------------*/
	if (ShipTimeAt(s_stShipPriv.cnt - 1) - ShipTimeAt(0) < 29.5)
	{
		/*--------------------method1EndIndex指向近10s内的数据--------------------*/
		int method1StartLogicalIndex = s_stShipPriv.method1StartLogicalIndex;

		/*--------------------对10s内数据进行赋值--------------------*/
		int Num10s = s_stShipPriv.cnt - method1StartLogicalIndex;		// 10s内数据总数
		memset(s_stShipPriv.detPitch, 0, sizeof(s_stShipPriv.detPitch));
		float SumPitch = 0.0F;						// 10s内俯仰角和
		for (int logicalIndex = method1StartLogicalIndex; logicalIndex < s_stShipPriv.cnt; logicalIndex++)
		{
			SumPitch += ShipTrainDataAt(logicalIndex)[1];
		}
		SumPitch /= (float)Num10s;
		for (int windowLogicalOffset = 0; windowLogicalOffset < Num10s; windowLogicalOffset++)
		{
			s_stShipPriv.detPitch[windowLogicalOffset] = ShipTrainDataAt(method1StartLogicalIndex + windowLogicalOffset)[1] - SumPitch;
		}

		//////////////////////////////计算平均周期//////////////////////////////
		double tAvg = 4.0;

		/*--------------------记录所有的穿越时间点--------------------*/
		memset(s_stShipPriv.dTCross, 0, sizeof(s_stShipPriv.dTCross));
		int dTCrossCount = 0;
		int currentState = 0;
		for (int windowLogicalOffset = 0; windowLogicalOffset < Num10s; windowLogicalOffset++)
		{
			/*--------------------找第一个点--------------------*/
			if (0 == currentState)
			{
				if (s_stShipPriv.detPitch[windowLogicalOffset] >= 0.1)
				{
					currentState = 1;
				}
				else if (s_stShipPriv.detPitch[windowLogicalOffset] < -0.1)
				{
					currentState = -1;
				}
			}
			/*--------------------当前在上方，寻找向下穿越点--------------------*/
			else if (1 == currentState)
			{
				if (s_stShipPriv.detPitch[windowLogicalOffset] <= -0.1)
				{
					s_stShipPriv.dTCross[dTCrossCount++] =
						ShipTimeAt(method1StartLogicalIndex + windowLogicalOffset);
					currentState = -1;
				}
			}
			/*--------------------当前在下方，寻找向上穿越点--------------------*/
			else if (-1 == currentState)
			{
				if (s_stShipPriv.detPitch[windowLogicalOffset] >= 0.1)
				{
					s_stShipPriv.dTCross[dTCrossCount++] =
						ShipTimeAt(method1StartLogicalIndex + windowLogicalOffset);
					currentState = 1;
				}
			}
		}

		/*--------------------遍历计算所有可得的周期--------------------*/
		int validPeriodCount = 0;
		if (dTCrossCount >= 3)
		{
			for (int i = 0; i < dTCrossCount - 2; i++)
			{
				double fullPeriod = s_stShipPriv.dTCross[i + 2] - s_stShipPriv.dTCross[i];
				if (fullPeriod > 3.0)
				{
					s_stShipPriv.validPeriod[validPeriodCount++] = fullPeriod;
				}
			}
		}
		if (validPeriodCount == 0 && dTCrossCount >= 2)
		{
			for (int i = 0; i < dTCrossCount - 1; i++)
			{
				double halfPeriod = s_stShipPriv.dTCross[i + 1] - s_stShipPriv.dTCross[i];
				if (halfPeriod > 1.5)
				{
					s_stShipPriv.validPeriod[validPeriodCount++] = halfPeriod * 2.0;
				}
			}
		}

		/*--------------------综合求平均--------------------*/
		if (validPeriodCount > 0)
		{
			double sum = 0;
			for (int i = 0; i < validPeriodCount; i++)
			{
				sum += s_stShipPriv.validPeriod[i];
			}
			tAvg = sum * 1.0 / validPeriodCount;
		}

		//////////////////////////////寻找最近峰值//////////////////////////////

		/*--------------------默认退化到当前最新点--------------------*/
		int peakLogicalIndex = Num10s;
		double peakTime = ShipTimeAt(s_stShipPriv.cnt - 1);

		if (Num10s >= 3)
		{
			/*--------------------计算峰峰值--------------------*/
			float mx = ShipTrainDataAt(method1StartLogicalIndex)[1];
			float mn = ShipTrainDataAt(method1StartLogicalIndex)[1];
			for (int logicalIndex = method1StartLogicalIndex + 1;
				 logicalIndex < s_stShipPriv.cnt;
				 logicalIndex++)
			{
				if (ShipTrainDataAt(logicalIndex)[1] > mx)
				{
					mx = ShipTrainDataAt(logicalIndex)[1];
				}
				if (ShipTrainDataAt(logicalIndex)[1] < mn)
				{
					mn = ShipTrainDataAt(logicalIndex)[1];
				}
			}
			float pk2pk = mx - mn;

			/*--------------------计算迟滞阈值--------------------*/
			float HDiff = (pk2pk * 0.08F < 1E-3F) ? 1E-3F : pk2pk * 0.08F;

			/*--------------------从后向前扫描（靠近当前时刻优先级最高）波峰条件：左侧显著上升，右侧显著下降--------------------*/
			int found = 0;
			for (int logicalIndex = s_stShipPriv.cnt - 2;
				 logicalIndex > method1StartLogicalIndex;
				 logicalIndex--)
			{
				float diffLeft = ShipTrainDataAt(logicalIndex)[1] -
					ShipTrainDataAt(logicalIndex - 1)[1];
				float diffRight = ShipTrainDataAt(logicalIndex + 1)[1] -
					ShipTrainDataAt(logicalIndex)[1];

				if (diffLeft > HDiff && diffRight < -HDiff)
				{
					peakLogicalIndex = logicalIndex;
					found = 1;
					break;
				}
			}

			/*--------------------退化1（未找到显著波峰，标准降为普通符号过0）--------------------*/
			if (0 == found)
			{
				for (int logicalIndex = s_stShipPriv.cnt - 2;
					 logicalIndex > method1StartLogicalIndex;
					 logicalIndex--)
				{
					float diffLeft = ShipTrainDataAt(logicalIndex)[1] -
						ShipTrainDataAt(logicalIndex - 1)[1];
					float diffRight = ShipTrainDataAt(logicalIndex + 1)[1] -
						ShipTrainDataAt(logicalIndex)[1];

					if (diffLeft > 0 && diffRight < 0)
					{
						peakLogicalIndex = logicalIndex;
						found = 1;
						break;
					}
				}
			}

			/*--------------------退化2（若缓存单调（无任何波峰），退化为全局最大值索引）--------------------*/
			if (0 == found)
			{
				float peakValTmp = ShipTrainDataAt(method1StartLogicalIndex)[1];
				peakLogicalIndex = method1StartLogicalIndex;
				for (int logicalIndex = method1StartLogicalIndex;
					 logicalIndex < s_stShipPriv.cnt;
					 logicalIndex++)
				{
					if (ShipTrainDataAt(logicalIndex)[1] > peakValTmp)
					{
						peakValTmp = ShipTrainDataAt(logicalIndex)[1];
						peakLogicalIndex = logicalIndex;
					}
				}
			}

			peakTime = ShipTimeAt(peakLogicalIndex);
		}

		//////////////////////////////计算当前时刻后最近的两个下降速度最大时刻//////////////////////////////
		/*--------------------计算基准下降时刻--------------------*/
		double baseDownTime = peakTime + tAvg / 4.0;

		/*--------------------计算需要偏移多少个周期才能使下降时刻在当前时刻之后--------------------*/
		int nOffset = 0;
		if (baseDownTime <= g_CombinedNaviInput.t_fly)
		{
			/*--------------------基准时刻在当前时刻之前，需要往后推移--------------------*/
			double timeDiff = g_CombinedNaviInput.t_fly - baseDownTime;
			nOffset = (int)floor(timeDiff / tAvg);

			/*--------------------向上取整--------------------*/
			double remainder = timeDiff - nOffset * tAvg;
			if (remainder > 1E-6)
			{
				nOffset += 1;
			}
		}

		/*--------------------计算两个下降时刻--------------------*/
		s_stShipPriv.tDown1 = baseDownTime + nOffset * tAvg;
		s_stShipPriv.tDown2 = baseDownTime + (nOffset + 1) * tAvg;

		/*--------------------确保输出的是未来时刻--------------------*/
		if (s_stShipPriv.tDown1 <= g_CombinedNaviInput.t_fly)
		{
			s_stShipPriv.tDown1 += tAvg;
			s_stShipPriv.tDown2 += tAvg;
		}
	}
	else
	{

		/*--------------------Cholesky分解并直接求解三个右端项--------------------*/
		const float ridgeLambda = 5.0F;
		s_stShipPriv.ridgeSolveStatus = RidgeSolveNormal(
			*s_stShipPriv.normalMatrix,
			*s_stShipPriv.normalRhs,
			ridgeLambda,
			*s_stShipPriv.MatrixB,
			&ridgeWork);
		if (s_stShipPriv.ridgeSolveStatus != RIDGE_OK)
		{
			return;
		}
        
        /*--------------------预测未来20s--------------------*/
		double averageSampleInterval = (ShipTimeAt(s_stShipPriv.cnt - 1) - ShipTimeAt(0)) / (s_stShipPriv.cnt - 1);
		int dynamicPredLen = (int)round(20.0 / averageSampleInterval);
		if (dynamicPredLen > SHIP_WINDOW_SIZE_20S - 1)
		{
			dynamicPredLen = SHIP_WINDOW_SIZE_20S - 1;
		}
		double dt = averageSampleInterval;

		/*--------------------初始化最近20个时刻的历史数据--------------------*/
		static float hist[SHIP_REGRESSION_WIN][SHIP_SIGNAL_DIM] = { 0 };
		for (int historyLogicalIndex = 0; historyLogicalIndex < SHIP_REGRESSION_WIN; historyLogicalIndex++)
		{
			memcpy(hist[historyLogicalIndex], ShipTrainDataAt(s_stShipPriv.cnt - SHIP_REGRESSION_WIN + historyLogicalIndex), sizeof(hist[historyLogicalIndex]));
		}

		/*--------------------指向当前最旧样本，也是下一次预测结果的写入位置--------------------*/
		int historyHeadPhysicalIndex = 0;
		for (int predictionIndex = 0; predictionIndex < dynamicPredLen; predictionIndex++)
		{
			/*--------------------使用3个独立累加器计算三个姿态角--------------------*/
			float y0 = 0.0F;
			float y1 = 0.0F;
			float y2 = 0.0F;

			const float *coefficientRow = &s_stShipPriv.MatrixB[0][0];
			int historyPhysicalIndex = historyHeadPhysicalIndex;
			for (int historyLogicalIndex = 0; historyLogicalIndex < SHIP_REGRESSION_WIN; historyLogicalIndex++)
			{
				const float *history = hist[historyPhysicalIndex];

				/*--------------------针对MatrixB的三列，将History的20*3进行展开计算--------------------*/
				for (int featureIndex = 0; featureIndex < SHIP_SIGNAL_DIM; featureIndex++)
				{
					const float featureValue = history[featureIndex];

					y0 += featureValue * coefficientRow[0];
					y1 += featureValue * coefficientRow[1];
					y2 += featureValue * coefficientRow[2];

					coefficientRow += SHIP_SIGNAL_DIM;
				}

				historyPhysicalIndex++;
				if (historyPhysicalIndex == SHIP_REGRESSION_WIN)
				{
					historyPhysicalIndex = 0;
				}
			}

			/*--------------------保存预测结果并覆盖最旧历史样本--------------------*/
			float *prediction = s_stShipPriv.PredData[predictionIndex];
			float *historyWrite = hist[historyHeadPhysicalIndex];
			prediction[0] = historyWrite[0] = y0;
			prediction[1] = historyWrite[1] = y1;
			prediction[2] = historyWrite[2] = y2;

			historyHeadPhysicalIndex++;
			if (historyHeadPhysicalIndex == SHIP_REGRESSION_WIN)
			{
				historyHeadPhysicalIndex = 0;
			}
		}
		/*--------------------在预测的俯仰角上寻找最快下降沿--------------------*/
		
		/*--------------------构建预测角序列--------------------*/
		float currentAngle = ShipTrainDataAt(s_stShipPriv.cnt - 1)[1];
		int fullPredLen = dynamicPredLen + 1;
		s_stShipPriv.stPredResult.predAngle[0] = currentAngle;
		for (int i = 0; i < dynamicPredLen; i++)
		{
			s_stShipPriv.stPredResult.predAngle[i + 1] = s_stShipPriv.PredData[i][1];
		}

		/*--------------------构建时间序列--------------------*/
		s_stShipPriv.stPredResult.tPred[0] = g_CombinedNaviInput.t_fly;
		for (int i = 0; i < dynamicPredLen; i++)
		{
			s_stShipPriv.stPredResult.tPred[i + 1] = g_CombinedNaviInput.t_fly + (i + 1) * dt;
		}

		/*--------------------计算预测俯仰角的一阶导数--------------------*/
		s_stShipPriv.PredSlope[0] = (float)((s_stShipPriv.stPredResult.predAngle[1] - s_stShipPriv.stPredResult.predAngle[0]) / dt);
		s_stShipPriv.PredSlope[fullPredLen - 1] = (float)((s_stShipPriv.stPredResult.predAngle[fullPredLen - 1] - s_stShipPriv.stPredResult.predAngle[fullPredLen - 2]) / dt);
		for (int i = 1; i < fullPredLen - 1; i++)
		{
			s_stShipPriv.PredSlope[i] = (float)((s_stShipPriv.stPredResult.predAngle[i + 1] - s_stShipPriv.stPredResult.predAngle[i - 1]) / (2.0 * dt));
		}

		/*--------------------寻找所有有效下降点--------------------*/
		int minZeros = 2;
		int interestNum = 2;
		for (int i = 0; i < fullPredLen; i++)
		{
			s_stShipPriv.dNeg[i] = (s_stShipPriv.PredSlope[i] < 0.0F) ? s_stShipPriv.PredSlope[i] : 0.0F;
		}
		int negCount = 0;
		for (int i = 0; i < fullPredLen; i++)
		{
			if (s_stShipPriv.dNeg[i] < 0)
			{
				s_stShipPriv.NegIdx[negCount++] = i;
			}
		}
		int valleyCount = 0;
		if (negCount > 0)
		{
			/*--------------------寻找下降段的断点--------------------*/
			int breakCount = 0;
			for (int i = 0; i < negCount - 1; i++)
			{
				if (s_stShipPriv.NegIdx[i + 1] - s_stShipPriv.NegIdx[i] > minZeros + 1)
				{
					s_stShipPriv.breakPoints[breakCount++] = i;
				}
			}

			/*--------------------划分段落--------------------*/
			for (int i = 0; i < breakCount; i++)
			{
				s_stShipPriv.segEnds[i] = s_stShipPriv.breakPoints[i];
			}
			s_stShipPriv.segEnds[breakCount] = negCount - 1;
			int startSegIdx = 0;
			int processNum = (interestNum < breakCount + 1) ? interestNum : breakCount + 1;
			for (int k = 0; k < processNum; k++)
			{
				int start = startSegIdx;
				int end = s_stShipPriv.segEnds[k];

				/*--------------------提取当前下降周期的所有索引--------------------*/
				int segLen = end - start + 1;
				for (int i = 0; i < segLen; i++)
				{
					s_stShipPriv.segSlopes[i] = s_stShipPriv.PredSlope[s_stShipPriv.NegIdx[start + i]];
				}

				/*--------------------找斜率最小（最负）的那个点--------------------*/
				int minLocalIdx = 0;
				float minVal = s_stShipPriv.segSlopes[0];
				for (int i = 0; i < segLen; i++)
				{
					if (s_stShipPriv.segSlopes[i] < minVal)
					{
						minVal = s_stShipPriv.segSlopes[i];
						minLocalIdx = i;
					}
				}

				/*--------------------记录该点的全局索引--------------------*/
				s_stShipPriv.valleyIdx[valleyCount++] = s_stShipPriv.NegIdx[start + minLocalIdx];

				/*--------------------更新下一段的起点--------------------*/
				startSegIdx = end + 1;
			}
		}
		/*--------------------二次抛物线插值，计算极值时刻--------------------*/
		int tzCount = 0;
		for (int k = 0; k < valleyCount; k++)
		{
			int idx = s_stShipPriv.valleyIdx[k];
			if (idx <= 0 || idx >= fullPredLen - 1)
			{
				continue;
			}
			float y1 = s_stShipPriv.PredSlope[idx - 1];
			float y2 = s_stShipPriv.PredSlope[idx];
			float y3 = s_stShipPriv.PredSlope[idx + 1];
			double t2 = s_stShipPriv.stPredResult.tPred[idx];

			/*--------------------抛物线顶点公式求精确极值时间--------------------*/
			float denominator = y1 - 2.0F * y2 + y3;
			double tExcat = t2;
			if (fabsf(denominator) > 1E-6F)
			{
				tExcat = t2 - (dt / 2.0) * (y3 - y1) / denominator;
			}

			s_stShipPriv.tzPredAll[tzCount++] = tExcat;
		}

		/*--------------------剔除时间轴上可能回退的错误点--------------------*/
		int tzFilteredCount = 0;
		for (int i = 0; i < tzCount; i++)
		{
			if (s_stShipPriv.tzPredAll[i] > g_CombinedNaviInput.t_fly)
			{
				s_stShipPriv.tzFiltered[tzFilteredCount++] = s_stShipPriv.tzPredAll[i];
			}
		}
		/*--------------------构建原始聚类池--------------------*/
		for (int zi = 0; zi < tzFilteredCount; zi++)
		{
			double tz = s_stShipPriv.tzFiltered[zi];
			int foundMatch = 0;

			for (int p = 0; p < s_stShipPriv.poolCount; p++)
			{
				double poolMean = 0.0;
				for (int i = 0; i < s_stShipPriv.activePools[p].poolSize; i++)
				{
					poolMean += s_stShipPriv.activePools[p].pool[i];
				}
				if (s_stShipPriv.activePools[p].poolSize > 0)
				{
					poolMean /= s_stShipPriv.activePools[p].poolSize;
				}
				if (fabs(tz - poolMean) <= 1.5)
				{
					foundMatch = 1;
					if (poolMean - g_CombinedNaviInput.t_fly > 3)
					{
						ClusterPool *activePool = &s_stShipPriv.activePools[p];
						if (activePool->poolSize < SHIP_POOL_CAPACITY)
						{
							activePool->pool[activePool->poolSize++] = tz;
						}
						else
						{
							/*--------------------聚类池满时保留最近结果--------------------*/
							memmove(&activePool->pool[0], &activePool->pool[1],
								(SHIP_POOL_CAPACITY - 1) * sizeof(activePool->pool[0]));
							activePool->pool[SHIP_POOL_CAPACITY - 1] = tz;
						}
					}
				}
			}

			if (!foundMatch &&
				(tz - g_CombinedNaviInput.t_fly) > 0 &&
				s_stShipPriv.poolCount < SHIP_MAX_TRACKED_POOLS)
			{
				s_stShipPriv.activePools[s_stShipPriv.poolCount].pool[0] = tz;
				s_stShipPriv.activePools[s_stShipPriv.poolCount].poolSize = 1;
				s_stShipPriv.poolCount++;
			}
		}

		/*--------------------提取聚类池有效目标并截断--------------------*/
		int tzPredCount = 0;
		for (int p = 0; p < s_stShipPriv.poolCount; p++)
		{
			double poolMean = 0.0;
			for (int i = 0; i < s_stShipPriv.activePools[p].poolSize; i++)
			{
				poolMean += s_stShipPriv.activePools[p].pool[i];
			}
			poolMean /= s_stShipPriv.activePools[p].poolSize;

			double timeToTarget = poolMean - g_CombinedNaviInput.t_fly;
			if (timeToTarget > 0)
			{
				s_stShipPriv.currentTzPreds[tzPredCount] = poolMean;
				s_stShipPriv.keepIdxArr[tzPredCount] = p;
				s_stShipPriv.timeToTargets[tzPredCount] = timeToTarget;
				tzPredCount++;
			}
		}

		if (tzPredCount > 0)
		{
			for (int i = 0; i < tzPredCount; i++)
			{
				s_stShipPriv.sortIdx[i] = i;
			}
			for (int i = 0; i < tzPredCount - 1; i++)
			{
				for (int j = 0; j < tzPredCount - i - 1; j++)
				{
					if (s_stShipPriv.timeToTargets[s_stShipPriv.sortIdx[j]] > s_stShipPriv.timeToTargets[s_stShipPriv.sortIdx[j + 1]])
					{
						int temp = s_stShipPriv.sortIdx[j];
						s_stShipPriv.sortIdx[j] = s_stShipPriv.sortIdx[j + 1];
						s_stShipPriv.sortIdx[j + 1] = temp;
					}
				}
			}
			int keepNum = (SHIP_MAX_OUTPUT_POOLS < tzPredCount) ? SHIP_MAX_OUTPUT_POOLS : tzPredCount;
			int newPoolCount = 0;
			for (int i = 0; i < keepNum; i++)
			{
				int idx = s_stShipPriv.sortIdx[i];
				s_stShipPriv.closestTz[i] = s_stShipPriv.currentTzPreds[idx];

				/*--------------------复制池子--------------------*/
				int oriPoolIdx = s_stShipPriv.keepIdxArr[idx];
				s_stShipPriv.newPools[i].poolSize = s_stShipPriv.activePools[oriPoolIdx].poolSize;
				for (int j = 0; j < s_stShipPriv.activePools[oriPoolIdx].poolSize; j++)
				{
					s_stShipPriv.newPools[i].pool[j] = s_stShipPriv.activePools[oriPoolIdx].pool[j];
				}
				newPoolCount++;
			}

			/*--------------------更新全局缓冲池--------------------*/
			s_stShipPriv.poolCount = newPoolCount;
			for (int i = 0; i < s_stShipPriv.poolCount; i++)
			{
				s_stShipPriv.activePools[i].poolSize = s_stShipPriv.newPools[i].poolSize;
				for (int j = 0; j < s_stShipPriv.newPools[i].poolSize; j++)
				{
					s_stShipPriv.activePools[i].pool[j] = s_stShipPriv.newPools[i].pool[j];
				}
			}

			/*--------------------赋值输出--------------------*/
			s_stShipPriv.tDown1 = -1;
			s_stShipPriv.tDown2 = -1;
			if (s_stShipPriv.poolCount >= 1)
			{
				s_stShipPriv.tDown1 = s_stShipPriv.closestTz[0];
			}
			if (s_stShipPriv.poolCount >= 2)
			{
				s_stShipPriv.tDown2 = s_stShipPriv.closestTz[1];
			}
		}
	}
}

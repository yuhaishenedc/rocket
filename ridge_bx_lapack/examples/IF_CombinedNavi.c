#define		SHIP_WINDOW_SIZE_30S	(301)
#define		SHIP_WINDOW_SIZE_20S	(201)
#define		SHIP_WINDOW_SIZE_10S	(101)
#define		LOST_NUM			    (50)
typedef struct
{
	double tDown1;
	double tDown2;
	double predAngle[100];
	int predLen;
	double tPred[100];
	int tLen;
}PredResult;
typedef struct
{
	double pool[100];
	int poolSize;
}ClusterPool;
typedef struct
{
	/*--------------------每100ms是否更新判断--------------------*/
    double dTimeStore;                              	// 100ms内最近一次更新数据时间
    double dAttangleStore[3];                       	// 100ms内最近一次更新船体姿态数据
    unsigned char bUpdate100ms;                     	// 100ms内数据是否更新过

	/*--------------------30s内存储的原始数据--------------------*/
	double dTimeStore30s[SHIP_WINDOW_SIZE_30S];     	// 30s内存储的原始时间数据
	double Attangle30s[SHIP_WINDOW_SIZE_30S][3];		// 30s内船姿态（滚转、偏航、俯仰）
	double AttangleDiff30s[SHIP_WINDOW_SIZE_30S][3];	// 30s内船角速度

    unsigned char bCalculated;                      // 本周期是否进行计算
    unsigned char bUseCheck;                        // 船姿数据启用标志字
    int cnt;									    // 已存储数据计数
    
    /*--------------------峰值+周期法--------------------*/
	int method1EndIndex;
	double detPitch[SHIP_WINDOW_SIZE_10S];			// 近10s内均值归零俯仰角数据
	double dTCross[SHIP_WINDOW_SIZE_10S];		    // 近10s内穿越时间点
	double validPeriod[SHIP_WINDOW_SIZE_10S];	    // 近10s内数据周期
	
	double XTrain[SHIP_WINDOW_SIZE_30S][120];		
	double MatrixB[120][6];
	double PredData[SHIP_WINDOW_SIZE_30S][6];
    
    double trainData[SHIP_WINDOW_SIZE_30S][6];	    // 平滑后数据（滚转/偏航/俯仰角、滚转/偏航/俯仰角速度）
	
    double PredSlope[SHIP_WINDOW_SIZE_30S];
	double dNeg[SHIP_WINDOW_SIZE_30S];
	int NegIdx[SHIP_WINDOW_SIZE_30S];
	int valleyIdx[SHIP_WINDOW_SIZE_30S];
	int breakPoints[SHIP_WINDOW_SIZE_30S];
	int segEnds[SHIP_WINDOW_SIZE_30S];
	double segSlopes[SHIP_WINDOW_SIZE_30S];
	double tzPredAll[SHIP_WINDOW_SIZE_30S];
	double tzFiltered[SHIP_WINDOW_SIZE_30S];

	PredResult stPredResult;

	int poolCount;
	ClusterPool activePools[100];
	
	double currentTzPreds[SHIP_WINDOW_SIZE_30S];
	int keepIdxArr[SHIP_WINDOW_SIZE_30S];
	double timeToTargets[SHIP_WINDOW_SIZE_30S];

	int sortIdx[SHIP_WINDOW_SIZE_30S];

	double closestTz[SHIP_WINDOW_SIZE_30S];
	ClusterPool newPools[100];

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
#define PI 3.1415926

/*--------------------更新指定位置的姿态角差分--------------------*/
static void ShipUpdateAttangleDiff(int pos)
{
	int left;
	int right;
	int i;
	double dt;

	if (pos < 0 || pos >= s_stShipPriv.cnt)
	{
		return;
	}

	if (s_stShipPriv.cnt < 2)
	{
		for (i = 0; i < 3; i++)
		{
			s_stShipPriv.AttangleDiff30s[pos][i] = 0.0;
		}
		return;
	}

	left = pos > 0 ? pos - 1 : 0;
	right = pos + 1 < s_stShipPriv.cnt ? pos + 1 : s_stShipPriv.cnt - 1;
	dt = s_stShipPriv.dTimeStore30s[right] - s_stShipPriv.dTimeStore30s[left];
	if (dt <= 0.0)
	{
		for (i = 0; i < 3; i++)
		{
			s_stShipPriv.AttangleDiff30s[pos][i] = 0.0;
		}
		return;
	}

	for (i = 0; i < 3; i++)
	{
		s_stShipPriv.AttangleDiff30s[pos][i] =
			(s_stShipPriv.Attangle30s[right][i] -
			 s_stShipPriv.Attangle30s[left][i]) / dt;
	}
}

/*--------------------更新指定位置的六维平滑数据--------------------*/
static void ShipUpdateTrainData(int pos)
{
	int left;
	int right;
	int i;
	int j;
	double angleSum[3] = { 0.0 };
	double diffSum[3] = { 0.0 };
	double scale;

	if (pos < 0 || pos >= s_stShipPriv.cnt)
	{
		return;
	}

	left = pos > 1 ? pos - 2 : 0;
	right = pos + 2 < s_stShipPriv.cnt ? pos + 2 : s_stShipPriv.cnt - 1;
	for (i = left; i <= right; i++)
	{
		for (j = 0; j < 3; j++)
		{
			angleSum[j] += s_stShipPriv.Attangle30s[i][j];
			diffSum[j] += s_stShipPriv.AttangleDiff30s[i][j];
		}
	}

	scale = 1.0 / (right - left + 1);
	for (i = 0; i < 3; i++)
	{
		s_stShipPriv.trainData[pos][i] = angleSum[i] * scale;
		s_stShipPriv.trainData[pos][i + 3] = diffSum[i] * scale;
	}
}


void testShip()
{
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
        memcpy(s_stShipPriv.dAttangleStore, g_CombinedNaviInput.AttAngle_ship, sizeof(g_CombinedNaviInput.AttAngle_ship));
        s_stShipPriv.bUpdate100ms = TRUE;
	}
    
    /*--------------------清除30s以前的数据--------------------*/
	int index = 0;
	for (index = 0; index < s_stShipPriv.cnt && g_CombinedNaviInput.t_fly - s_stShipPriv.dTimeStore30s[index] > 30; index++);
	if (index > 0)
	{
		memmove(&s_stShipPriv.dTimeStore30s[0], &s_stShipPriv.dTimeStore30s[index], sizeof(s_stShipPriv.dTimeStore30s[0]) * (s_stShipPriv.cnt - index));
		memmove(&s_stShipPriv.Attangle30s[0], &s_stShipPriv.Attangle30s[index], sizeof(s_stShipPriv.Attangle30s[0]) * (s_stShipPriv.cnt - index));
		memmove(&s_stShipPriv.AttangleDiff30s[0], &s_stShipPriv.AttangleDiff30s[index], sizeof(s_stShipPriv.AttangleDiff30s[0]) * (s_stShipPriv.cnt - index));
		memmove(&s_stShipPriv.trainData[0], &s_stShipPriv.trainData[index], sizeof(s_stShipPriv.trainData[0]) * (s_stShipPriv.cnt - index));
		s_stShipPriv.cnt -= index;
		s_stShipPriv.method1EndIndex -= index;
		if (s_stShipPriv.method1EndIndex < 0)
		{
			s_stShipPriv.method1EndIndex = 0;
		}

		/*--------------------删除数据后更新左边界差分和平滑值--------------------*/
		ShipUpdateAttangleDiff(0);
		ShipUpdateTrainData(0);
		ShipUpdateTrainData(1);
		ShipUpdateTrainData(2);
	}

    /*--------------------数据降频，100ms数据更新一次--------------------*/
	static int counter = 0;
	if (++counter >= 5)
	{
		counter = 0;
        
        /*--------------------数据存储及异常处理--------------------*/
        if(TRUE == s_stShipPriv.bUpdate100ms)
        {
            s_stShipPriv.Attangle30s[s_stShipPriv.cnt][0] = s_stShipPriv.dAttangleStore[0];
            s_stShipPriv.Attangle30s[s_stShipPriv.cnt][1] = s_stShipPriv.dAttangleStore[2];
            s_stShipPriv.Attangle30s[s_stShipPriv.cnt][2] = s_stShipPriv.dAttangleStore[1];
            for (int i = 0; i < 3; i++)
            {
                if ((s_stShipPriv.cnt > 0) 
                    && (s_stShipPriv.Attangle30s[s_stShipPriv.cnt][i] - s_stShipPriv.Attangle30s[s_stShipPriv.cnt - 1][i] > 3))	// 异常剔除
                {
                    s_stShipPriv.Attangle30s[s_stShipPriv.cnt][i] = s_stShipPriv.Attangle30s[s_stShipPriv.cnt - 1][i];
                }
            }
            s_stShipPriv.dTimeStore30s[s_stShipPriv.cnt] = s_stShipPriv.dTimeStore;
            s_stShipPriv.cnt++;

            /*--------------------加入数据后更新右边界差分和平滑值--------------------*/
            ShipUpdateAttangleDiff(s_stShipPriv.cnt - 2);
            ShipUpdateAttangleDiff(s_stShipPriv.cnt - 1);
            ShipUpdateTrainData(s_stShipPriv.cnt - 4);
            ShipUpdateTrainData(s_stShipPriv.cnt - 3);
            ShipUpdateTrainData(s_stShipPriv.cnt - 2);
            ShipUpdateTrainData(s_stShipPriv.cnt - 1);

            /*--------------------更新method1近10s数据起始索引--------------------*/
            while (s_stShipPriv.method1EndIndex < s_stShipPriv.cnt &&
                   s_stShipPriv.dTimeStore30s[s_stShipPriv.cnt - 1] -
                   s_stShipPriv.dTimeStore30s[s_stShipPriv.method1EndIndex] > 10.0)
            {
                s_stShipPriv.method1EndIndex++;
            }
        }
        s_stShipPriv.bUpdate100ms = FALSE;
	}

	/*--------------------检查数据是否启用--------------------*/
	if (s_stShipPriv.cnt > 0 && FALSE == s_stShipPriv.bUseCheck)
	{
		if (s_stShipPriv.dTimeStore30s[s_stShipPriv.cnt - 1] - s_stShipPriv.dTimeStore30s[0] >= 5)
		{
			s_stShipPriv.bUseCheck = TRUE;
			for (int i = 1; i < s_stShipPriv.cnt; i++)
			{
				if (s_stShipPriv.dTimeStore30s[s_stShipPriv.cnt - 1] - s_stShipPriv.dTimeStore30s[i] <= 5
					&& s_stShipPriv.dTimeStore30s[i] - s_stShipPriv.dTimeStore30s[i - 1] > 1)
				{
					s_stShipPriv.bUseCheck = FALSE;
					break;
				}
			}
			if (g_CombinedNaviInput.t_fly - s_stShipPriv.dTimeStore30s[s_stShipPriv.cnt - 1] > 1)
			{
				s_stShipPriv.bUseCheck = FALSE;
			}
		}
		return;
	}

	/*--------------------检查启动条件（存储船姿数据大于10s）--------------------*/
	if (s_stShipPriv.dTimeStore30s[s_stShipPriv.cnt - 1] - s_stShipPriv.dTimeStore30s[0] < 10)
	{
		return;
	}

	/*--------------------差分数据更新（差分时间间隔每次重算，因此放在这里）--------------------*/
	double dt = (s_stShipPriv.dTimeStore30s[s_stShipPriv.cnt - 1] - s_stShipPriv.dTimeStore30s[0]) / (s_stShipPriv.cnt - 1);
	VectorSub(s_stShipPriv.Attangle30s[1], s_stShipPriv.Attangle30s[0], 3, s_stShipPriv.AttangleDiff30s[0]);
	VectorMulConst(s_stShipPriv.AttangleDiff30s[0], 3, 1.0 / dt, s_stShipPriv.AttangleDiff30s[0]);												// 第一个点
	VectorSub(s_stShipPriv.Attangle30s[s_stShipPriv.cnt - 1], s_stShipPriv.Attangle30s[s_stShipPriv.cnt - 2], 3, s_stShipPriv.AttangleDiff30s[s_stShipPriv.cnt - 1]);
	VectorMulConst(s_stShipPriv.AttangleDiff30s[s_stShipPriv.cnt - 1], 3, 1.0 / dt, s_stShipPriv.AttangleDiff30s[s_stShipPriv.cnt - 1]);		// 最后一个点
	for (int i = 1; i < s_stShipPriv.cnt - 1; i++)
	{
		VectorSub(s_stShipPriv.Attangle30s[i + 1], s_stShipPriv.Attangle30s[i - 1], 3, s_stShipPriv.AttangleDiff30s[i]);
		VectorMulConst(s_stShipPriv.AttangleDiff30s[i], 3, 1.0 / (2 * dt), s_stShipPriv.AttangleDiff30s[i]);
	}

	/*--------------------姿态角及差分平均值更新--------------------*/
    int PitchSmallCount = 0;
	for (int i = 0; i < s_stShipPriv.cnt; i++)
	{
		int left = i > 1 ? i - 2 : 0;
		int right = i + 2 < s_stShipPriv.cnt ? i + 2 : s_stShipPriv.cnt - 1;
		double average[3] = { 0 };
		double averageDiff[3] = { 0 };
		for (int j = left; j <= right; j++)
		{
			VectorAdd(average, s_stShipPriv.Attangle30s[j], 3, average);
			VectorAdd(averageDiff, s_stShipPriv.AttangleDiff30s[j], 3, averageDiff);
		}
		VectorMulConst(average, 3, 1.0 / (right - left + 1), s_stShipPriv.trainData[i]);
		VectorMulConst(averageDiff, 3, 1.0 / (right - left + 1), &s_stShipPriv.trainData[i][3]);

		if (s_stShipPriv.trainData[i][1] < 0.5)
		{
			PitchSmallCount++;
		}
	}

	/*--------------------计算相位调整标志字--------------------*/
	int tgoFlag = 1;
	if (PitchSmallCount > s_stShipPriv.cnt * 0.995)	// 无需调整
	{
		tgoFlag = 0;
	}
    
    /*--------------------30s后预测每周期预计算--------------------*/
    
#define WIN 20
#define DIM 6
#define COL (WIN * DIM)

    static int head = 0;        // 指向当前最旧元素
    static int headPre = 0;     // 指向上一次最旧元素
    static int count = 0;       // 当前有效行数
    static int countPre = 0;    // 上次有效行数
    
    static double G[120][120] = { 0 };
    static double H[120][6] = { 0 };
    
    static double oldX[120] = { 0 };    // 每次增量操作需要的临时120维变量
    static double newX[120] = { 0 };    
    static double oldY[6] = { 0 };
    
    static int n30sInit = 0;    // 第一次构建
    if(0 == n30sInit)
    {
        n30sInit = 1;
        /*--------------------岭回归对角元素仅加一次--------------------*/
        for (int i = 0; i < 120; i++)
        {
            G[i][i] += 5;
        }
    }
    
    int doDel = (head != headPre);
    int doAdd = (count != countPre);
    //const int removePair = doDel && (count >);       // 
    const int countAfterDel = head - count + 1;
    
    static double XTrainTXTrain[120][120] = { 0 };
    if (s_stShipPriv.dTimeStore30s[s_stShipPriv.cnt - 1] - s_stShipPriv.dTimeStore30s[0] >= 29.5)
    {
        /*--------------------第一次计算完整矩阵--------------------*/
        if(0 == n30sInit)
        {
            
            
            
        }
        
    }

	/*--------------------峰值法+周期法--------------------*/
	if (s_stShipPriv.dTimeStore30s[s_stShipPriv.cnt - 1] - s_stShipPriv.dTimeStore30s[0] < 29.5)
	{
		/*--------------------method1EndIndex指向近10s内的数据--------------------*/
		int method1EndIndex = s_stShipPriv.method1EndIndex;

		/*--------------------对10s内数据进行赋值--------------------*/
		int Num10s = s_stShipPriv.cnt - method1EndIndex;		// 10s内数据总数
		memset(s_stShipPriv.detPitch, 0, sizeof(s_stShipPriv.detPitch));
		double SumPitch = 0;							// 10s内俯仰角和
		for (int i = method1EndIndex; i < s_stShipPriv.cnt; i++)
		{
			SumPitch += s_stShipPriv.trainData[i][1];
		}
		SumPitch /= Num10s;
		for (int i = 0; i < Num10s; i++)
		{
			s_stShipPriv.detPitch[i] = s_stShipPriv.trainData[i + method1EndIndex][1] - SumPitch;
		}

		//////////////////////////////计算平均周期//////////////////////////////
		double tAvg = 4.0;

		/*--------------------记录所有的穿越时间点--------------------*/
		memset(s_stShipPriv.dTCross, 0, sizeof(s_stShipPriv.dTCross));
		int dTCrossCount = 0;
		int currentState = 0;
		for (int i = 0; i < Num10s; i++)
		{
			/*--------------------找第一个点--------------------*/
			if (0 == currentState)
			{
				if (s_stShipPriv.detPitch[i] >= 0.1)
				{
					currentState = 1;
				}
				else if (s_stShipPriv.detPitch[i] < -0.1)
				{
					currentState = -1;
				}
			}
			/*--------------------当前在上方，寻找向下穿越点--------------------*/
			else if (1 == currentState)
			{
				if (s_stShipPriv.detPitch[i] <= -0.1)
				{
					s_stShipPriv.dTCross[dTCrossCount++] = s_stShipPriv.dTimeStore30s[method1EndIndex + i];
					currentState = -1;
				}
			}
			/*--------------------当前在下方，寻找向上穿越点--------------------*/
			else if (-1 == currentState)
			{
				if (s_stShipPriv.detPitch[i] >= 0.1)
				{
					s_stShipPriv.dTCross[dTCrossCount++] = s_stShipPriv.dTimeStore30s[method1EndIndex + i];
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
		int peakIdx = Num10s;
		double peakVal = s_stShipPriv.detPitch[Num10s - 1];
		double peakTime = s_stShipPriv.dTimeStore30s[s_stShipPriv.cnt - 1];

		if (Num10s >= 3)
		{
			/*--------------------计算峰峰值--------------------*/
			double mx = s_stShipPriv.trainData[method1EndIndex][1];
			double mn = s_stShipPriv.trainData[method1EndIndex][1];
			for (int i = method1EndIndex + 1; i < s_stShipPriv.cnt; i++)
			{
				if (s_stShipPriv.trainData[i][1] > mx)
				{
					mx = s_stShipPriv.trainData[i][1];
				}
				if (s_stShipPriv.trainData[i][1] < mn)
				{
					mn = s_stShipPriv.trainData[i][1];
				}
			}
			double pk2pk = mx - mn;

			/*--------------------计算迟滞阈值--------------------*/
			double HDiff = (pk2pk * 0.08 < 1E-3) ? 1E-3 : pk2pk * 0.08;

			/*--------------------从后向前扫描（靠近当前时刻优先级最高）波峰条件：左侧显著上升，右侧显著下降--------------------*/
			double found = 0;
			for (int i = s_stShipPriv.cnt - 2; i > method1EndIndex; i--)
			{
				double diffLeft = s_stShipPriv.trainData[i][1] - s_stShipPriv.trainData[i - 1][1];
				double diffRight = s_stShipPriv.trainData[i + 1][1] - s_stShipPriv.trainData[i][1];

				if (diffLeft > HDiff && diffRight < -HDiff)
				{
					peakIdx = i;
					found = 1;
					break;
				}
			}

			/*--------------------退化1（未找到显著波峰，标准降为普通符号过0）--------------------*/
			if (0 == found)
			{
				for (int i = s_stShipPriv.cnt - 2; i > method1EndIndex; i--)
				{
					double diffLeft = s_stShipPriv.trainData[i][1] - s_stShipPriv.trainData[i - 1][1];
					double diffRight = s_stShipPriv.trainData[i + 1][1] - s_stShipPriv.trainData[i][1];

					if (diffLeft > 0 && diffRight < 0)
					{
						peakIdx = i;
						found = 1;
						break;
					}
				}
			}

			/*--------------------退化2（若缓存单调（无任何波峰），退化为全局最大值索引）--------------------*/
			if (0 == found)
			{
				double peakValTmp = s_stShipPriv.trainData[method1EndIndex][1];
				peakIdx = method1EndIndex;
				for (int i = method1EndIndex; i < s_stShipPriv.cnt; i++)
				{
					if (s_stShipPriv.trainData[i][1] > peakValTmp)
					{
						peakValTmp = s_stShipPriv.trainData[i][1];
						peakIdx = i;
					}
				}
			}

			peakVal = s_stShipPriv.trainData[peakIdx][1];
			peakTime = s_stShipPriv.dTimeStore30s[peakIdx];
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
		double tDown1 = baseDownTime + nOffset * tAvg;
		double tDown2 = baseDownTime + (nOffset + 1) * tAvg;

		/*--------------------确保输出的是未来时刻--------------------*/
		if (tDown1 <= g_CombinedNaviInput.t_fly)
		{
			tDown1 += tAvg;
			tDown2 += tAvg;
		}
	}
	else
	{
		int lag = 20;
		int numSamples = s_stShipPriv.cnt - lag;
		int numFeatures = 6;

        //////////////////////////////构建回归预测模型//////////////////////////////
        
        double (*YTrain)[6] = &s_stShipPriv.trainData[lag];

        /*--------------------计算左项--------------------*/
        static double blk[6][6];
        
        /*
            d表示两个20*6时间位置之间相差多少
        */
        for(int d = 0; d < lag; ++d)
        {
            memset(blk, 0, sizeof(blk));
            
            /*--------------------先完整计算这一条“块对角线”的第一个块，以进行后续递推--------------------*/
            for(int r = 0; r < numSamples; ++r)
            {
                const double *x = s_stShipPriv.trainData[r];
                const double *y = s_stShipPriv.trainData[r + d];
                
                for(int p = 0; p < numFeatures; ++p)
                {
                    const double v = x[p];
                    double *b = blk[p];
                    b[0] += v * y[0];
                    b[1] += v * y[1];
                    b[2] += v * y[2];
                    b[3] += v * y[3];
                    b[4] += v * y[4];
                    b[5] += v * y[5];
                }
            }
            
            /*
                沿当前d方向计算所有6 * 6的块
            */
            for(int a = 0; a < lag - d; ++a)
            {
                int bb = a + d;
                
                /*
                    把当前6*6 blk写到真正的120*120矩阵G中
                */
                for(int p = 0; p < numFeatures; ++p)
                {
                    for(int q = 0; q < numFeatures; ++q)
                    {
                        int row = a * numFeatures + p;
                        int col = bb * numFeatures + q;
                        
                        XTrainTXTrain[row][col] = blk[p][q];
                        // 对称矩阵
                        XTrainTXTrain[col][row] = blk[p][q];
                    }
                }
                
                /*--------------------已经到当前d的最后一个块，后面不再需要再更新blk--------------------*/
                if(a == lag - d + 1)
                {
                    break;
                }
                
                /*
                    不重新执行m次累加来计算下一个6*6块
                */
                
                /*--------------------上一个窗口中离开的两个6维数组--------------------*/
                const double *oldX = s_stShipPriv.trainData[a];
                const double *oldY = s_stShipPriv.trainData[a + d];
                
                /*--------------------新窗口最后进入的两个6维数组--------------------*/
                const double *newX = s_stShipPriv.trainData[a + numSamples];
                const double *newY = s_stShipPriv.trainData[a + numSamples + d];
                
                /*
                    更新整个6*6的块
                */
                for(int p = 0; p < numFeatures; p++)
                {
                    const double oldV = oldX[p];
                    const double newV = newX[p];
                    
                    double *b = blk[p];
                    
                    b[0] += newV * newY[0] - oldV * oldY[0];
                    b[1] += newV * newY[1] - oldV * oldY[1];
                    b[2] += newV * newY[2] - oldV * oldY[2];
                    b[3] += newV * newY[3] - oldV * oldY[3];
                    b[4] += newV * newY[4] - oldV * oldY[4];
                    b[5] += newV * newY[5] - oldV * oldY[5];
                }
            }
        }
        
        /*--------------------计算右项--------------------*/
        static double XTrainTYTrain[120][6] = { 0 };
        
        for(int r = 0; r < numSamples; r++)
        {
            /*--------------------提前取出，让其驻留在VFP寄存器--------------------*/
            const double y0 = YTrain[r][0];
            const double y1 = YTrain[r][1];
            const double y2 = YTrain[r][2];
            const double y3 = YTrain[r][3];
            const double y4 = YTrain[r][4];
            const double y5 = YTrain[r][5];
            
            for(int a = 0; a < lag; ++a)
            {
                const double *x = s_stShipPriv.trainData[r + a];
                double (*out)[6] = &XTrainTYTrain[a * numFeatures];
                
                for(int p = 0; p < numFeatures; ++p)
                {
                    const double v = x[p];
                    double *o = out[p];
                    
                    o[0] += v * y0;
                    o[1] += v * y1;
                    o[2] += v * y2;
                    o[3] += v * y3;
                    o[4] += v * y4;
                    o[5] += v * y5;
                }
            }
        }

		/*--------------------计算标准解析解--------------------*/
		MatrixInv(*XTrainTXTrain, 120);
		MatrixMultiply(*XTrainTXTrain, *XTrainTYTrain, 120, 120, 6, *s_stShipPriv.MatrixB);
        
        /*--------------------预测未来20s--------------------*/
		int dynamicPredLen = (int)round(20.0 / dt);
        static double hist[20][6] = { 0 };
        memcpy(hist, s_stShipPriv.trainData[s_stShipPriv.cnt - lag], sizeof(double) * 120);
        int head = 0;
		for (int i = 0; i < dynamicPredLen; i++)
		{
            /*--------------------使用6个独立寄存器，尽量让其保存在VFP寄存器中--------------------*/
			double y0 = 0, y1 = 0, y2 = 0, y3 = 0, y4 = 0, y5 = 0;
            
            /*--------------------b从B[0][0]开始，仅连续向前移动，尽量命中L1 cache--------------------*/
            const double *b = &s_stShipPriv.MatrixB[0][0];
            for(int k = 0; k < lag; ++k)
            {
                /*--------------------将环形缓冲拆成两段，避免热点计算未命中--------------------*/
                for(int j = head; j < 20; ++j, b += lag)
                {
                    /*--------------------将X保持在VFP中--------------------*/
                    const double x = hist[k][j];
                    
                    y0 += x * b[0];
                    y1 += x * b[1];
                    y2 += x * b[2];
                    y3 += x * b[3];
                    y4 += x * b[4];
                    y5 += x * b[5];
                }
                
                /*--------------------hist回绕，b继续向前扫描--------------------*/
                for(int j = 0; j < head; ++j, b += lag)
                {
                    /*--------------------将X保持在VFP中--------------------*/
                    const double x = hist[k][j];
                    
                    y0 += x * b[0];
                    y1 += x * b[1];
                    y2 += x * b[2];
                    y3 += x * b[3];
                    y4 += x * b[4];
                    y5 += x * b[5];
                }
            }
            
            double *out = s_stShipPriv.PredData[i];
            out[0] = hist[0][head] = y0;
            out[1] = hist[1][head] = y1;
            out[2] = hist[2][head] = y2;
            out[3] = hist[3][head] = y3;
            out[4] = hist[4][head] = y4;
            out[5] = hist[5][head] = y5;
            
            if(++head == numFeatures)
            {
                head = 0;
            }
		}
#if 0
		//////////////////////////////在预测的俯仰角上寻找最快下降沿//////////////////////////////
		
		/*--------------------构建预测角序列--------------------*/
		double currentAngle = s_stShipPriv.trainData[s_stShipPriv.cnt - 1][1];
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
		s_stShipPriv.PredSlope[0] = (s_stShipPriv.stPredResult.predAngle[1] - s_stShipPriv.stPredResult.predAngle[0]) / dt;
		s_stShipPriv.PredSlope[fullPredLen - 1] = (s_stShipPriv.stPredResult.predAngle[fullPredLen - 1] - s_stShipPriv.stPredResult.predAngle[fullPredLen - 2]) / dt;
		for (int i = 1; i < fullPredLen - 1; i++)
		{
			s_stShipPriv.PredSlope[i] = (s_stShipPriv.stPredResult.predAngle[i + 1] - s_stShipPriv.stPredResult.predAngle[i - 1]) / (2.0 * dt);
		}

		/*--------------------寻找所有有效下降点--------------------*/
		int minZeros = 2;
		int interestNum = 2;
		for (int i = 0; i < fullPredLen; i++)
		{
			s_stShipPriv.dNeg[i] = (s_stShipPriv.PredSlope[i] < 0) ? s_stShipPriv.PredSlope[i] : 0.0;
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
				double minVal = s_stShipPriv.segSlopes[0];
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
#endif
#if 0
		//////////////////////////////二次抛物线插值，计算极值时刻//////////////////////////////
		int tzCount = 0;
		for (int k = 0; k < valleyCount; k++)
		{
			int idx = s_stShipPriv.valleyIdx[k];
			if (idx <= 0 || idx >= fullPredLen - 1)
			{
				continue;
			}
			double y1 = s_stShipPriv.PredSlope[idx - 1];
			double y2 = s_stShipPriv.PredSlope[idx];
			double y3 = s_stShipPriv.PredSlope[idx + 1];
			double t2 = s_stShipPriv.stPredResult.tPred[idx];

			/*--------------------抛物线顶点公式求精确极值时间--------------------*/
			double denominator = y1 - 2.0 * y2 + y3;
			double tExcat = t2;
			if (fabs(denominator) > 1E-6)
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
#endif
#if 0
		//////////////////////////////构建原始聚类池//////////////////////////////
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
						s_stShipPriv.activePools[p].pool[s_stShipPriv.activePools[p].poolSize++] = tz;
					}
				}
			}

			if (!foundMatch && (tz - g_CombinedNaviInput.t_fly) > 0)
			{
				s_stShipPriv.activePools[s_stShipPriv.poolCount].pool[0] = tz;
				s_stShipPriv.activePools[s_stShipPriv.poolCount].poolSize = 1;
				s_stShipPriv.poolCount++;
			}
		}

		//////////////////////////////提取聚类池有效目标并截断//////////////////////////////
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
			int keepNum = (2 < tzPredCount) ? 2 : tzPredCount;
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
			s_stShipPriv.stPredResult.tDown1 = -1;
			s_stShipPriv.stPredResult.tDown2 = -1;
			if (s_stShipPriv.poolCount >= 1)
			{
				s_stShipPriv.stPredResult.tDown1 = s_stShipPriv.closestTz[0];
			}
			if (s_stShipPriv.poolCount >= 2)
			{
				s_stShipPriv.stPredResult.tDown2 = s_stShipPriv.closestTz[1];
			}
		}
#endif
	}
}



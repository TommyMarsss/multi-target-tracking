package main

// kalmanFilter 是匀速（constant-velocity）运动模型的 4 维卡尔曼滤波器，
// 状态为 [px, py, vx, vy]，观测为位置 (px, py)。仅使用标准库，矩阵维度固定，
// 全部运算手工展开，避免引入第三方线性代数库。
type kalmanFilter struct {
	x [4]float64
	P [4][4]float64
}

// 状态转移矩阵 F（dt = 1）。
var fMat = [4][4]float64{
	{1, 0, 1, 0},
	{0, 1, 0, 1},
	{0, 0, 1, 0},
	{0, 0, 0, 1},
}

// newKalman 以首个观测初始化滤波器。位置方差取测量噪声方差，
// 速度未知，给较大初始方差使其能快速从后续观测中学习。
func newKalman(z Point, measVar float64) kalmanFilter {
	k := kalmanFilter{}
	k.x = [4]float64{z.X, z.Y, 0, 0}
	k.P = [4][4]float64{
		{measVar, 0, 0, 0},
		{0, measVar, 0, 0},
		{0, 0, 100, 0},
		{0, 0, 0, 100},
	}
	return k
}

// qMat 是离散白噪声加速度模型的过程噪声协方差（dt = 1），q 为加速度谱密度。
func qMat(q float64) [4][4]float64 {
	return [4][4]float64{
		{0.25 * q, 0, 0.5 * q, 0},
		{0, 0.25 * q, 0, 0.5 * q},
		{0.5 * q, 0, q, 0},
		{0, 0.5 * q, 0, q},
	}
}

func mul4(a, b [4][4]float64) (c [4][4]float64) {
	for i := 0; i < 4; i++ {
		for j := 0; j < 4; j++ {
			s := 0.0
			for k := 0; k < 4; k++ {
				s += a[i][k] * b[k][j]
			}
			c[i][j] = s
		}
	}
	return c
}

func transpose4(a [4][4]float64) (b [4][4]float64) {
	for i := 0; i < 4; i++ {
		for j := 0; j < 4; j++ {
			b[i][j] = a[j][i]
		}
	}
	return b
}

// predict 执行时间更新：x = F x, P = F P Fᵀ + Q。
func (k *kalmanFilter) predict(q float64) {
	k.x = [4]float64{k.x[0] + k.x[2], k.x[1] + k.x[3], k.x[2], k.x[3]}
	P := mul4(mul4(fMat, k.P), transpose4(fMat))
	Q := qMat(q)
	for i := 0; i < 4; i++ {
		for j := 0; j < 4; j++ {
			k.P[i][j] = P[i][j] + Q[i][j]
		}
	}
}

// innovationS 返回新息协方差 S = H P Hᵀ + R（H = [I₂ 0]，R = measVar·I₂）。
func (k *kalmanFilter) innovationS(measVar float64) (s [2][2]float64) {
	s[0][0] = k.P[0][0] + measVar
	s[0][1] = k.P[0][1]
	s[1][0] = k.P[1][0]
	s[1][1] = k.P[1][1] + measVar
	return s
}

func inv2(s [2][2]float64) [2][2]float64 {
	det := s[0][0]*s[1][1] - s[0][1]*s[1][0]
	return [2][2]float64{
		{s[1][1] / det, -s[0][1] / det},
		{-s[1][0] / det, s[0][0] / det},
	}
}

// mahalanobis2 返回观测 z 相对当前（预测）状态的平方马氏距离 yᵀS⁻¹y，
// 用作关联代价与门控依据：它按滤波器自身不确定度归一化，
// 因此对观测噪声与预测不确定度都具有自适应性。
func (k *kalmanFilter) mahalanobis2(z Point, measVar float64) float64 {
	return mahalanobis2WithS(k, z, k.innovationS(measVar))
}

// mahalanobis2WithS 用给定的新息协方差 S 计算平方马氏距离。
// 遮挡滑行期间调用方会传入冻结的 S，防止门控区域随滑行时间无限膨胀。
func mahalanobis2WithS(k *kalmanFilter, z Point, s [2][2]float64) float64 {
	y0 := z.X - k.x[0]
	y1 := z.Y - k.x[1]
	si := inv2(s)
	return y0*(si[0][0]*y0+si[0][1]*y1) + y1*(si[1][0]*y0+si[1][1]*y1)
}

// update 执行量测更新（标准卡尔曼增益形式，H = [I₂ 0]）。
func (k *kalmanFilter) update(z Point, measVar float64) {
	y0 := z.X - k.x[0]
	y1 := z.Y - k.x[1]
	si := inv2(k.innovationS(measVar))
	var K [4][2]float64
	for i := 0; i < 4; i++ {
		K[i][0] = k.P[i][0]*si[0][0] + k.P[i][1]*si[1][0]
		K[i][1] = k.P[i][0]*si[0][1] + k.P[i][1]*si[1][1]
	}
	for i := 0; i < 4; i++ {
		k.x[i] += K[i][0]*y0 + K[i][1]*y1
	}
	old := k.P
	for i := 0; i < 4; i++ {
		for j := 0; j < 4; j++ {
			k.P[i][j] = old[i][j] - K[i][0]*old[0][j] - K[i][1]*old[1][j]
		}
	}
}

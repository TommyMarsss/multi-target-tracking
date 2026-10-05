package tracker

// Point 是二维平面上的一个观测点（传感器量测）。
type Point struct {
	X, Y float64
}

// kfState 是匀速模型（Constant Velocity）卡尔曼滤波器状态。
// 状态向量 x = [px, py, vx, vy]，帧间隔 dt = 1。
type kfState struct {
	x [4]float64
	p [4][4]float64
}

// fMat 为状态转移矩阵 F（dt = 1）。
var fMat = [4][4]float64{
	{1, 0, 1, 0},
	{0, 1, 0, 1},
	{0, 0, 1, 0},
	{0, 0, 0, 1},
}

// qMat 为离散白噪声加速度模型下的过程噪声协方差 Q。
func qMat(q float64) [4][4]float64 {
	return [4][4]float64{
		{0.25 * q, 0, 0.5 * q, 0},
		{0, 0.25 * q, 0, 0.5 * q},
		{0.5 * q, 0, q, 0},
		{0, 0.5 * q, 0, q},
	}
}

// predict 执行一步预测：x = F x，P = F P F' + Q。
func (k *kfState) predict(q float64) {
	k.x = [4]float64{k.x[0] + k.x[2], k.x[1] + k.x[3], k.x[2], k.x[3]}
	var fp [4][4]float64
	for i := 0; i < 4; i++ {
		for j := 0; j < 4; j++ {
			fp[i][j] = fMat[i][0]*k.p[0][j] + fMat[i][1]*k.p[1][j] +
				fMat[i][2]*k.p[2][j] + fMat[i][3]*k.p[3][j]
		}
	}
	qm := qMat(q)
	var np [4][4]float64
	for i := 0; i < 4; i++ {
		for j := 0; j < 4; j++ {
			np[i][j] = fp[i][0]*fMat[j][0] + fp[i][1]*fMat[j][1] +
				fp[i][2]*fMat[j][2] + fp[i][3]*fMat[j][3] + qm[i][j]
		}
	}
	k.p = np
}

// innovation 返回新息 y = z - Hx 与新息协方差 S = H P H' + R，其中 H = [I2 0]。
func (k *kfState) innovation(z Point, r float64) (y [2]float64, s [2][2]float64) {
	y = [2]float64{z.X - k.x[0], z.Y - k.x[1]}
	s = [2][2]float64{
		{k.p[0][0] + r, k.p[0][1]},
		{k.p[1][0], k.p[1][1] + r},
	}
	return
}

// inv2 求 2x2 矩阵的逆。
func inv2(s [2][2]float64) [2][2]float64 {
	det := s[0][0]*s[1][1] - s[0][1]*s[1][0]
	return [2][2]float64{
		{s[1][1] / det, -s[0][1] / det},
		{-s[1][0] / det, s[0][0] / det},
	}
}

// mahalanobis 计算 y' S^-1 y（马氏距离平方），作为观测-航迹关联代价与门控依据。
func mahalanobis(y [2]float64, sInv [2][2]float64) float64 {
	return y[0]*(sInv[0][0]*y[0]+sInv[0][1]*y[1]) +
		y[1]*(sInv[1][0]*y[0]+sInv[1][1]*y[1])
}

// update 以观测 z 执行卡尔曼校正。
func (k *kfState) update(z Point, r float64) {
	y, s := k.innovation(z, r)
	si := inv2(s)
	// K = P H' S^-1，H = [I2 0]，故 P H' 为 P 的前两列。
	var gain [4][2]float64
	for i := 0; i < 4; i++ {
		gain[i][0] = k.p[i][0]*si[0][0] + k.p[i][1]*si[1][0]
		gain[i][1] = k.p[i][0]*si[0][1] + k.p[i][1]*si[1][1]
	}
	for i := 0; i < 4; i++ {
		k.x[i] += gain[i][0]*y[0] + gain[i][1]*y[1]
	}
	// P = (I - K H) P = P - K (H P)，H P 为 P 的前两行。
	var np [4][4]float64
	for i := 0; i < 4; i++ {
		for j := 0; j < 4; j++ {
			np[i][j] = k.p[i][j] - gain[i][0]*k.p[0][j] - gain[i][1]*k.p[1][j]
		}
	}
	k.p = np
}

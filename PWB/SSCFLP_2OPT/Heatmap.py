import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import gaussian_kde

# 1. Uniform 분포에서 30개 2D 샘플 생성
np.random.seed(13)
x = np.random.uniform(0, 1, 30)
y = np.random.uniform(0, 1, 30)
xy = np.vstack([x, y])  # shape: (2, 30)

# 2. KDE 수행
kde = gaussian_kde(xy)

# 3. 그리드 생성 및 밀도 추정
xgrid, ygrid = np.mgrid[0:1:100j, 0:1:100j]
positions = np.vstack([xgrid.ravel(), ygrid.ravel()])
density = kde(positions).reshape(100, 100)

# 4. 히트맵 시각화
plt.figure(figsize=(6, 6))
plt.imshow(density.T, origin='lower', extent=[0, 1, 0, 1], cmap='hot', aspect='auto')
plt.colorbar(label='Density')
plt.scatter(x, y, c='blue', s=30, edgecolor='white', label='Points')
plt.title("2D KDE Heatmap from 30 Uniform Samples")
plt.xlabel("x")
plt.ylabel("y")
plt.legend()
plt.tight_layout()
plt.show()

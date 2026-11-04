"""GMR：GMM 拟合 + 高斯混合回归 + 三种采样器。

对应论文 Section III-A，式 (1)-(6)：

    式(1)   联合 GMM：p(xi) = sum_k pi_k * N(xi; mu_k, Sigma_k)
    式(2)   分块：  mu_k = {mu_t,k, mu_s,k},  Sigma_k = [[S_tt, S_ts], [S_st, S_ss]]
    式(3)   条件分布：mu_hat = mu_s + S_st * S_tt^-1 (t - mu_t)
                     Sigma_hat = S_ss - S_st * S_tt^-1 * S_ts
    式(5)   责任度：beta_k = pi_k N(t; mu_t,k, S_tt,k) / sum_j pi_j N(t; ...)
    式(6)   合并：  mu_hat = sum_k beta_k * mu_hat_k
                   Sigma_hat = sum_k beta_k^2 * Sigma_hat_k      <-- 注意是 beta 的平方
"""

import numpy as np
from sklearn.mixture import GaussianMixture

N_COMPONENTS = 8     # 论文取 K=8


# ---------------------------------------------------------------- GMM / GMR

def fit_gmm(demos, n_components=N_COMPONENTS, seed=0, reg_covar=1e-6):
    """在演示数据上拟合联合 GMM。

    数据组织：每条演示按时间展开为 [t, x, y]，t = 1..N（与论文一致，不归一化）。
    返回 (gmm, N)。
    """
    n = demos[0].shape[0]
    blocks = []
    for d in demos:
        t = np.arange(1, n + 1, dtype=float)[:, None]
        blocks.append(np.hstack([t, np.asarray(d, dtype=float)]))
    x = np.vstack(blocks)

    gmm = GaussianMixture(n_components=n_components, covariance_type="full",
                          reg_covar=reg_covar, n_init=5, random_state=seed)
    gmm.fit(x)
    return gmm, n


def _log_gauss_1d(t, mu, var):
    var = max(float(var), 1e-12)
    return -0.5 * np.log(2.0 * np.pi * var) - 0.5 * (t - mu) ** 2 / var


def gmr_predict(gmm, n_points):
    """式(3)-(6)：逐时刻回归出空间均值与协方差。

    返回 mu (N,2)，sigma (N,2,2)。
    """
    w = gmm.weights_
    means = gmm.means_
    covs = gmm.covariances_
    k = len(w)

    mu_out = np.zeros((n_points, 2))
    sig_out = np.zeros((n_points, 2, 2))

    for i in range(n_points):
        t = float(i + 1)
        log_beta = np.empty(k)
        mu_k = np.zeros((k, 2))
        sig_k = np.zeros((k, 2, 2))
        for j in range(k):
            mu_t = means[j, 0]
            mu_s = means[j, 1:]
            var_t = covs[j, 0, 0]
            cov_ts = covs[j, 0, 1:]
            cov_ss = covs[j, 1:, 1:]
            # 式(3)
            mu_k[j] = mu_s + cov_ts * (t - mu_t) / var_t
            sig_k[j] = cov_ss - np.outer(cov_ts, cov_ts) / var_t
            # 式(5) 用对数域计算，避免下溢
            log_beta[j] = np.log(w[j]) + _log_gauss_1d(t, mu_t, var_t)
        log_beta -= log_beta.max()
        beta = np.exp(log_beta)
        beta /= beta.sum()
        # 式(6)
        mu_out[i] = (beta[:, None] * mu_k).sum(axis=0)
        sig_out[i] = ((beta ** 2)[:, None, None] * sig_k).sum(axis=0)

    return mu_out, sig_out


def cov_sqrt(s, eps=1e-6):
    """预计算协方差平方根 L（满足 L @ L.T = Sigma），用于快速采样。

    比每步调用 multivariate_normal 快得多：sample = mu + L @ randn(2)。
    """
    vals, vecs = np.linalg.eigh(0.5 * (s + s.T))
    vals = np.maximum(vals, eps)
    return vecs * np.sqrt(vals)


# ---------------------------------------------------------------- 采样器

class UniformSampler:
    """基线：均匀采样。"""

    name = "RRT*"

    def __init__(self, env, rng):
        self.env = env
        self.rng = rng

    def sample(self):
        return self.rng.uniform(0.0, self.env.size, size=2)


class GMMSampler:
    """GMM-RRT*：直接从 GMM 的空间边缘分布采样（随机选分量再采样）。"""

    name = "GMM-RRT*"

    def __init__(self, gmm, rng):
        self.cum = np.cumsum(gmm.weights_)
        self.cum[-1] = 1.0
        self.mu = gmm.means_[:, 1:]
        self.l = np.stack([cov_sqrt(c) for c in gmm.covariances_[:, 1:, 1:]])
        self.rng = rng

    def sample(self):
        k = int(np.searchsorted(self.cum, self.rng.random()))
        k = min(k, len(self.cum) - 1)
        return self.mu[k] + self.l[k] @ self.rng.standard_normal(2)


class GMRSampler:
    """GMR-RRT*，论文 Algorithm 2。

    Random(1) > 0.5 -> 均匀采样（保留探索能力，避免掉进陷阱）
    否则：随机取时刻 t，从条件高斯 N(mu_hat_t, Sigma_hat_t) 采样。
    """

    name = "GMR-RRT*"

    def __init__(self, mu, sigma, env, rng, p_uniform=0.5):
        self.mu = mu
        self.l = np.stack([cov_sqrt(s) for s in sigma])
        self.env = env
        self.rng = rng
        self.p_uniform = p_uniform

    def sample(self):
        if self.rng.random() > self.p_uniform:
            return self.rng.uniform(0.0, self.env.size, size=2)
        k = int(self.rng.integers(0, self.mu.shape[0]))
        return self.mu[k] + self.l[k] @ self.rng.standard_normal(2)

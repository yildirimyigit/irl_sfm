import torch
import numpy as np
from scipy.linalg import sqrtm


def gradient_penalty(D, x_true, x_fake, derivative, device):
    if len(x_true.shape) == 2:
        alpha = torch.rand(x_true.size()[0], 1, device=device)
    else:
        alpha = torch.rand(x_true.size()[0], 1, 1, 1, device=device)

    alpha = alpha.expand(x_true.size())
    interpolates = alpha * x_true + (1-alpha) * x_fake
    interpolates = torch.autograd.Variable(interpolates, requires_grad=True)
    disc_interpolates = D(interpolates)
    gradients = torch.autograd.grad(outputs=disc_interpolates,
                                    inputs=interpolates,
                                    grad_outputs=torch.ones(disc_interpolates.size(), device=device),
                                    create_graph=True,
                                    retain_graph=True,
                                    only_inputs=True)[0]
    gradient_penalty = ((gradients.norm(2, dim=1)-derivative) ** 2).mean() * 10
    return gradient_penalty


def FID_score(x_real, x_fake):
    mu_real = x_real.mean(dim=0)
    mu_fake = x_fake.mean(dim=0)
    cov_real = np.cov(x_real, rowvar=False)
    cov_fake = np.cov(x_fake, rowvar=False)
    mu_diff = np.linalg.norm(mu_real-mu_fake, 2) ** 2
    covmean = 2 * sqrtm(np.matmul(cov_real, cov_fake.T))
    cov_diff = np.trace(cov_real + cov_fake - covmean)
    return mu_diff + cov_diff.real


def nn_accuracy(p_fake, p_real, device=torch.device('cpu'), k=5):
    size = p_fake.shape[0]
    p_fake = p_fake.view(size, -1).to(device)
    p_real = p_real.view(size, -1).to(device)
    p_all = torch.cat([p_fake, p_real], dim=0)
    dists = torch.cdist(p_all, p_all) + torch.eye(2*size, device=device) * 1e12
    values, indexes = torch.topk(dists, k=k, largest=False)

    decisions = (indexes > size-1).sum(dim=1).float() / k
    fake_acc = (size - decisions[:size].sum()).float() / size
    real_acc = (decisions[size:].sum()).float() / size
    return fake_acc.item(), real_acc.item()


def switch_grads(model, mask):
    for p in model.parameters():
        p.requires_grad = mask

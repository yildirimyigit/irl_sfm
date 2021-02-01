import argparse
import os

import torch
import numpy as np

from data import Dataset
from gan import GAN


parser = argparse.ArgumentParser(description="Train GAN.")
parser.add_argument("-p", help="Dataset path.", type=str, required=True)
parser.add_argument("-z_dim", help="Noise dimension.", type=int, required=True)
parser.add_argument("-w", help="0: GAN 1: WGAN", type=int, required=True)
parser.add_argument("-bs", help="Batch size.", type=int, required=True)
parser.add_argument("-lr", help="Learning rate. Default 0.0001.", type=float, default=0.0001)
parser.add_argument("-e", help="Epoch.", type=int, required=True)
parser.add_argument("-d", help="Depth of D and G. Default 2.", type=int, default=2)
parser.add_argument("-n", help="Hidden units. Default 128.", type=int, default=128)
parser.add_argument("-o", help="Output path.", type=str, required=True)
args = parser.parse_args()

if os.path.exists(args.o):
    os.system("rm -rf %s" % args.o)
os.makedirs(args.o)

WASSERSTEIN = True if args.w == 1 else False
C_ITER = 5 if WASSERSTEIN else 1

x = np.load(args.p)
x = torch.tensor(x, dtype=torch.float)
out_dim = x.shape[-1]
x = x.reshape(-1, out_dim)
dataset = Dataset(x)

if torch.cuda.is_available():
    dv = "cuda"
else:
    dv = "cpu"

g_layers = [torch.nn.Linear(args.z_dim, args.n), torch.nn.ReLU()]
d_layers = [torch.nn.Linear(out_dim, args.n), torch.nn.ReLU()]
for _ in range(args.d):
    g_layers.append(torch.nn.Linear(args.n, args.n))
    d_layers.append(torch.nn.Linear(args.n, args.n))
    g_layers.append(torch.nn.ReLU())
    d_layers.append(torch.nn.ReLU())
g_layers.append(torch.nn.Linear(args.n, out_dim))
d_layers.append(torch.nn.Linear(args.n, 1))

g = torch.nn.Sequential(*g_layers).to(dv)
d = torch.nn.Sequential(*d_layers).to(dv)

gan = GAN(g, d, args.z_dim, lr=args.lr, wasserstein=WASSERSTEIN, device=dv)
print(gan)

loader = torch.utils.data.DataLoader(dataset, batch_size=args.bs, shuffle=True)
gan.train(loader, args.e, C_ITER, verbose=True)
gan.save(os.path.join(args.o, "epoch%d" % args.e))

with torch.no_grad():
    v_all = gan.discriminate(x.to(dv)).cpu()
print("V min: %.5f - V max: %.5f" % (v_all.min(), v_all.max()))

import argparse
import os

import rospy
from std_msgs.msg import Float32MultiArray, Float32
import torch

from gan import GAN


def state_callback(msg):
    global gan, state_pub
    data = torch.tensor(msg.data, dtype=torch.float)
    with torch.no_grad():
        value = gan.discriminate(data)
    res = Float32()
    res.data = value
    state_pub.publish(res)


parser = argparse.ArgumentParser(description="Train GAN.")
parser.add_argument("-z_dim", help="Noise dimension.", type=int, required=True)
parser.add_argument("-o_dim", help="Output dimension.", type=int, required=True)
parser.add_argument("-w", help="0: GAN 1: WGAN", type=int, required=True)
parser.add_argument("-d", help="Depth of D and G. Default 2.", type=int, default=2)
parser.add_argument("-n", help="Hidden units. Default 128.", type=int, default=128)
parser.add_argument("-o", help="Model path.", type=str, required=True)
args = parser.parse_args()

if os.path.exists(args.o):
    os.system("rm -rf %s" % args.o)
os.makedirs(args.o)

WASSERSTEIN = True if args.w == 1 else False
C_ITER = 5 if WASSERSTEIN else 1
dv = "cpu"
out_dim = args.o_dim

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

gan = GAN(g, d, args.z_dim, lr=0.0001, wasserstein=WASSERSTEIN, device=dv)
gan.load(args.o)
print(gan)


state_sub = rospy.Subscriber("/state_sub", Float32MultiArray, callback=state_callback)
state_pub = rospy.Publisher("/gan_output", Float32, queue_size=10)

rospy.init_node("gan_node")
rospy.spin()
